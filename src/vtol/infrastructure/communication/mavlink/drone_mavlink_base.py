import math
import time
from abc import abstractmethod
from dataclasses import replace
from threading import RLock
from typing import Optional

from loguru import logger
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from vtol.domain.drone import Drone, DroneMode, DroneStatus, LandedState, VtolState, HoldPosition
from vtol.domain.mission.execution_status import MissionExecutionStatus
from vtol.domain.models import Pose3D
from vtol.domain.rigid_transform import RigidTransform

from .mavlink_connection_params import MavlinkConnectionParams


class DroneMavlinkBase(Drone):
    HEARTBEAT_INTERVAL_S = 1.0
    MAX_MESSAGES_PER_POLL = 100
    RECEIVE_BUDGET_S = 0.005

    def __init__(self, params: MavlinkConnectionParams):
        """

        :param params:
        """
        self.parameters = params
        self.connection: Optional[mavutil.mavfile] = None
        self._lock = RLock()
        self._target_system: int | None = None
        self._target_component: int | None = None
        self._last_sent_heartbeat_s: float | None = None
        self._flight_parameters: dict[str, float] = {}
        self._parameter_requests: dict[str, float] = {}
        self._yaw_telemetry_requested = False
        self._yaw_channel: int | None = None
        self._link_was_alive = False
        self._mission_execution_status = MissionExecutionStatus()
        self._hold_requested = False
        self._hold_accepted: bool | None = None
        self.status: DroneStatus = DroneStatus(
            DroneMode.UNKNOWN,
            groundspeed_mps=0.0,
            battery_voltage_v=0.0,
            battery_remaining_pct=0,
            gps_fix_type=0,
            armed=False,
            last_heartbeat_s=0.0,
            last_signal_gpio_s=0.0,
            alt_m=0.0,
            speed=0.0,
            relative_altitude=0.0,
            relative_altitude_ms=0.0,
            latitude=0.0,
            longitude=0.0,
            heading_deg=0.0,
        )

    @abstractmethod
    def _init_mavlink_connection(self) -> None:
        raise NotImplementedError()

    def notify_gpio_signal(self, now_s: Optional[float] = None) -> None:
        """Appelle ça depuis ton handler GPIO quand le pin s'active."""
        self.status.last_signal_gpio_s = time.time() if now_s is None else float(now_s)

    def connect(self) -> None:
        with self._lock:
            if self.connection is not None:
                raise RuntimeError("MAVLink connection is already open")
            logger.info(
                "Opening MAVLink connection to {} (timeout {}s)", self.parameters.address, self.parameters.timeout
            )
            try:
                self._init_mavlink_connection()
                self._configure_mavlink2()
                heartbeat_received = self._wait_for_heartbeat()
                self._send_heartbeat()
                if heartbeat_received:
                    self._request_status_streams()
                    self._link_was_alive = True
                    logger.info(
                        "MAVLink autopilot connected: system {}, component {}",
                        self._target_system,
                        self._target_component,
                    )
                else:
                    logger.warning(
                        "No autopilot heartbeat within {}s; continuing without FC (fail_on_timeout=false). "
                        "MAVLink discovery remains active",
                        self.parameters.timeout,
                    )
            except BaseException:
                try:
                    self.close()
                except Exception:
                    logger.exception("Failed to close MAVLink transport after connection failure")
                raise

    def close(self) -> None:
        with self._lock:
            try:
                self.release_yaw_control()
            except Exception:
                logger.exception("Could not release yaw override while closing the MAVLink transport")
                self._yaw_channel = None
            connection, self.connection = self.connection, None
            self.status.last_heartbeat_s = 0.0
            self.status.last_heartbeat_monotonic_s = None
            self.status.landed_state = LandedState.UNKNOWN
            self.status.vtol_state = VtolState.UNKNOWN
            self._mission_execution_status = MissionExecutionStatus()
            self._target_system = self._target_component = None
            self._last_sent_heartbeat_s = None
            self._flight_parameters.clear()
            self._parameter_requests.clear()
            self._yaw_telemetry_requested = False
            self._link_was_alive = False
            self._hold_requested = False
            self._hold_accepted = None
            if connection is not None:
                connection.close()
                logger.info("MAVLink connection closed")

    def _configure_mavlink2(self) -> None:
        """Select the wire encoder per connection, independent of import order/env."""
        self._require_connected()
        self.connection.mav = mavlink2.MAVLink(
            self.connection,
            srcSystem=self.connection.source_system,
            srcComponent=self.connection.source_component,
        )
        self.connection.mav.robust_parsing = True
        self.connection.WIRE_PROTOCOL_VERSION = "2.0"

    def get_status(self) -> DroneStatus:
        with self._lock:
            self._update_status()
            self._maintain_heartbeat()
            alive = self.status.connected
            if alive != self._link_was_alive:
                if alive:
                    logger.info("MAVLink heartbeat recovered")
                    self._request_status_streams()
                else:
                    logger.warning("MAVLink heartbeat expired; vehicle connection lost")
                    self._mission_execution_status = MissionExecutionStatus()
                    self.status.landed_state = LandedState.UNKNOWN
                    self.status.vtol_state = VtolState.UNKNOWN
                    self._target_system = self._target_component = None
                    self.connection.target_system = None
                    self.connection.target_component = None
                self._link_was_alive = alive
            return replace(self.status)

    def get_mission_execution_status(self) -> MissionExecutionStatus:
        with self._lock:
            return self._mission_execution_status

    def send_landing_target(
        self,
        position: Pose3D,
        target_size: tuple[float, float],
        *,
        body_from_marker: RigidTransform | None = None,
        captured_at_s: float | None = None,
    ) -> None:
        with self._lock:
            self._send_landing_target(position, target_size, body_from_marker, captured_at_s)

    def _request_parameter(self, name: str) -> None:
        now = time.monotonic()
        if name not in self._flight_parameters and now - self._parameter_requests.get(name, -math.inf) >= 1.0:
            self.connection.mav.param_request_read_send(
                self._target_system, self._target_component, name.encode("ascii"), -1
            )
            self._parameter_requests[name] = now

    def prepare_yaw_control(self) -> bool:
        """Read actual FC settings without blocking the telemetry receiver.

        QuadPlane's VTOL controller consumes RC yaw in GUIDED when stick
        mixing is enabled. SET_ATTITUDE_TARGET's fixed-wing handler does not
        provide this VTOL yaw control path.
        """
        with self._lock:
            self._require_connected()
            if not self.status.connected:
                return False
            if not self._yaw_telemetry_requested:
                self.connection.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavlink2.MAVLINK_MSG_ID_ATTITUDE,
                    100_000,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                self._yaw_telemetry_requested = True
            required = (
                "Q_ENABLE",
                "Q_GUIDED_MODE",
                "STICK_MIXING",
                "RCMAP_YAW",
                "RC_OVERRIDE_TIME",
                "RC_OPTIONS",
                "Q_WVANE_ENABLE",
            )
            for name in (*required, "MAV_GCS_SYSID", "SYSID_MYGCS"):
                self._request_parameter(name)
            params = self._flight_parameters
            if any(name not in params for name in required):
                return False
            gcs_id = params.get("MAV_GCS_SYSID", params.get("SYSID_MYGCS"))
            if gcs_id is None:
                return False
            if not all(math.isfinite(params[name]) for name in required) or not math.isfinite(gcs_id):
                raise ValueError("QuadPlane parameters must be finite")
            if params["Q_ENABLE"] <= 0 or params["Q_GUIDED_MODE"] != 1:
                raise ValueError("Yaw alignment requires Q_ENABLE > 0 and Q_GUIDED_MODE = 1")
            if params["STICK_MIXING"] == 0:
                raise ValueError("GUIDED VTOL RC yaw requires STICK_MIXING enabled")
            if params["Q_WVANE_ENABLE"] != 0:
                raise ValueError("Marker yaw alignment requires Q_WVANE_ENABLE = 0")
            if not 0 < params["RC_OVERRIDE_TIME"] <= 1 or int(params["RC_OPTIONS"]) & 2:
                raise ValueError("RC overrides must be enabled with 0 < RC_OVERRIDE_TIME <= 1 second")
            if int(gcs_id) != self.connection.source_system:
                raise ValueError("Companion system ID must match MAV_GCS_SYSID / SYSID_MYGCS for RC yaw")
            channel = int(params["RCMAP_YAW"])
            if channel != params["RCMAP_YAW"] or not 1 <= channel <= 16:
                raise ValueError("RCMAP_YAW must identify an ArduPilot override channel in [1, 16]")
            names = tuple(f"RC{channel}_{suffix}" for suffix in ("MIN", "TRIM", "MAX", "REVERSED", "DZ"))
            for name in names:
                self._request_parameter(name)
            if any(name not in params for name in names):
                return False
            minimum, trim, maximum, reversed_, deadzone = (params[name] for name in names)
            if not all(math.isfinite(params[name]) for name in names):
                raise ValueError("RC yaw calibration must be finite")
            if not (800 <= minimum < trim < maximum <= 2200 and 0 <= deadzone < min(trim - minimum, maximum - trim)):
                raise ValueError("Invalid RC yaw MIN/TRIM/MAX/DZ calibration")
            if reversed_ not in (0, 1):
                raise ValueError("RC yaw REVERSED must be zero or one")
            if self._yaw_channel is not None and self._yaw_channel != channel:
                raise ValueError("RCMAP_YAW changed during active yaw control")
            return True

    def request_mode(self, mode: DroneMode, *, expected_modes: tuple[DroneMode, ...] | None = None) -> bool:
        with self._lock:
            self._require_connected()
            # Drain newly received failsafe/mode heartbeats under the same lock
            # as transmission, rather than trusting an older supervisor snapshot.
            self._update_status()
            if not self.status.connected:
                raise RuntimeError("Cannot change mode without a fresh autopilot heartbeat")
            if expected_modes is not None and self.status.mode not in expected_modes:
                return False
            name = "LOITERALTQLAND" if mode is DroneMode.LOITER_ALT_QLAND else mode.value
            mapping = self.connection.mode_mapping() or {}
            if name not in mapping:
                raise ValueError(f"Flight controller does not advertise mode {mode.value}")
            self.connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavlink2.MAV_CMD_DO_SET_MODE,
                0,
                mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[name],
                0,
                0,
                0,
                0,
                0,
            )
            return True

    def request_position_hold(self, position: HoldPosition) -> bool:
        with self._lock:
            self._require_connected()
            self._update_status()
            if not self.status.connected or self.status.mode is not DroneMode.GUIDED:
                return False
            self._hold_requested = True
            self._hold_accepted = None
            # ArduPlane consumes DO_REPOSITION for all three position axes.
            # Flags=0 also rejects the command if an external mode wins the race.
            self.connection.mav.command_int_send(
                self._target_system,
                self._target_component,
                mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavlink2.MAV_CMD_DO_REPOSITION,
                0,
                0,
                -1,
                0,
                0,
                float("nan"),
                round(position.latitude_deg * 1e7),
                round(position.longitude_deg * 1e7),
                position.relative_altitude_m,
            )
            return True

    def position_hold_accepted(self) -> bool | None:
        with self._lock:
            return self._hold_accepted

    def send_yaw_input(self, value: float) -> None:
        with self._lock:
            if not math.isfinite(value) or not -1 <= value <= 1:
                raise ValueError("Yaw input must be finite and in [-1, 1]")
            if not self.prepare_yaw_control():
                raise RuntimeError("QuadPlane yaw control is not ready")
            if self.status.mode is not DroneMode.GUIDED or not self.status.armed:
                raise RuntimeError("Yaw control requires armed GUIDED VTOL")
            channel = int(self._flight_parameters["RCMAP_YAW"])
            params = self._flight_parameters
            minimum, trim, maximum, reversed_, deadzone = (
                params[f"RC{channel}_{suffix}"] for suffix in ("MIN", "TRIM", "MAX", "REVERSED", "DZ")
            )
            if reversed_:
                value = -value
            pwm = trim
            if value > 0:
                pwm += deadzone + value * (maximum - trim - deadzone)
            elif value < 0:
                pwm -= deadzone + abs(value) * (trim - minimum - deadzone)
            channels = [65535] * 18  # Ignore every channel except the mapped yaw input.
            channels[channel - 1] = round(pwm)
            self._yaw_channel = channel
            self.connection.mav.rc_channels_override_send(self._target_system, self._target_component, *channels)

    def release_yaw_control(self) -> None:
        with self._lock:
            if self._yaw_channel is None:
                return
            if self.connection is not None:
                channels = [65535] * 18
                # MAVLink extension channels 9..18 have a different release sentinel.
                channels[self._yaw_channel - 1] = 0 if self._yaw_channel <= 8 else 65534
                self.connection.mav.rc_channels_override_send(self._target_system, self._target_component, *channels)
            self._yaw_channel = None

    def _send_landing_target(
        self,
        uav_pose: Pose3D,
        target_size: tuple[float, float],
        body_from_marker: RigidTransform | None,
        captured_at_s: float | None,
    ) -> None:
        """Send the MAVLink 2 position branch in BODY_FRD coordinates (in meters).

        Image angles/sizes cannot be recovered from a body-frame translation
        for an arbitrary camera mounting. Leave these unused legacy fields at
        zero; ArduPilot consumes x/y/z and distance when position_valid is one.
        ``target_size`` remains in the domain API but is not an angular size.
        """
        self._require_connected()
        distance = math.hypot(uav_pose.x, uav_pose.y, uav_pose.z)
        if not math.isfinite(distance) or uav_pose.z <= 0:
            raise ValueError("Landing target must be finite and below the vehicle (BODY_FRD z > 0)")

        quaternion = [0.0, 0.0, 0.0, 0.0]
        if body_from_marker is not None:
            if body_from_marker.position != uav_pose:
                raise ValueError("Landing position and orientation must come from the same transform")
            if captured_at_s is None:
                raise ValueError("Oriented landing targets require a capture timestamp")
            quaternion = body_from_marker.quaternion_wxyz()
        timestamp = time.monotonic() if captured_at_s is None else captured_at_s
        if not math.isfinite(timestamp) or timestamp < 0 or timestamp >= 2**63 / 1_000_000:
            raise ValueError("Landing target capture timestamp is invalid")

        self.connection.mav.landing_target_send(
            int(timestamp * 1_000_000),  # time_usec
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
            0.0,  # angle_x: not provided by the position branch
            0.0,  # angle_y
            distance,  # distance
            0.0,  # size_x: angular size unknown
            0.0,  # size_y
            uav_pose.x,  # x
            uav_pose.y,  # y
            uav_pose.z,  # z
            quaternion,  # marker-to-BODY_FRD rotation, w/x/y/z; all-zero means unknown
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            1,  # position_valid
        )

    def _require_connected(self) -> None:
        if self.connection is None:
            raise RuntimeError("Drone not connected. Call must be connected first.")

    def _update_status(self) -> None:
        self._require_connected()
        deadline = time.monotonic() + self.RECEIVE_BUDGET_S
        for _ in range(self.MAX_MESSAGES_PER_POLL):
            received_message = self.connection.recv_match(blocking=False)
            if received_message is None:
                break
            if self._target_system is None:
                self._accept_autopilot_heartbeat(received_message)
            elif self._is_from_autopilot(received_message):
                self._handle_message(received_message)
            if time.monotonic() >= deadline:
                break

    def _is_from_autopilot(self, message) -> bool:
        return (message.get_srcSystem(), message.get_srcComponent()) == (self._target_system, self._target_component)

    def _handle_message(self, message) -> None:
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self.status.mode_name = mavutil.mode_string_v10(message)
            self.status.mode = DroneMode.from_str(self.status.mode_name)
            self.status.armed = bool(message.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
            self.status.last_heartbeat_s = time.time()
            self.status.last_heartbeat_monotonic_s = time.monotonic()
        elif message_type == "VFR_HUD":
            self.status.alt_m = float(message.alt)
            self.status.groundspeed_mps = float(message.groundspeed)
        elif message_type == "SYS_STATUS":
            self.status.battery_voltage_v = float(message.voltage_battery) / 1000.0
            self.status.battery_remaining_pct = int(message.battery_remaining)
        elif message_type == "GPS_RAW_INT":
            self.status.gps_fix_type = int(message.fix_type)
        elif message_type == "GLOBAL_POSITION_INT":
            self.status.position_timestamp_s = time.monotonic()
            self.status.latitude = float(message.lat) / 1e7
            self.status.longitude = float(message.lon) / 1e7
            self.status.relative_altitude_ms = float(message.alt) / 1000.0
            self.status.relative_altitude = float(message.relative_alt) / 1000.0
            self.status.speed = float(message.vz) / 100.0
            self.status.groundspeed_mps = math.hypot(message.vx, message.vy) / 100.0
            self.status.heading_deg = None if message.hdg == 65535 else message.hdg / 100.0
        elif message_type == "EXTENDED_SYS_STATE":
            self.status.vtol_state = VtolState.from_mavlink(int(message.vtol_state))
            self.status.landed_state = LandedState.from_mavlink(int(message.landed_state))
        elif message_type == "MISSION_CURRENT":
            self._update_mission_current(message)
        elif message_type == "MISSION_ITEM_REACHED":
            self._update_mission_item_reached(message)
        elif message_type == "ATTITUDE":
            self.status.roll_rad = float(message.roll)
            self.status.pitch_rad = float(message.pitch)
            self.status.yaw_rad = float(message.yaw)
            self.status.attitude_timestamp_s = time.monotonic()
        elif message_type == "PARAM_VALUE":
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode("ascii")
            self._flight_parameters[name.rstrip("\x00")] = float(message.param_value)
        elif message_type == "COMMAND_ACK" and message.command == mavlink2.MAV_CMD_DO_REPOSITION:
            if (
                self._hold_requested
                and message.target_system in (0, self.connection.source_system)
                and message.target_component in (0, self.connection.source_component)
            ):
                if message.result != mavlink2.MAV_RESULT_IN_PROGRESS:
                    self._hold_accepted = message.result == mavlink2.MAV_RESULT_ACCEPTED
        elif message_type == "COMMAND_ACK" and message.command == mavlink2.MAV_CMD_DO_SET_MODE:
            if message.result not in (mavlink2.MAV_RESULT_ACCEPTED, mavlink2.MAV_RESULT_IN_PROGRESS):
                logger.error("Autopilot rejected mode change (MAV_RESULT={})", message.result)
        elif message_type == "COMMAND_ACK" and message.command == mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL:
            if message.result not in (mavlink2.MAV_RESULT_ACCEPTED, mavlink2.MAV_RESULT_IN_PROGRESS):
                logger.warning("Autopilot rejected a telemetry rate request (MAV_RESULT={})", message.result)

    def _update_mission_current(self, message) -> None:
        mission_id_value = getattr(message, "mission_id", None)
        mission_id = int(mission_id_value) if mission_id_value not in (None, 0) else None
        previous = self._mission_execution_status
        if mission_id is not None and previous.mission_id != mission_id:
            previous = MissionExecutionStatus(mission_id=mission_id)
        self._mission_execution_status = MissionExecutionStatus(
            current_item_seq=int(message.seq),
            last_reached_item_seq=previous.last_reached_item_seq,
            last_updated_monotonic_s=time.monotonic(),
            mission_id=mission_id if mission_id is not None else previous.mission_id,
        )

    def _update_mission_item_reached(self, message) -> None:
        previous = self._mission_execution_status
        self._mission_execution_status = MissionExecutionStatus(
            current_item_seq=previous.current_item_seq,
            last_reached_item_seq=int(message.seq),
            last_updated_monotonic_s=time.monotonic(),
            mission_id=previous.mission_id,
        )

    def _maintain_heartbeat(self) -> None:
        if (
            self._last_sent_heartbeat_s is None
            or time.monotonic() - self._last_sent_heartbeat_s >= self.HEARTBEAT_INTERVAL_S
        ):
            self._send_heartbeat()

    def _request_status_streams(self) -> None:
        for message_id, rate_hz in (
            (mavlink2.MAVLINK_MSG_ID_VFR_HUD, 5),
            (mavlink2.MAVLINK_MSG_ID_SYS_STATUS, 1),
            (mavlink2.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
            (mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavlink2.MAVLINK_MSG_ID_MISSION_CURRENT, 1),
            (mavlink2.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2),
        ):
            self.connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                int(1_000_000 / rate_hz),
                0,
                0,
                0,
                0,
                0,
            )
        logger.info(
            "Requested MAVLink telemetry streams (position 10 Hz, HUD 5 Hz, GPS 2 Hz, "
            "mission progress 1 Hz, landed/VTOL state 2 Hz, battery 1 Hz)"
        )

    def _send_heartbeat(self):
        self._require_connected()
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
            mavlink_version=3,  # Protocol-defined HEARTBEAT field, not wire version.
        )
        self._last_sent_heartbeat_s = time.monotonic()

    def _accept_autopilot_heartbeat(self, message) -> bool:
        if (
            message.get_type() != "HEARTBEAT"
            or message.autopilot == mavlink2.MAV_AUTOPILOT_INVALID
            or message.type in (mavlink2.MAV_TYPE_GCS, mavlink2.MAV_TYPE_ONBOARD_CONTROLLER)
            or message.get_srcSystem() == 0
        ):
            return False
        self._target_system = message.get_srcSystem()
        self._target_component = message.get_srcComponent()
        self._mission_execution_status = MissionExecutionStatus()
        self.status.landed_state = LandedState.UNKNOWN
        self.status.vtol_state = VtolState.UNKNOWN
        self.connection.target_system = self._target_system
        self.connection.target_component = self._target_component
        self._handle_message(message)
        return True

    def _wait_for_heartbeat(self) -> bool:
        self._require_connected()
        deadline = time.monotonic() + self.parameters.timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(blocking=True, timeout=min(0.2, max(0, deadline - time.monotonic())))
            if message is not None and self._accept_autopilot_heartbeat(message):
                return True
        if self.parameters.fail_on_timeout:
            raise TimeoutError(
                f"No autopilot heartbeat from {self.parameters.address} within {self.parameters.timeout}s"
            )
        return False
