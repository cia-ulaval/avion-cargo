import math
import time
from abc import abstractmethod
from dataclasses import replace
from threading import RLock
from typing import Optional

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from loguru import logger

from domain.drone import Drone, DroneMode, DroneStatus
from domain.models import Pose3D

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
        self._link_was_alive = False
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
                self._wait_for_heartbeat()
                self._send_heartbeat()
                self._request_status_streams()
                self._link_was_alive = True
                logger.info(
                    "MAVLink autopilot connected: system {}, component {}", self._target_system, self._target_component
                )
            except BaseException:
                try:
                    self.close()
                except Exception:
                    logger.exception("Failed to close MAVLink transport after connection failure")
                raise

    def close(self) -> None:
        with self._lock:
            connection, self.connection = self.connection, None
            self.status.last_heartbeat_s = 0.0
            self.status.last_heartbeat_monotonic_s = None
            self._target_system = self._target_component = None
            self._last_sent_heartbeat_s = None
            self._link_was_alive = False
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
                else:
                    logger.warning("MAVLink heartbeat expired; vehicle connection lost")
                self._link_was_alive = alive
            return replace(self.status)

    def land_on_target(self, uav_pose: Pose3D, target_size: tuple[float, float]) -> None:
        with self._lock:
            self._send_landing_target(uav_pose, target_size)

    def _send_landing_target(self, uav_pose: Pose3D, target_size: tuple[float, float]) -> None:
        """Send the MAVLink 2 position branch, expressed in metres in BODY_FRD.

        Image angles/sizes cannot be recovered from a body-frame translation
        for an arbitrary camera mounting. Leave these unused legacy fields at
        zero; ArduPilot consumes x/y/z and distance when position_valid is one.
        ``target_size`` remains in the domain API but is not an angular size.
        """
        self._require_connected()
        distance = math.hypot(uav_pose.x, uav_pose.y, uav_pose.z)
        if not math.isfinite(distance) or uav_pose.z <= 0:
            raise ValueError("Landing target must be finite and below the vehicle (BODY_FRD z > 0)")

        self.connection.mav.landing_target_send(
            int(time.time() * 1_000_000),  # time_usec
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
            [1.0, 0.0, 0.0, 0.0],  # q  <-- tableau de 4 floats
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            1,  # position_valid
        )

    def activate_land_mode(self) -> None:
        pass

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
            if self._is_from_autopilot(received_message):
                self._handle_message(received_message)
            if time.monotonic() >= deadline:
                break

    def _is_from_autopilot(self, message) -> bool:
        return (message.get_srcSystem(), message.get_srcComponent()) == (self._target_system, self._target_component)

    def _handle_message(self, message) -> None:
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self.status.mode = DroneMode.from_str(mavutil.mode_string_v10(message))
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
            self.status.latitude = float(message.lat) / 1e7
            self.status.longitude = float(message.lon) / 1e7
            self.status.relative_altitude_ms = float(message.alt) / 1000.0
            self.status.relative_altitude = float(message.relative_alt) / 1000.0
            self.status.speed = float(message.vz) / 100.0
            self.status.heading_deg = None if message.hdg == 65535 else message.hdg / 100.0
        elif message_type == "COMMAND_ACK" and message.command == mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL:
            if message.result not in (mavlink2.MAV_RESULT_ACCEPTED, mavlink2.MAV_RESULT_IN_PROGRESS):
                logger.warning("Autopilot rejected a telemetry rate request (MAV_RESULT={})", message.result)

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
        logger.info("Requested MAVLink status streams (position 10 Hz, HUD 5 Hz, GPS 2 Hz, battery 1 Hz)")

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

    def _wait_for_heartbeat(self):
        self._require_connected()
        deadline = time.monotonic() + self.parameters.timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(blocking=True, timeout=min(0.2, max(0, deadline - time.monotonic())))
            if message is None or message.get_type() != "HEARTBEAT":
                continue
            if (
                message.autopilot == mavlink2.MAV_AUTOPILOT_INVALID
                or message.type in (mavlink2.MAV_TYPE_GCS, mavlink2.MAV_TYPE_ONBOARD_CONTROLLER)
                or message.get_srcSystem() == 0
            ):
                continue
            self._target_system = message.get_srcSystem()
            self._target_component = message.get_srcComponent()
            self.connection.target_system = self._target_system
            self.connection.target_component = self._target_component
            self._handle_message(message)
            return
        raise TimeoutError(f"No autopilot heartbeat from {self.parameters.address} within {self.parameters.timeout}s")

    def switch_mode(self, mode: DroneMode) -> None:
        pass
