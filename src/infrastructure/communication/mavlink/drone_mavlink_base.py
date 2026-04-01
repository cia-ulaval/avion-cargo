import math
import time
from abc import abstractmethod
from typing import Optional

from pymavlink import mavutil

from domain.drone import Drone, DroneMode, DroneStatus
from domain.models import Pose3D

from .mavlink_connection_params import MavlinkConnectionParams


class DroneMavlinkBase(Drone):
    def __init__(self, params: MavlinkConnectionParams):
        """

        :param params:
        """
        self.parameters = params
        self.connection: Optional[mavutil.mavfile] = None
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
        self._init_mavlink_connection()
        self._wait_for_heartbeat()
        self._send_heartbeat()
        self._update_status()

    def get_status(self) -> DroneStatus:
        self._update_status()
        return self.status

    def land_on_target(self, uav_pose: Pose3D, target_size: tuple[float, float]) -> None:
        const = math.pi / 180
        h_fov, v_fov = 53.5 * const, 41.41 * const
        x_ang = (uav_pose.x - 640 * 0.5) * h_fov / 640
        y_ang = (uav_pose.y - 480 * 0.5) * v_fov / 480

        distance = math.sqrt(uav_pose.x**2 + uav_pose.y**2 + uav_pose.z**2)

        self.connection.mav.landing_target_send(
            int(time.time() * 1_000_000),  # time_usec
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
            x_ang,  # angle_x
            y_ang,  # angle_y
            distance,  # distance
            target_size[0],  # size_x
            target_size[1],  # size_y
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
        received_message = self.connection.recv_match(blocking=True)

        if received_message:

            message_type: str = received_message.get_type()

            if message_type == "HEARTBEAT":
                self.status.mode = DroneMode.from_str(mavutil.mode_string_v10(received_message))
                self.status.armed = (received_message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.status.last_heartbeat_s = time.time()

            elif message_type == "VFR_HUD":
                self.status.alt_m = float(getattr(received_message, "alt", 0.0))
                self.status.groundspeed_mps = float(getattr(received_message, "groundspeed", 0.0))

            elif message_type == "SYS_STATUS":
                battery_mv = getattr(received_message, "voltage_battery", 0)  # mV
                self.status.battery_voltage_v = float(battery_mv) / 1000.0
                self.status.battery_remaining_pct = int(getattr(received_message, "battery_remaining", 0) or 0)

            elif message_type == "GPS_RAW_INT":
                self.status.gps_fix_type = int(getattr(received_message, "fix_type", 0) or 0)

            elif message_type == "GLOBAL_POSITION_INT":
                self.status.latitude = float(getattr(received_message, "lat", 0)) / 1e7
                self.status.longitude = float(getattr(received_message, "lon", 0)) / 1e7
                self.status.relative_altitude_ms = float(getattr(received_message, "alt", 0)) / 1000.0
                self.status.relative_altitude = float(getattr(received_message, "relative_alt", 0)) / 1000.0
                self.status.speed = float(getattr(received_message, "vz", 0)) / 100.0
                hdg = int(getattr(received_message, "hdg", 65535))
                self.status.heading_deg = None if hdg == 65535 else hdg / 100.0

    def _send_heartbeat(self):
        self._require_connected()
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
            mavlink_version=2,
        )

    def _wait_for_heartbeat(self):
        self._require_connected()
        self.connection.wait_heartbeat()

    def switch_mode(self, mode: DroneMode) -> None:
        pass
