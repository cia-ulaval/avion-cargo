import math
import time
from abc import abstractmethod
from dataclasses import dataclass
from typing import Optional
from pymavlink import mavutil
from domain.drone import Drone, DroneMode, DroneStatus
from domain.models import Pose3D


@dataclass(frozen=True, slots=True)
class MavlinkConnectionParams:
    address: str
    port: int = 0
    timeout: float= 10.0
    baud_rate: int = 921600


class DroneMavlinkBase(Drone):
    def __init__(self, params: MavlinkConnectionParams):
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
            heading_deg=0.0
        )

    def notify_gpio_signal(self, now_s: Optional[float] = None) -> None:
        """Appelle ça depuis ton handler GPIO quand le pin s'active."""
        self.status.last_signal_gpio_s = time.time() if now_s is None else float(now_s)


    def connect(self) -> None:
        self.init_mavlink_connection(self.parameters)
        self._wait_for_heartbeat()
        self._send_heartbeat()
        self._update_status()

    @abstractmethod
    def init_mavlink_connection(self, params: MavlinkConnectionParams) -> None:
        raise NotImplementedError()

    def get_status(self) -> DroneStatus:
        self._update_status()
        return self.status

    def land_on_target(self, uav_pose: Pose3D) -> None:
        const = math.pi / 180
        h_fov, v_fov = 53.5 * const, 41.41 * const
        x_ang = (uav_pose.x - 640 * .5) * h_fov / 640
        y_ang = (uav_pose.y - 480 * .5) * v_fov / 480

        distance = math.sqrt(uav_pose.x ** 2 + uav_pose.y ** 2 + uav_pose.z ** 2)
        angle_x = math.atan2(uav_pose.y, uav_pose.z)
        angle_y = math.atan2(uav_pose.x, uav_pose.z)
        self.connection.mav.landing_target_send(
        int(time.time() * 1_000_000),                 # time_usec
        0,                                            # target_num
        mavutil.mavlink.MAV_FRAME_BODY_FRD,          # frame
        x_ang,                                          # angle_x
        y_ang,                                          # angle_y
        distance,                                     # distance
        1.0185,                                          # size_x
        1.0185,                                          # size_y
        uav_pose.x,                                        # x
        uav_pose.y,                                      # y
        uav_pose.z,                                       # z
        [1.0, 0.0, 0.0, 0.0],                         # q  <-- tableau de 4 floats
        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        1                                          # position_valid
    )




    def activate_land_mode(self) -> None:
        pass


    def _require_connected(self) -> None:
        if self.connection is None:
            raise RuntimeError("Drone not connected. Call connect() first.")

    def _update_status(self) -> None:
        self._require_connected()

        msg = self.connection.recv_match(blocking=True)
        if not msg: return

        message_type: str = msg.get_type()

        if message_type == "HEARTBEAT":
            self.status.mode = DroneMode.from_str(mavutil.mode_string_v10(msg))
            self.status.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.status.last_heartbeat_s = time.time()

        elif message_type == "VFR_HUD":
            self.status.alt_m = float(getattr(msg, "alt", 0.0))
            self.status.groundspeed_mps = float(getattr(msg, "groundspeed", 0.0))

        elif message_type == "SYS_STATUS":
            battery_mv = getattr(msg, "voltage_battery", 0)  # mV
            self.status.battery_voltage_v = float(battery_mv) / 1000.0
            self.status.battery_remaining_pct = int(getattr(msg, "battery_remaining", 0) or 0)

        elif message_type == "GPS_RAW_INT":
            self.status.gps_fix_type = int(getattr(msg, "fix_type", 0) or 0)


        elif message_type == "GLOBAL_POSITION_INT":
            self.status.latitude = float(getattr(msg, "lat", 0)) / 1e7
            self.status.longitude = float(getattr(msg, "lon", 0)) / 1e7
            self.status.relative_altitude_ms = float(getattr(msg, "alt", 0)) / 1000.0
            self.status.relative_altitude= float(getattr(msg, "relative_alt", 0)) / 1000.0
            self.status.speed = float(getattr(msg, "vz", 0)) / 100.0
            hdg = int(getattr(msg, "hdg", 65535))
            self.status.heading_deg = None if hdg == 65535 else hdg / 100.0

    def _send_heartbeat(self):
        self._require_connected()
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,0,0,
            mavlink_version= 2
        )

    def _wait_for_heartbeat(self):
        self._require_connected()
        self.connection.wait_heartbeat()


class DroneMavlinkSerial(DroneMavlinkBase):
    def __init__(self, params: MavlinkConnectionParams):
        super().__init__(params)

    def init_mavlink_connection(self, params) -> None:
        self.connection = mavutil.mavlink_connection(self.parameters.address, baud=self.parameters.baud_rate)


class DroneMavlinkUDP(DroneMavlinkBase):
    def __init__(self, params: MavlinkConnectionParams):
        super().__init__(params)

    def init_mavlink_connection(self, params) -> None:
        self.connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
