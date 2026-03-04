import math
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from domain.drone import Drone, DroneMode, DroneStatus
from domain.models import Pose3D


@dataclass(frozen=True, slots=True)
class MavlinkConnectionParams:
    address: str
    port: int = 0
    timeout: int = 5
    baud_rate: int = 921600


class DroneMavlinkBase(Drone):
    def __init__(self, params: MavlinkConnectionParams):
        self._p = params
        self._conn: Optional[mavutil.mavfile] = None

        # Cache télémétrie (valeurs brutes)
        self._mode_str: str = "UNKNOWN"
        self._status: DroneStatus = DroneStatus(
            DroneMode.UNKNOWN,
            groundspeed_mps=0.0,
            battery_voltage_v=0.0,
            battery_remaining_pct=0,
            gps_fix_type=0,
            armed=False,
            last_heartbeat_s=0.0,
            last_signal_gpio_s=0.0,
            alt_m = 0.0
        )

    # --------- helpers GPIO (optionnel) ---------

    def notify_gpio_signal(self, now_s: Optional[float] = None) -> None:
        """Appelle ça depuis ton handler GPIO quand le pin s'active."""
        self._status.last_signal_gpio_s = time.time() if now_s is None else float(now_s)

    # --------- Drone interface ---------

    def connect(self) -> None:
        raise NotImplementedError

    def get_status(self) -> DroneStatus:
        self._update_status()
        return self._status

    def move_to(self, position: Pose3D) -> None:
        """
        Envoie LANDING_TARGET (comme ton VehicleInterface) :
          - x: droite/gauche (m)
          - y: avant/arrière (m)
          - z: distance verticale / profondeur (m)  (doit être > 0)

        -> angles:
            angle_x = atan2(y, z)
            angle_y = atan2(x, z)
            distance = sqrt(x^2+y^2+z^2)
        """
        self._require_connected()
        if position.z <= 0.0:
            return

        angle_x = math.atan2(position.y, position.z)
        angle_y = math.atan2(position.x, position.z)
        distance = math.sqrt((position.x * position.x) +
                             (position.y * position.y) +
                             (position.z * position.z))

        self._conn.mav.landing_target_send(
            0,  # time_usec
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            angle_x,
            angle_y,
            distance,
            1,  # size_x
            1,  # size_y
        )

    def land(self) -> None:
        """
        Le 'position' n'est pas utilisé ici (pas de lat/lon). On fait:
          - set_mode('LAND') si disponible.
        """
        self._require_connected()
        self._conn.set_mode("LAND")
        self._update_status()

    # --------- internal ---------

    def _require_connected(self) -> None:
        if self._conn is None:
            raise RuntimeError("Drone not connected. Call connect() first.")

    def _update_status(self) -> None:

        self._require_connected()
        msg = self._conn.recv_match(blocking=False)
        while msg:
            message_type:str = msg.get_type()

            if message_type ==  "HEARTBEAT":
                self._status.mode = DroneMode.from_str(mavutil.mode_string_v10(msg))
                self._status.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self._status.last_heartbeat_s = time.time()

            elif message_type == "VFR_HUD":
                self._status.alt_m = float(getattr(msg, "alt", 0.0))
                self._status.groundspeed_mps = float(getattr(msg, "groundspeed", 0.0))

            elif message_type == "SYS_STATUS":
                battery_mv = getattr(msg, "voltage_battery", 0)  # mV
                self._status.battery_voltage_v = float(battery_mv) / 1000.0
                self._status.battery_remaining_pct = int(getattr(msg, "battery_remaining", 0) or 0)

            elif message_type == "GPS_RAW_INT":
                self._status.gps_fix_type = int(getattr(msg, "fix_type", 0) or 0)

            msg = self._conn.recv_match(blocking=False)


class DroneMavlinkSerial(DroneMavlinkBase):
    def connect(self) -> None:
        self._conn = mavutil.mavlink_connection(self._p.address, baud=self._p.baud_rate)
        self._conn.wait_heartbeat(timeout=self._p.timeout)
        self._status.last_heartbeat_s = time.time()


class DroneMavlinkUDP(DroneMavlinkBase):
    def connect(self) -> None:
        conn_str = f"udp:{self._p.address}:{self._p.port}"
        self._conn = mavutil.mavlink_connection(conn_str)
        self._conn.wait_heartbeat(timeout=self._p.timeout)
        self._status.last_heartbeat_s = time.time()