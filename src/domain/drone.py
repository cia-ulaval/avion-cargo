from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from domain.models import Pose3D


class DroneMode(str, Enum):
    UNKNOWN = "UNKNOWN"
    STABILIZE = "STABILIZE"
    ACRO = "ACRO"
    ALT_HOLD = "ALT_HOLD"
    POSHOLD = "POSHOLD"
    LOITER = "LOITER"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    RTL = "RTL"
    LAND = "LAND"
    BRAKE = "BRAKE"
    CIRCLE = "CIRCLE"
    DRIFT = "DRIFT"
    SPORT = "SPORT"
    FLIP = "FLIP"
    PLND = "PLND"

    @classmethod
    def from_str(cls, s: str | None) -> "DroneMode":
        if not s:
            return cls.UNKNOWN
        s = s.strip().upper()
        aliases = {
            "ALTHOLD": "ALT_HOLD",
            "POS_HOLD": "POSHOLD",
        }
        s = aliases.get(s, s)
        return cls(s) if s in cls._value2member_map_ else cls.UNKNOWN


@dataclass(slots=True)
class DroneStatus:
    mode: DroneMode
    alt_m: float
    groundspeed_mps: float
    battery_voltage_v: float
    battery_remaining_pct: int
    gps_fix_type: int
    armed: bool
    last_heartbeat_s: float
    last_signal_gpio_s: float

    @property
    def connected(self) -> bool:
        return self.last_heartbeat_s > 0

    def heartbeat_age_s(self, now_s: float) -> float:
        return now_s - self.last_heartbeat_s if self.last_heartbeat_s else float("inf")

    def should_drop(self, now_s: float, window_s: float = 1.0) -> bool:
        return (now_s - self.last_signal_gpio_s) < window_s


class Drone(ABC):
    @abstractmethod
    def connect(self):
        raise NotImplementedError()

    @abstractmethod
    def get_status(self) -> DroneStatus:
        raise NotImplementedError()

    @abstractmethod
    def move_to(self, position: Pose3D):
        raise NotImplementedError()

    @abstractmethod
    def land(self):
        raise NotImplementedError()

    def activate_precision_landing_mode(self):
        raise NotImplementedError()

    def activate_guided_mode(self):
        raise NotImplementedError()

    def arm(self):
        raise NotImplementedError()

    def takeoff(self):
        raise NotImplementedError()
