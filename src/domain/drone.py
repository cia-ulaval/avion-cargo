import time
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
    speed: float
    relative_altitude: float
    latitude: float
    longitude: float
    relative_altitude_ms: float
    heading_deg: float | None
    last_heartbeat_monotonic_s: float | None = None
    heartbeat_timeout_s: float = 3.0

    @property
    def connected(self) -> bool:
        if self.last_heartbeat_monotonic_s is not None:
            age = time.monotonic() - self.last_heartbeat_monotonic_s
        else:
            age = self.heartbeat_age_s(time.time())
        return self.last_heartbeat_s > 0 and 0 <= age < self.heartbeat_timeout_s

    def heartbeat_age_s(self, now_s: float) -> float:
        return now_s - self.last_heartbeat_s if self.last_heartbeat_s else float("inf")

    def should_drop(self, now_s: float, window_s: float = 1.0) -> bool:
        return self.last_signal_gpio_s > 0 and 0 <= (now_s - self.last_signal_gpio_s) < window_s


class Drone(ABC):
    """Receive vehicle telemetry and send target measurements, without flight control."""

    def close(self) -> None:
        """Release the connection; adapters owning a transport override this."""

    @abstractmethod
    def connect(self):
        raise NotImplementedError()

    @abstractmethod
    def get_status(self) -> DroneStatus:
        raise NotImplementedError()

    @abstractmethod
    def send_landing_target(self, position: Pose3D, target_size: tuple[float, float]):
        """Send a LANDING_TARGET measurement; this message does not select a flight mode."""
        raise NotImplementedError()
