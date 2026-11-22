import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from vtol.domain.models import Pose3D
from vtol.domain.mission.execution_status import MissionExecutionStatus


class DroneMode(str, Enum):
    UNKNOWN = "UNKNOWN"
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    TRAINING = "TRAINING"
    ACRO = "ACRO"
    FBWA = "FBWA"
    FBWB = "FBWB"
    CRUISE = "CRUISE"
    AUTOTUNE = "AUTOTUNE"
    ALT_HOLD = "ALT_HOLD"
    POSHOLD = "POSHOLD"
    LOITER = "LOITER"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    RTL = "RTL"
    TAKEOFF = "TAKEOFF"
    LAND = "LAND"
    BRAKE = "BRAKE"
    CIRCLE = "CIRCLE"
    AVOID_ADSB = "AVOID_ADSB"
    DRIFT = "DRIFT"
    SPORT = "SPORT"
    FLIP = "FLIP"
    PLND = "PLND"
    INITIALISING = "INITIALISING"
    THERMAL = "THERMAL"
    AUTOLAND = "AUTOLAND"
    LOITER_ALT_QLAND = "LOITER_ALT_QLAND"
    QSTABILIZE = "QSTABILIZE"
    QHOVER = "QHOVER"
    QLOITER = "QLOITER"
    QLAND = "QLAND"
    QRTL = "QRTL"
    QAUTOTUNE = "QAUTOTUNE"
    QACRO = "QACRO"
    QGUIDED = "QGUIDED"

    @classmethod
    def from_str(cls, s: str | None) -> "DroneMode":
        if not s:
            return cls.UNKNOWN
        s = s.strip().upper()
        aliases = {
            "ALTHOLD": "ALT_HOLD",
            "POS_HOLD": "POSHOLD",
            # pymavlink 2.4.49 reports this QuadPlane mode without separators.
            "LOITERALTQLAND": "LOITER_ALT_QLAND",
        }
        s = aliases.get(s, s)
        return cls(s) if s in cls._value2member_map_ else cls.UNKNOWN


class LandedState(str, Enum):
    UNKNOWN = "UNKNOWN"
    ON_GROUND = "ON_GROUND"
    IN_AIR = "IN_AIR"
    TAKEOFF = "TAKEOFF"
    LANDING = "LANDING"

    @classmethod
    def from_mavlink(cls, value: int | None) -> "LandedState":
        return {
            1: cls.ON_GROUND,
            2: cls.IN_AIR,
            3: cls.TAKEOFF,
            4: cls.LANDING,
        }.get(value, cls.UNKNOWN)


class VtolState(str, Enum):
    """VTOL configuration and transition state reported by MAVLink."""

    UNKNOWN = "UNKNOWN"
    UNDEFINED = "UNDEFINED"
    TRANSITION_TO_FIXED_WING = "TRANSITION_TO_FIXED_WING"
    TRANSITION_TO_MULTICOPTER = "TRANSITION_TO_MULTICOPTER"
    MULTICOPTER = "MULTICOPTER"
    FIXED_WING = "FIXED_WING"

    @classmethod
    def from_mavlink(cls, value: int | None) -> "VtolState":
        return {
            0: cls.UNDEFINED,
            1: cls.TRANSITION_TO_FIXED_WING,
            2: cls.TRANSITION_TO_MULTICOPTER,
            3: cls.MULTICOPTER,
            4: cls.FIXED_WING,
        }.get(value, cls.UNKNOWN)


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
    mode_name: str | None = None
    landed_state: LandedState = LandedState.UNKNOWN
    vtol_state: VtolState = VtolState.UNKNOWN

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

    def get_mission_execution_status(self) -> MissionExecutionStatus:
        """Return the latest mission observation without polling the transport."""
        return MissionExecutionStatus()

    @abstractmethod
    def send_landing_target(self, position: Pose3D, target_size: tuple[float, float]):
        """Send a LANDING_TARGET measurement; this message does not select a flight mode."""
        raise NotImplementedError()
