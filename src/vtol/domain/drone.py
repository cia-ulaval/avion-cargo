import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from vtol.domain.models import Pose3D
from vtol.domain.mission.execution_status import MissionExecutionStatus
from vtol.domain.rigid_transform import RigidTransform


class DroneMode(str, Enum):
    """ArduPlane/QuadPlane flight modes, with existing ArduCopter support."""

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
            "FLY_BY_WIRE_A": "FBWA",
            "FLY_BY_WIRE_B": "FBWB",
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
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    attitude_timestamp_s: float | None = None
    position_timestamp_s: float | None = None

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


@dataclass(frozen=True, slots=True)
class HoldPosition:
    latitude_deg: float
    longitude_deg: float
    relative_altitude_m: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.latitude_deg, self.longitude_deg, self.relative_altitude_m)):
            raise ValueError("Hold position must be finite")
        if not -90 <= self.latitude_deg <= 90 or not -180 <= self.longitude_deg <= 180:
            raise ValueError("Invalid hold latitude/longitude")

    def horizontal_error_m(self, status: DroneStatus) -> float:
        north = math.radians(status.latitude - self.latitude_deg)
        east = math.radians((status.longitude - self.longitude_deg + 180) % 360 - 180)
        east *= math.cos(math.radians(self.latitude_deg))
        return 6371000.0 * math.hypot(north, east)


class Drone(ABC):
    """Vehicle telemetry, target measurements and optional supervised yaw control."""

    def prepare_yaw_control(self) -> bool:
        """Poll nonblocking QuadPlane/RC preflight checks; true once ready."""
        raise NotImplementedError("This drone adapter does not support yaw control")

    def request_mode(self, mode: DroneMode, *, expected_modes: tuple[DroneMode, ...] | None = None) -> bool:
        """Request a mode; its application must be confirmed through telemetry."""
        raise NotImplementedError("This drone adapter does not support mode changes")

    def send_yaw_input(self, value: float) -> None:
        """Send a calibrated yaw stick input in [-1, 1], in GUIDED VTOL only."""
        raise NotImplementedError("This drone adapter does not support yaw control")

    def request_position_hold(self, position: HoldPosition) -> bool:
        """Set a fixed GUIDED position/altitude; never switch an external mode."""
        raise NotImplementedError("This drone adapter does not support position hold")

    def position_hold_accepted(self) -> bool | None:
        """Latest hold command ACK: accepted, rejected, or still pending."""
        raise NotImplementedError("This drone adapter does not support position hold")

    def release_yaw_control(self) -> None:
        """Release only the RC channel owned by this adapter."""

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
    def send_landing_target(
        self,
        position: Pose3D,
        target_size: tuple[float, float],
        *,
        body_from_marker: RigidTransform | None = None,
        captured_at_s: float | None = None,
    ) -> None:
        """Send a LANDING_TARGET measurement without changing flight mode."""
        raise NotImplementedError()
