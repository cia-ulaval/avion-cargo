from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Heading:
    angle_rad: float


@dataclass(frozen=True, slots=True)
class PositionEstimate:
    position: Position
    residual_m: float | None
    anchor_ids: tuple[str, ...]
    sequence: int
    device_monotonic_us: int | None = None
    epoch_us: int | None = None
    received_at: float | None = None
    covariance_xy_m2: tuple[float, float, float] | None = None
    quality: str = "unknown"
