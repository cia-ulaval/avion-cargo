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
