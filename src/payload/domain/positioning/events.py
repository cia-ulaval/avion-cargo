from dataclasses import dataclass
from typing import TypeAlias

from shared.contracts.messages import FailureReason


@dataclass(frozen=True, slots=True)
class DistanceSample:
    """One measured 3-D range to a configured UWB anchor."""

    anchor_id: str
    distance_m: float
    sequence: int
    device_monotonic_us: int | None = None
    epoch_us: int | None = None
    received_at: float | None = None
    status: int | None = None
    quality_flags: int | None = None


@dataclass(frozen=True, slots=True)
class DistanceFailure:
    """A failed ranging attempt to a plane anchor."""

    anchor_id: str
    reason: FailureReason
    device_monotonic_us: int | None = None
    epoch_us: int | None = None
    received_at: float | None = None
    status: int | None = None
    quality_flags: int | None = None


DistanceEvent: TypeAlias = DistanceSample | DistanceFailure
