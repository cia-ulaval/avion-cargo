from dataclasses import dataclass
from typing import TypeAlias

from shared.contracts.messages import FailureReason


@dataclass(frozen=True, slots=True)
class DistanceSample:
    """
    One measured distance to a plane anchor, in meters.
    """
    anchor_id: str
    distance_m: float
    sequence: int


@dataclass(frozen=True, slots=True)
class DistanceFailure:
    """
    A failed ranging attempt to a plane anchor.
    """
    anchor_id: str
    reason: FailureReason


DistanceEvent: TypeAlias = DistanceSample | DistanceFailure
