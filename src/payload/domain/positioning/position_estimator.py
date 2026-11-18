from typing import Protocol

from payload.domain.positioning.events import DistanceEvent
from shared.contracts.geometry import PositionEstimate


class PositionEstimator(Protocol):
    """
    Consumes ranging events and emits an estimate when enough data is available.
    """

    def update(self, event: DistanceEvent) -> PositionEstimate | None:
        ...
