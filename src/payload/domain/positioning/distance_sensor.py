from collections.abc import AsyncIterator
from typing import Protocol

from payload.domain.positioning.events import DistanceEvent


class DistanceSensor(Protocol):
    """
    Produces distance samples and manages ranging failures.
    """

    def events(self) -> AsyncIterator[DistanceEvent]:
        """
        Yield ranging events as they arrive.
        """
        ...
