from collections.abc import AsyncIterator
from typing import Protocol

from payload.domain.positioning.events import DistanceEvent


class DistanceSensor(Protocol):
    """
    Produces distance samples and manages ranging failures.
    """

    async def start_ranging(self) -> None:
        """
        Start the configured ranging schedule.
        """
        ...

    async def stop_ranging(self) -> None:
        """
        Stop the active ranging schedule.
        """
        ...

    def events(self) -> AsyncIterator[DistanceEvent]:
        """
        Yield ranging events as they arrive.
        """
        ...
