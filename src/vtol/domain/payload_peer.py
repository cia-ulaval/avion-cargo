"""VTOL-side interface to one remote payload."""

from collections.abc import AsyncIterator
from typing import Protocol

from shared.contracts.geometry import Heading, Position
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    PayloadEvent,
    PayloadStatusReport,
)


class PayloadPeer(Protocol):
    """One payload as observed by the plane application."""

    @property
    def payload_id(self) -> str:
        """Stable application identity, distinct from BLE peer and UWB target IDs."""
        ...

    def events(self) -> AsyncIterator[PayloadEvent]:
        """Yield position, state, and operation-result reports from this payload."""
        ...

    async def request_undock(self, target_position: Position) -> CommandReceipt:
        """Ask the payload to undock and park at a plane-local target position."""
        ...

    async def request_dock(self) -> CommandReceipt:
        """Ask the payload to dock to the plane."""
        ...

    async def request_status(self) -> PayloadStatusReport:
        """Request the payload's latest state and position report."""
        ...

    async def publish_heading(self, heading: Heading, sequence: int) -> None:
        """Publish a plane heading update to this payload."""
        ...

    async def publish_dlz_state(self, state: DLZState) -> None:
        """Publish the latest plane-maintained DLZ snapshot to this payload."""
        ...
