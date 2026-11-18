from collections.abc import AsyncIterator
from typing import Protocol

from shared.contracts.geometry import PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    PayloadOperationEvent,
    PayloadStatusReport,
    PlaneEvent,
)
from shared.contracts.payload_state import PayloadState


class PlanePeer(Protocol):
    """
    The plane as observed by one payload application.
    """

    def events(self) -> AsyncIterator[PlaneEvent]:
        """
        Yield plane telemetry, DLZ snapshots, commands, and status queries.
        """
        ...

    async def report_position(self, estimate: PositionEstimate) -> None:
        """
        Report the payload's estimated position in the plane-local frame.
        """
        ...

    async def report_state(self, state: PayloadState, sequence: int) -> None:
        """
        Report operational and observed latch states.
        """
        ...

    async def acknowledge_command(self, receipt: CommandReceipt) -> None:
        """
        Accept or reject an incoming command promptly, before physical completion.
        """
        ...

    async def respond_status_request(
        self, request_id: str, report: PayloadStatusReport
    ) -> None:
        """Return the application's current status snapshot for a plane query."""
        ...

    async def report_operation_event(self, event: PayloadOperationEvent) -> None:
        """
        Report the eventual result of a parking or docking operation.
        """
        ...
