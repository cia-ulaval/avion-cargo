import asyncio

from payload.application.runtime import PayloadApplicationRuntime
from shared.contracts.geometry import Position
from shared.contracts.messages import (
    CommandReceipt,
    DockRequest,
    FailureReason,
    PayloadStatusRequested,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus


class FakePlanePeer:
    def __init__(self):
        self.receipts = []
        self.status_reports = []

    async def acknowledge_command(self, receipt):
        self.receipts.append(receipt)

    async def respond_status_request(self, request_id, report):
        self.status_reports.append((request_id, report))


class FakeDistanceSensor:
    pass


def test_unimplemented_dock_and_undock_are_explicitly_rejected() -> None:
    async def scenario():
        plane = FakePlanePeer()
        runtime = PayloadApplicationRuntime(plane, FakeDistanceSensor())

        await runtime._reject_unsupported_command(DockRequest("dock-1"))
        await runtime._reject_unsupported_command(UndockRequest("park-1", Position(1.0, 2.0, 0.0)))

        assert [receipt.command_id for receipt in plane.receipts] == ["dock-1", "park-1"]
        assert all(not receipt.accepted for receipt in plane.receipts)
        assert all(receipt.rejection_reason.code == "not_implemented" for receipt in plane.receipts)

    asyncio.run(scenario())


def test_status_query_reports_unknown_instead_of_managing_payload_state() -> None:
    async def scenario():
        plane = FakePlanePeer()
        runtime = PayloadApplicationRuntime(plane, FakeDistanceSensor())
        await runtime._report_unknown_status(PayloadStatusRequested("status-1"))

        request_id, report = plane.status_reports[0]
        assert request_id == "status-1"
        assert report.state == PayloadState(PayloadStatus.UNKNOWN, DockingLatchState.UNKNOWN)
        assert report.position is None
        assert report.sequence == 0

    asyncio.run(scenario())
