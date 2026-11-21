from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    DockingAborted,
    DockingSucceeded,
    FailureReason,
    ParkingAborted,
    ParkingCompleted,
    PayloadSnapshot,
    PayloadStateUpdated,
    PlaneHeadingUpdated,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus


def test_payload_status_and_latch_are_independent_contracts() -> None:
    state = PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN)

    assert state.status is PayloadStatus.PARKED
    assert state.latch is DockingLatchState.OPEN
    assert [status.name for status in PayloadStatus] == [
        "UNKNOWN",
        "DOCKED",
        "PARKING",
        "PARKED",
        "DOCKING",
        "FAULT",
    ]


def test_dlz_snapshot_contains_application_identity_not_transport_identity() -> None:
    estimate = PositionEstimate(
        position=Position(0.2, -0.1, 0.0),
        residual_m=0.015,
        anchor_ids=("plane-anchor-a", "plane-anchor-b", "plane-anchor-c"),
        sequence=8,
    )
    payload = PayloadSnapshot(
        payload_id="payload-17",
        state=PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN),
        position=estimate,
        sequence=12,
    )
    state = DLZState(revision=4, payloads=(payload,))

    assert state.payloads[0].payload_id == "payload-17"
    assert state.payloads[0].position is estimate


def test_message_contracts_separate_command_acknowledgement_from_completion() -> None:
    heading_update = PlaneHeadingUpdated(Heading(angle_rad=1.2), sequence=5)
    request = UndockRequest("cmd-1", Position(0.4, 0.2, 0.0))
    accepted = CommandReceipt(command_id="cmd-1", accepted=True)
    parked = ParkingCompleted("cmd-1", Position(0.4, 0.2, 0.0))
    parking_abort = ParkingAborted("cmd-2", reason=FailureReason("blocked", "Path blocked"))
    docking_success = DockingSucceeded("cmd-3")
    docking_abort = DockingAborted("cmd-4", reason=FailureReason("alignment", "Alignment lost"))
    state_update = PayloadStateUpdated(PayloadState(PayloadStatus.DOCKING, DockingLatchState.OPEN), sequence=9)

    assert heading_update.sequence == 5
    assert request.command_id == accepted.command_id == parked.command_id
    assert parking_abort.reason.code == "blocked"
    assert docking_success.command_id == "cmd-3"
    assert docking_abort.reason.code == "alignment"
    assert state_update.state.status is PayloadStatus.DOCKING
