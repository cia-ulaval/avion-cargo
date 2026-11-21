import pytest

from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    DLZStateUpdated,
    DockRequest,
    DockingAborted,
    DockingSucceeded,
    FailureReason,
    ParkingAborted,
    ParkingCompleted,
    PayloadPositionUpdated,
    PayloadSnapshot,
    PayloadStateUpdated,
    PayloadStatusReport,
    PlaneHeadingUpdated,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus
from shared.infrastructure.d3kble.codec import (
    D3kblePayloadTooLarge,
    decode_command_receipt,
    decode_command_request,
    decode_payload_event,
    decode_plane_event,
    decode_status_report,
    dlz_state_from_data,
    dlz_state_to_data,
    encode_command_receipt,
    encode_dock_request,
    encode_payload_event,
    encode_plane_event,
    encode_status_report,
    encode_undock_request,
)


@pytest.fixture
def estimate() -> PositionEstimate:
    return PositionEstimate(
        position=Position(0.25, -0.12, 0.0),
        residual_m=0.012,
        anchor_ids=("anchor-a", "anchor-b", "anchor-c"),
        sequence=17,
    )


def test_plane_telemetry_round_trips() -> None:
    events = (
        PlaneHeadingUpdated(Heading(1.25), sequence=8),
        DLZStateUpdated(DLZState(revision=2, payloads=())),
    )
    for event in events:
        assert decode_plane_event(encode_plane_event(event)) == event


def test_payload_reports_and_operation_events_round_trip(estimate: PositionEstimate) -> None:
    state = PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN)
    events = (
        PayloadPositionUpdated(estimate),
        PayloadStateUpdated(state, sequence=18),
        ParkingCompleted("cmd-1", Position(0.1, 0.2, 0.0)),
        ParkingAborted("cmd-2", FailureReason("blocked", "Path is obstructed")),
        DockingSucceeded("cmd-3"),
        DockingAborted("cmd-4", FailureReason("alignment", "Alignment was lost")),
    )
    for event in events:
        assert decode_payload_event(encode_payload_event(event)) == event


def test_command_request_and_receipt_round_trip() -> None:
    dock = encode_dock_request("cmd-1")
    undock = encode_undock_request("cmd-2", Position(0.5, -0.2, 0.0))
    accepted = CommandReceipt("cmd-1", accepted=True)
    rejected = CommandReceipt("cmd-2", accepted=False, rejection_reason=FailureReason("busy", "Not ready"))

    assert decode_command_request(dock) == DockRequest("cmd-1")
    assert decode_command_request(undock) == UndockRequest("cmd-2", Position(0.5, -0.2, 0.0))
    assert decode_command_receipt(encode_command_receipt(accepted)) == accepted
    assert decode_command_receipt(encode_command_receipt(rejected)) == rejected


def test_status_report_and_dlz_snapshot_round_trip(estimate: PositionEstimate) -> None:
    state = PayloadState(PayloadStatus.DOCKED, DockingLatchState.CLOSED)
    report = PayloadStatusReport(state=state, position=estimate, sequence=20)
    dlz = DLZState(
        revision=9,
        payloads=(PayloadSnapshot("payload-1", state, estimate, sequence=20),),
    )

    assert decode_status_report(encode_status_report(report)) == report
    assert dlz_state_from_data(dlz_state_to_data(dlz)) == dlz


def test_wire_codec_rejects_unknown_versions_and_non_finite_coordinates() -> None:
    from shared.infrastructure.d3kble.codec import decode_message, position_from_data

    with pytest.raises(ValueError, match="unsupported protocol version"):
        decode_message({"protocol": "vehicle-dlz", "version": 2, "type": "x", "data": {}})
    with pytest.raises(ValueError, match="finite number"):
        position_from_data({"x": float("nan"), "y": 0.0, "z": 0.0})


def test_codec_rejects_dlz_snapshot_over_d3kble_limit() -> None:
    large = DLZState(
        revision=1,
        payloads=tuple(
            PayloadSnapshot(
                payload_id=f"payload-{index}",
                state=PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN),
                position=PositionEstimate(
                    Position(0.0, 0.0, 0.0),
                    residual_m=0.01,
                    anchor_ids=("anchor-a", "anchor-b", "anchor-c", "anchor-d"),
                    sequence=index,
                ),
                sequence=index,
            )
            for index in range(20)
        ),
    )

    with pytest.raises(D3kblePayloadTooLarge):
        encode_plane_event(DLZStateUpdated(large))


def test_typical_dlz_snapshot_with_four_payloads_fits_ble_envelope(estimate: PositionEstimate) -> None:
    state = PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN)
    dlz = DLZState(
        revision=12,
        payloads=tuple(
            PayloadSnapshot(
                payload_id=f"payload-{index}",
                state=state,
                position=PositionEstimate(
                    Position(0.1 * index, -0.05 * index, 0.0),
                    residual_m=0.015,
                    anchor_ids=estimate.anchor_ids,
                    sequence=index + 1,
                ),
                sequence=index + 1,
            )
            for index in range(4)
        ),
    )

    message = encode_plane_event(DLZStateUpdated(dlz))

    assert decode_plane_event(message) == DLZStateUpdated(dlz)
