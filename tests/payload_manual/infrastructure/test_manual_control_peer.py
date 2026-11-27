import asyncio
import json
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from payload_manual.domain.manual_control import ManualControlCommand, ReferenceVelocity
from payload_manual.infrastructure.d3kble.manual_control_peer import (
    D3kbleManualControlPeer,
    ManualControlProtocolError,
    decode_manual_control_message,
    encode_manual_control_message,
)
from shared.infrastructure.d3kble.router import D3kbleMessageRouter
from shared.contracts.geometry import Position, PositionEstimate
from payload.domain.positioning.heading_estimator import HeadingEstimate


class FakeTicket:
    def __init__(self):
        self.ack_future = Future()


class FakeComms:
    def __init__(self):
        self.sent = None
        self.sent_messages = []
        self.ticket = None
        self.tickets = []

    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def send(self, peer_id, payload):
        self.sent = (peer_id, payload)
        self.sent_messages.append((peer_id, payload))
        self.ticket = FakeTicket()
        self.tickets.append(self.ticket)
        return self.ticket

    def on_message(self, callback=None):
        self.message_handler = callback

    def on_request(self, callback=None):
        self.request_handler = callback


class FakeNode:
    def __init__(self):
        self.comms = FakeComms()


def test_manual_control_protocol_round_trips_drive_and_state_commands():
    command = ManualControlCommand("test-session", 7, "drive", ReferenceVelocity(0.12, -0.04, 0.5))
    assert decode_manual_control_message(encode_manual_control_message(command)) == command

    for name in ("arm", "stop", "disarm", "status"):
        command = ManualControlCommand("test-session", 8, name)
        assert decode_manual_control_message(encode_manual_control_message(command)) == command


@pytest.mark.parametrize(
    "payload",
    [
        {"protocol": "wrong", "version": 1, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": True, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": 3, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": 1, "type": "arm", "session_id": "s", "sequence": True},
        {"protocol": "payload-manual-control", "version": 1, "type": "arm", "sequence": 0},
        {
            "protocol": "payload-manual-control",
            "version": 1,
            "type": "drive",
            "session_id": "s",
            "sequence": 0,
            "vx_m_s": float("nan"),
            "vy_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
        },
        {
            "protocol": "payload-manual-control",
            "version": 1,
            "type": "drive",
            "session_id": "s",
            "sequence": 0,
            "vx_m_s": True,
            "vy_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
        },
        {
            "protocol": "payload-manual-control",
            "version": 1,
            "type": "arm",
            "session_id": "s",
            "sequence": 0,
            "vx_m_s": 0.0,
        },
    ],
)
def test_manual_control_protocol_rejects_invalid_messages(payload):
    with pytest.raises(ManualControlProtocolError):
        decode_manual_control_message(payload)


@pytest.mark.asyncio
async def test_peer_routes_only_registered_peer_and_coalesces_drive_setpoints():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=50, queue_size=4)
    await peer.start()

    handler = node.comms.message_handler
    # The router dispatches by sender peer ID.
    handler(
        SimpleNamespace(
            peer_id=51,
            payload=encode_manual_control_message(ManualControlCommand("test-session", 1, "arm")),
        )
    )
    handler(
        SimpleNamespace(
            peer_id=50,
            payload=encode_manual_control_message(
                ManualControlCommand("test-session", 2, "drive", ReferenceVelocity(0.1, 0, 0))
            ),
        )
    )
    handler(
        SimpleNamespace(
            peer_id=50,
            payload=encode_manual_control_message(
                ManualControlCommand("test-session", 3, "drive", ReferenceVelocity(0.2, 0, 0))
            ),
        )
    )

    events = peer.events()
    received = await anext(events)
    assert received.command_type == "drive"
    assert received.sequence == 3
    assert received.velocity.vx_m_s == pytest.approx(0.2)

    await events.aclose()
    peer.close()
    router.close()


@pytest.mark.asyncio
async def test_peer_sends_payload_arm_state_acknowledgement():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=50)

    await peer.send_status("ui-session", 12, True, True)

    peer_id, payload = node.comms.sent
    assert peer_id == 50
    assert payload == {
        "protocol": "payload-manual-control-status",
        "version": 1,
        "type": "status",
        "session_id": "ui-session",
        "sequence": 12,
        "armed": True,
        "session_active": True,
    }
    # The acknowledgement future intentionally remains unresolved. State
    # reporting must not hold up the manual command consumer while it waits.
    assert not node.comms.ticket.ack_future.done()
    router.close()


@pytest.mark.asyncio
async def test_positioning_telemetry_fits_d3kble_496_byte_envelope():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=2)
    estimate = PositionEstimate(
        position=Position(0.0318, 0.1995, 0.081),
        residual_m=0.0052,
        anchor_ids=("anchor-3", "anchor-4", "anchor-5"),
        sequence=12345,
        device_monotonic_us=242_672_000,
        covariance_xy_m2=(0.000185, 0.000002, 0.000125),
        quality="good",
    )
    snapshot = SimpleNamespace(
        tag_estimate=estimate,
        payload_position=Position(0.0198, 0.2055, 0.081),
        heading=HeadingEstimate(0.3, 0.8, 123),
        status="tracking",
        anchors_xy_m=(
            ("anchor-3", -1.0, -0.6),
            ("anchor-4", 1.0, -0.6),
            ("anchor-5", 0.0, 1.2),
        ),
        payload_to_tag_xy_m=(0.012, -0.006),
        reference_origin_world_m=(0.0, -0.2, 0.06),
        reference_x_axis_world=(1.0, 0.0, 0.0),
        reference_y_axis_world=(0.0, 1.0, 0.0167),
    )

    await peer.send_positioning(snapshot)
    frame_ticket = node.comms.ticket
    assert node.comms.sent[1]["protocol"] == "payload-manual-position-frame"
    frame_envelope = {
        "v": 1,
        "id": "0" * 32,
        "kind": "message",
        "payload": node.comms.sent[1],
    }
    frame_encoded = json.dumps(
        frame_envelope, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    assert len(frame_encoded) <= 496
    frame_ticket.ack_future.set_result({"accepted": True})
    # The frame ACK releases the single-flight gate; the latest position can
    # then be queued immediately.
    await peer.send_positioning(snapshot)
    peer_id, payload = node.comms.sent
    assert peer_id == 2
    assert payload["protocol"] == "payload-manual-position"
    assert payload["position_m"] == {"x": 0.0198, "y": 0.2055}
    assert payload["tag_position_m"] == {"x": 0.0318, "y": 0.1995}

    envelope = {
        "v": 1,
        "id": "0" * 32,
        "kind": "message",
        "payload": payload,
    }
    encoded = json.dumps(envelope, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    assert len(encoded) <= 496
    router.close()


@pytest.mark.asyncio
async def test_peer_reports_waiting_for_fresh_ranges_without_a_position_estimate():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=2)
    snapshot = SimpleNamespace(
        tag_estimate=None,
        payload_position=None,
        status="waiting_for_fresh_ranges",
        range_count=2,
        rejected_range_count=4,
        anchors_xy_m=(
            ("anchor-3", -1.0, -0.6),
            ("anchor-4", 1.0, -0.6),
            ("anchor-5", 0.0, 1.2),
        ),
        payload_to_tag_xy_m=(0.012, -0.006),
        reference_origin_world_m=(0.0, -0.2, 0.081),
        reference_x_axis_world=(1.0, 0.0, 0.0),
        reference_y_axis_world=(0.0, 1.0, 0.0),
    )

    await peer.send_positioning(snapshot)
    frame_ticket = node.comms.ticket
    assert node.comms.sent[1]["protocol"] == "payload-manual-position-frame"
    frame_ticket.ack_future.set_result({"accepted": True})

    await peer.send_positioning(snapshot)

    peer_id, payload = node.comms.sent
    assert peer_id == 2
    assert payload == {
        "protocol": "payload-manual-position-status",
        "version": 1,
        "type": "status",
        "status": "waiting_for_fresh_ranges",
        "range_count": 2,
        "rejected_ranges": 4,
    }
    router.close()


@pytest.mark.asyncio
async def test_peer_does_not_retransmit_the_same_position_sequence():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=2)
    estimate = PositionEstimate(
        position=Position(0.0318, 0.1995, 0.081),
        residual_m=0.0052,
        anchor_ids=("anchor-3", "anchor-4", "anchor-5"),
        sequence=12345,
        device_monotonic_us=242_672_000,
        covariance_xy_m2=(0.000185, 0.000002, 0.000125),
        quality="good",
    )
    snapshot = SimpleNamespace(
        tag_estimate=estimate,
        payload_position=Position(0.0198, 0.2055, 0.081),
        heading=HeadingEstimate(0.3, 0.8, 3),
        status="tracking",
        anchors_xy_m=(
            ("anchor-3", -1.0, -0.6),
            ("anchor-4", 1.0, -0.6),
            ("anchor-5", 0.0, 1.2),
        ),
        payload_to_tag_xy_m=(0.012, -0.006),
        reference_origin_world_m=(0.0, -0.2, 0.081),
        reference_x_axis_world=(1.0, 0.0, 0.0),
        reference_y_axis_world=(0.0, 1.0, 0.0),
        range_count=3,
        rejected_range_count=0,
    )

    await peer.send_positioning(snapshot)
    frame_ticket = node.comms.ticket
    frame_ticket.ack_future.set_result({"accepted": True})
    await peer.send_positioning(snapshot)
    last_sent = node.comms.sent

    await peer.send_positioning(snapshot)
    assert node.comms.sent == last_sent
    router.close()


@pytest.mark.asyncio
async def test_positioning_sender_coalesces_backlog_and_sends_only_latest_estimate():
    node = FakeNode()
    router = D3kbleMessageRouter(node)
    peer = D3kbleManualControlPeer(node, router, controller_peer_id=2)

    def make_snapshot(sequence: int, x: float):
        estimate = PositionEstimate(
            position=Position(x + 0.012, 0.0, 0.081),
            residual_m=0.005,
            anchor_ids=("anchor-3", "anchor-4", "anchor-5"),
            sequence=sequence,
            device_monotonic_us=sequence * 20_000,
            covariance_xy_m2=(0.0001, 0.0, 0.0001),
            quality="good",
        )
        return SimpleNamespace(
            tag_estimate=estimate,
            payload_position=Position(x, 0.0, 0.081),
            heading=HeadingEstimate(0.0, 0.8, 3),
            status="tracking",
            anchors_xy_m=(
                ("anchor-3", -1.0, -0.6),
                ("anchor-4", 1.0, -0.6),
                ("anchor-5", 0.0, 1.2),
            ),
            payload_to_tag_xy_m=(0.012, 0.0),
            reference_origin_world_m=(0.0, 0.0, 0.081),
            reference_x_axis_world=(1.0, 0.0, 0.0),
            reference_y_axis_world=(0.0, 1.0, 0.0),
            range_count=3,
            rejected_range_count=0,
        )

    # The frame is sent once, and no position is queued until it is ACKed.
    await peer.send_positioning(make_snapshot(1, 0.0))
    assert len(node.comms.sent_messages) == 1
    frame_ticket = node.comms.ticket
    frame_ticket.ack_future.set_result({"accepted": True})

    # First position starts the reliable BLE transaction.
    await peer.send_positioning(make_snapshot(2, 0.1))
    assert len(node.comms.sent_messages) == 2
    position_ticket = node.comms.ticket
    assert node.comms.sent[1]["sequence"] == 2

    # While the ACK is pending, all intermediate updates replace one pending
    # snapshot in memory. They must not add messages to the D3K FIFO queue.
    await peer.send_positioning(make_snapshot(3, 0.2))
    await peer.send_positioning(make_snapshot(4, 0.3))
    await peer.send_positioning(make_snapshot(5, 0.4))
    assert len(node.comms.sent_messages) == 2
    assert peer.coalesced_positioning_updates == 3

    # Once sequence 2 is accepted, the next send carries the newest state
    # (sequence 5); sequences 3 and 4 are intentionally discarded.
    position_ticket.ack_future.set_result({"accepted": True})
    await asyncio.sleep(0.11)
    await peer.send_positioning(make_snapshot(5, 0.4))
    assert len(node.comms.sent_messages) == 3
    assert node.comms.sent[1]["sequence"] == 5

    router.close()
