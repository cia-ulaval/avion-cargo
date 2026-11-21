import asyncio

from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    DockingSucceeded,
    PayloadPositionUpdated,
    PayloadSnapshot,
    PayloadStateUpdated,
    PayloadStatusReport,
    PlaneHeadingUpdated,
    ParkingCompleted,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus
from shared.infrastructure.d3kble.codec import (
    decode_message,
    encode_payload_event,
    encode_status_report,
)
from shared.infrastructure.d3kble.router import D3kbleMessageRouter
from tests.d3kble_fakes import FakeNode
from vtol.infrastructure.d3kble.payload_peer import D3kblePayloadPeer


def test_payload_peer_sends_commands_and_decodes_status_response() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePayloadPeer(node, router, payload_id="payload-7", peer_id=27)

        receipt = await peer.request_undock(Position(0.5, -0.2, 0.0))
        assert receipt.accepted is True
        sent_peer, request = node.comms.requests[-1]
        message_type, data = decode_message(request)
        assert sent_peer == 27
        assert message_type == "plane.command.undock"
        assert data["target_position"] == {"x": 0.5, "y": -0.2, "z": 0.0}

        state = PayloadState(PayloadStatus.DOCKED, DockingLatchState.CLOSED)
        estimate = PositionEstimate(Position(0.0, 0.1, 0.0), 0.02, ("a", "b", "c"), 3)
        node.comms.status_report = PayloadStatusReport(state, estimate, sequence=3)
        status = await peer.request_status()
        assert status.state == state
        assert status.position == estimate
        assert peer.payload_id == "payload-7"
        assert peer.ble_peer_id == 27
        peer.close()

    asyncio.run(scenario())


def test_payload_peer_receives_reports_from_only_its_assigned_peer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePayloadPeer(node, router, payload_id="payload-7", peer_id=27)
        await peer.start()
        events = peer.events()
        pending = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        estimate = PositionEstimate(Position(0.2, -0.1, 0.0), 0.01, ("a", "b", "c"), 5)
        message = encode_payload_event(PayloadPositionUpdated(estimate))

        node.comms.message_handlers[0](type("Message", (), {"peer_id": 28, "payload": message})())
        node.comms.message_handlers[0](type("Message", (), {"peer_id": 27, "payload": message})())
        event = await asyncio.wait_for(pending, timeout=1)
        assert event == PayloadPositionUpdated(estimate)

        pending_state = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        state = PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN)
        node.comms.message_handlers[0](
            type("Message", (), {
                "peer_id": 27,
                "payload": encode_payload_event(PayloadStateUpdated(state, sequence=9)),
            })()
        )
        assert await asyncio.wait_for(pending_state, timeout=1) == PayloadStateUpdated(state, 9)
        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_payload_peer_publishes_heading_and_dlz_state_as_acknowledged_messages() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePayloadPeer(node, router, payload_id="payload-7", peer_id=27)
        await peer.publish_heading(Heading(0.75), sequence=4)
        await peer.publish_dlz_state(
            DLZState(
                revision=2,
                payloads=(
                    PayloadSnapshot(
                        "payload-7",
                        PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN),
                        None,
                        sequence=3,
                    ),
                ),
            )
        )
        assert [peer_id for peer_id, _message in node.comms.sent] == [27, 27]
        assert decode_message(node.comms.sent[0][1])[0] == "plane.heading_updated"
        assert decode_message(node.comms.sent[1][1])[0] == "plane.dlz_state_updated"
        peer.close()

    asyncio.run(scenario())


def test_payload_peer_receives_operation_reports() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePayloadPeer(node, router, payload_id="payload-7", peer_id=27)
        await peer.start()
        events = peer.events()
        pending = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        event = ParkingCompleted("cmd-1", Position(0.1, 0.2, 0.0))
        node.comms.message_handlers[0](
            type("Message", (), {"peer_id": 27, "payload": encode_payload_event(event)})()
        )
        assert await asyncio.wait_for(pending, timeout=1) == event
        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_closing_payload_peer_wakes_pending_event_consumer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePayloadPeer(node, router, payload_id="payload-7", peer_id=27)
        await peer.start()
        events = peer.events()
        pending = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        peer.close()
        try:
            await asyncio.wait_for(pending, timeout=1)
        except StopAsyncIteration:
            pass
        else:
            raise AssertionError("closed event stream should stop")

    asyncio.run(scenario())
