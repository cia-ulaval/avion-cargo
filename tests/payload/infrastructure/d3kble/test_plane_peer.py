import asyncio

from payload.infrastructure.d3kble.plane_peer import D3kblePlanePeer
from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    DLZStateUpdated,
    DockRequest,
    PlaneHeadingUpdated,
    PayloadStatusRequested,
    PayloadStatusReport,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus
from shared.infrastructure.d3kble.codec import (
    decode_command_receipt,
    encode_dock_request,
    encode_message,
    encode_plane_event,
    encode_undock_request,
)
from shared.infrastructure.d3kble.router import D3kbleMessageRouter
from tests.d3kble_fakes import FakeIncomingRequest, FakeNode


def test_plane_peer_receives_heading_and_dlz_updates() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12)
        await peer.start()
        events = peer.events()
        heading_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        node.comms.message_handlers[0](
            type("Message", (), {
                "peer_id": 12,
                "payload": encode_message(
                    "plane.heading_updated",
                    {"heading": {"angle_rad": 1.1}, "sequence": 3},
                ),
            })()
        )
        assert await asyncio.wait_for(heading_task, timeout=1) == PlaneHeadingUpdated(Heading(1.1), 3)

        dlz_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        node.comms.message_handlers[0](
            type(
                "Message",
                (),
                {
                    "peer_id": 12,
                    "payload": encode_plane_event(DLZStateUpdated(DLZState(1, ()))),
                },
            )()
        )
        assert await asyncio.wait_for(dlz_task, timeout=1) == DLZStateUpdated(DLZState(1, ()))
        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_plane_peer_exposes_commands_and_returns_application_receipt() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12, command_ack_timeout=0.5)
        await peer.start()
        events = peer.events()
        event_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)

        command_task = asyncio.create_task(
            node.comms.request_handlers[0](
                FakeIncomingRequest(12, encode_undock_request("cmd-1", Position(0.2, 0.3, 0.0)))
            )
        )
        command = await asyncio.wait_for(event_task, timeout=1)
        assert command == UndockRequest("cmd-1", Position(0.2, 0.3, 0.0))
        await peer.acknowledge_command(CommandReceipt("cmd-1", accepted=True))
        response = await asyncio.wait_for(command_task, timeout=1)
        assert decode_command_receipt(response) == CommandReceipt("cmd-1", accepted=True)

        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_plane_peer_rejects_command_when_application_does_not_acknowledge() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12, command_ack_timeout=0.01)
        await peer.start()
        events = peer.events()
        event_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        response = await node.comms.request_handlers[0](
            FakeIncomingRequest(12, encode_dock_request("cmd-timeout"))
        )
        command = await asyncio.wait_for(event_task, timeout=1)
        receipt = decode_command_receipt(response)
        assert command == DockRequest("cmd-timeout")
        assert receipt.accepted is False
        assert receipt.rejection_reason.code == "application_ack_timeout"
        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_plane_peer_forwards_status_request_to_application_for_response() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12)
        await peer.start()
        events = peer.events()
        event_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)
        request = FakeIncomingRequest(12, encode_message("payload.request_status", {}, kind="request"))
        request_task = asyncio.create_task(node.comms.request_handlers[0](request))

        status_request = await asyncio.wait_for(event_task, timeout=1)
        assert isinstance(status_request, PayloadStatusRequested)
        assert status_request.request_id == request.message_id

        estimate = PositionEstimate(Position(0.1, 0.2, 0.0), 0.01, ("a", "b", "c"), 6)
        state = PayloadState(PayloadStatus.PARKED, DockingLatchState.OPEN)
        report = PayloadStatusReport(state=state, position=estimate, sequence=7)
        await peer.respond_status_request(status_request.request_id, report)
        response = await asyncio.wait_for(request_task, timeout=1)

        from shared.infrastructure.d3kble.codec import decode_status_report

        assert decode_status_report(response) == report
        await events.aclose()
        peer.close()

    asyncio.run(scenario())


def test_closing_plane_peer_wakes_pending_event_consumer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12)
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


def test_closing_plane_peer_rejects_pending_command_without_executing_it() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12)
        await peer.start()
        events = peer.events()
        event_task = asyncio.create_task(anext(events))
        await asyncio.sleep(0)

        command_task = asyncio.create_task(
            node.comms.request_handlers[0](FakeIncomingRequest(12, encode_dock_request("cmd-close")))
        )
        assert await asyncio.wait_for(event_task, timeout=1) == DockRequest("cmd-close")

        peer.close()
        response = await asyncio.wait_for(command_task, timeout=1)
        receipt = decode_command_receipt(response)
        assert receipt.command_id == "cmd-close"
        assert receipt.accepted is False
        assert receipt.rejection_reason.code == "adapter_closed"

    asyncio.run(scenario())


def test_timed_out_command_is_not_delivered_to_a_later_event_consumer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12, command_ack_timeout=0.01)
        await peer.start()

        response = await node.comms.request_handlers[0](
            FakeIncomingRequest(12, encode_dock_request("cmd-no-consumer"))
        )
        receipt = decode_command_receipt(response)
        assert receipt.accepted is False
        assert receipt.rejection_reason.code == "application_ack_timeout"

        # The request was rejected before any application event consumer was
        # running. It must not be replayed when a consumer starts later.
        events = peer.events()
        try:
            await asyncio.wait_for(anext(events), timeout=0.03)
        except asyncio.TimeoutError:
            pass
        else:
            raise AssertionError("timed-out command must not be delivered later")
        finally:
            await events.aclose()
            peer.close()

    asyncio.run(scenario())


def test_timed_out_status_request_is_not_delivered_to_a_later_event_consumer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        peer = D3kblePlanePeer(node, router, plane_peer_id=12, status_response_timeout=0.01)
        await peer.start()

        request = FakeIncomingRequest(12, encode_message("payload.request_status", {}, kind="request"))
        try:
            await node.comms.request_handlers[0](request)
        except TimeoutError:
            pass
        else:
            raise AssertionError("status request should time out without an application response")

        events = peer.events()
        try:
            await asyncio.wait_for(anext(events), timeout=0.03)
        except asyncio.TimeoutError:
            pass
        else:
            raise AssertionError("timed-out status query must not be delivered later")
        finally:
            await events.aclose()
            peer.close()

    asyncio.run(scenario())
