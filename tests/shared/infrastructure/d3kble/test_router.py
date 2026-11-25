import asyncio
from types import SimpleNamespace

import pytest

from shared.infrastructure.d3kble.router import D3kbleMessageRouter


class FakeComms:
    def __init__(self):
        self.message_handlers = []
        self.request_handlers = []
        self.loop = None

    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def on_message(self, callback=None):
        self.message_handlers.append(callback)
        return callback

    def on_request(self, callback=None):
        self.request_handlers.append(callback)
        return callback


class FakeNode:
    def __init__(self):
        self.comms = FakeComms()


class FakeIncomingRequest:
    def __init__(self, peer_id, payload):
        self.peer_id = peer_id
        self.payload = payload
        self.message_id = "request-1"
        self.failure = None

    def fail(self, reason, content=None):
        self.failure = (reason, content)


def test_router_registers_one_callback_pair_and_routes_per_peer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        received = []
        await router.start()
        route_a = router.register_peer(21, message_handler=lambda message: received.append((21, message.payload)))
        route_b = router.register_peer(22, message_handler=lambda message: received.append((22, message.payload)))

        assert len(node.comms.message_handlers) == 1
        assert len(node.comms.request_handlers) == 1
        node.comms.message_handlers[0](SimpleNamespace(peer_id=22, payload={"v": 1}))
        assert received == [(22, {"v": 1})]

        route_b.close()
        node.comms.message_handlers[0](SimpleNamespace(peer_id=22, payload={"v": 2}))
        node.comms.message_handlers[0](SimpleNamespace(peer_id=21, payload={"v": 3}))
        assert received == [(22, {"v": 1}), (21, {"v": 3})]
        route_a.close()
        router.close()

    asyncio.run(scenario())


def test_router_dispatches_requests_and_fails_unregistered_peer() -> None:
    async def scenario():
        node = FakeNode()
        router = D3kbleMessageRouter(node)
        await router.start()
        await_results = []

        async def handle(request):
            await_results.append(request.payload)
            return {"accepted": True}

        router.register_peer(40, request_handler=handle)
        response = await node.comms.request_handlers[0](FakeIncomingRequest(40, {"command": "dock"}))
        assert response == {"accepted": True}
        assert await_results == [{"command": "dock"}]

        unknown = FakeIncomingRequest(41, {})
        assert await node.comms.request_handlers[0](unknown) is None
        assert unknown.failure[0].startswith("no request handler")

    asyncio.run(scenario())


def test_router_rejects_duplicate_routes_and_wrong_event_loop() -> None:
    async def scenario():
        router = D3kbleMessageRouter(FakeNode())
        await router.start()
        router.register_peer(9, message_handler=lambda _message: None)
        with pytest.raises(ValueError, match="already registered"):
            router.register_peer(9, message_handler=lambda _message: None)
        router.close()
        with pytest.raises(RuntimeError, match="closed"):
            await router.start()

    asyncio.run(scenario())


def test_router_requires_start_before_registering_a_route() -> None:
    router = D3kbleMessageRouter(FakeNode())
    with pytest.raises(RuntimeError, match="start the router"):
        router.register_peer(25, message_handler=lambda _message: None)
