from __future__ import annotations

import sys
import time
from types import ModuleType, SimpleNamespace
from typing import Any

from d3k_diagnostics.main import (
    DeviceConfiguration,
    _BleDiagnostics,
    _exchange_ping,
    _handle_ping_request,
)
from shared.infrastructure.d3kble.codec import decode_message, encode_message


class _FakeTicket:
    def __init__(self, response: Any) -> None:
        self._response = response

    async def _wait(self):
        return self._response

    def __await__(self):
        return self._wait().__await__()


class _FakeComms:
    def __init__(self, node: "_FakeNode") -> None:
        self.node = node
        self.requests: list[tuple[int, dict[str, Any]]] = []

    def request(self, peer_id: int, payload: dict[str, Any]) -> _FakeTicket:
        self.requests.append((peer_id, payload))
        message_type, data = decode_message(payload)
        assert message_type == "diagnostics.ping"
        assert data
        if peer_id in self.node.fail_peers:
            return _FakeTicket(SimpleNamespace(status="error", error="simulated peer failure", content=None))
        remote = _FakeNode.by_id[peer_id]
        route = remote.router.routes.get(self.node.device_id)
        if route is None:
            return _FakeTicket(SimpleNamespace(status="error", error="no route", content=None))
        request = SimpleNamespace(peer_id=self.node.device_id, payload=payload)
        try:
            content = route(request)
        except Exception as exc:
            return _FakeTicket(SimpleNamespace(status="error", error=str(exc), content=None))
        return _FakeTicket(SimpleNamespace(status="ok", error=None, content=content))


class _FakeRegistration:
    def __init__(self, router: "_FakeRouter", peer_id: int) -> None:
        self.router = router
        self.peer_id = peer_id

    def close(self) -> None:
        self.router.routes.pop(self.peer_id, None)


class _FakeRouter:
    instances: list["_FakeRouter"] = []

    def __init__(self, node: "_FakeNode") -> None:
        self.node = node
        self.routes: dict[int, Any] = {}
        self.started = False
        self.closed = False
        node.router = self
        self.instances.append(self)

    async def start(self) -> None:
        self.started = True

    def register_peer(self, peer_id: int, *, message_handler=None, request_handler=None):
        assert message_handler is None
        assert request_handler is not None
        self.routes[peer_id] = request_handler
        return _FakeRegistration(self, peer_id)

    def close(self) -> None:
        self.closed = True
        self.routes.clear()


class _FakeNode:
    by_id: dict[int, "_FakeNode"] = {}

    def __init__(self, device_id: int, *, fail_peers: tuple[int, ...] = ()) -> None:
        self.device_id = device_id
        self.fail_peers = set(fail_peers)
        self.router: _FakeRouter | None = None
        self.comms = _FakeComms(self)
        self.by_id[device_id] = self

    def get_state(self, refresh: bool = False):
        return SimpleNamespace(runtime=SimpleNamespace(values={"device_id": self.device_id}))


def _install_fake_router(monkeypatch) -> None:
    _FakeRouter.instances.clear()
    module = ModuleType("shared.infrastructure.d3kble.router")
    module.D3kbleMessageRouter = _FakeRouter
    monkeypatch.setitem(sys.modules, "shared.infrastructure.d3kble.router", module)


def _wait_until(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_request_handler_returns_the_pong_payload_for_library_correlation():
    ping = encode_message(
        "diagnostics.ping",
        {"sequence": 7, "sent_at_ns": 123456789},
        kind="request",
    )

    pong = _handle_ping_request(SimpleNamespace(peer_id=2, payload=ping))

    assert decode_message(pong) == (
        "diagnostics.pong",
        {"sequence": 7, "sent_at_ns": 123456789},
    )


def test_exchange_uses_request_ticket_and_validates_correlated_pong():
    class Comms:
        def __init__(self):
            self.sent = None

        def request(self, peer_id, payload):
            self.sent = (peer_id, payload)
            message_type, data = decode_message(payload)
            assert message_type == "diagnostics.ping"
            response = _handle_ping_request(SimpleNamespace(peer_id=peer_id, payload=payload))
            return _FakeTicket(SimpleNamespace(status="ok", error=None, content=response))

    class Node:
        comms = Comms()

    import asyncio

    node = Node()
    rtt_ms = asyncio.run(_exchange_ping(node, 18, 4, 0.5))
    peer_id, request = node.comms.sent
    message_type, data = decode_message(request)

    assert peer_id == 18
    assert message_type == "diagnostics.ping"
    assert rtt_ms >= 0
    assert data["sequence"] == 4


def test_multi_peer_service_uses_one_router_per_node_and_keeps_peers_independent(monkeypatch):
    _install_fake_router(monkeypatch)
    _FakeNode.by_id.clear()
    hub = _FakeNode(2, fail_peers=(18,))
    peer_b = _FakeNode(18)
    peer_c = _FakeNode(19)
    sessions = [
        (DeviceConfiguration("hub", "/tmp/a", peer_ids=(18, 19)), hub, None),
        (DeviceConfiguration("b", "/tmp/b", peer_id=2), peer_b, None),
        (DeviceConfiguration("c", "/tmp/c", peer_id=2), peer_c, None),
    ]
    diagnostics = _BleDiagnostics(sessions, interval_s=0.02, timeout_s=0.2)
    diagnostics.start()

    try:
        assert _wait_until(lambda: len(hub.comms.requests) >= 2)
        assert {peer_id for peer_id, _payload in hub.comms.requests} == {18, 19}
        assert len(_FakeRouter.instances) == 3
        assert len({id(router) for router in _FakeRouter.instances}) == 3
        assert set(hub.router.routes) == {18, 19}
        assert set(peer_b.router.routes) == {2}
        assert set(peer_c.router.routes) == {2}
        # Peer 18 deliberately returns an error; peer 19 must continue receiving
        # independent requests rather than being cancelled with the failed task.
        assert _wait_until(
            lambda: sum(peer == 19 for peer, _ in hub.comms.requests) >= 2,
            timeout_s=1.0,
        )
    finally:
        diagnostics.stop()

    assert all(router.closed for router in _FakeRouter.instances)
