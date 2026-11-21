from __future__ import annotations

import asyncio
import sys
import time
from types import ModuleType
from types import SimpleNamespace
from typing import Any

from d3k_diagnostics.main import _BlePingPong
from shared.infrastructure.d3kble.codec import decode_message, encode_message


class _FakeSendTicket:
    def wait_ack(self, timeout: float | None = None) -> dict[str, Any]:
        return {"accepted": True}


class _FakeComms:
    def __init__(self) -> None:
        self.message_handler = None
        self.loop = None
        self.sent: list[tuple[int, dict[str, Any]]] = []

    def bind_asyncio_loop(self, loop) -> None:
        self.loop = loop

    def on_message(self, callback=None):
        self.message_handler = callback
        return callback

    def on_request(self, callback=None):
        return callback

    def send(self, peer_id: int, payload: dict[str, Any]) -> _FakeSendTicket:
        self.sent.append((peer_id, payload))
        return _FakeSendTicket()

    def deliver(self, peer_id: int, payload: dict[str, Any]) -> None:
        assert self.message_handler is not None
        self.message_handler(SimpleNamespace(peer_id=peer_id, payload=payload))


class _FakeRegistration:
    def close(self) -> None:
        return None


class _FakeRouter:
    def __init__(self, node: Any) -> None:
        self._node = node
        self._peer_id: int | None = None
        self._handler = None

    async def start(self) -> None:
        self._node.comms.bind_asyncio_loop(asyncio.get_running_loop())
        self._node.comms.on_message(self._dispatch)

    def register_peer(self, peer_id: int, *, message_handler=None, request_handler=None):
        self._peer_id = peer_id
        self._handler = message_handler
        return _FakeRegistration()

    def _dispatch(self, message) -> None:
        if message.peer_id == self._peer_id and self._handler is not None:
            self._handler(message)

    def close(self) -> None:
        self._handler = None


class _FakeNode:
    def __init__(self) -> None:
        self.comms = _FakeComms()


def _install_fake_router(monkeypatch) -> None:
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


def test_responder_echoes_ping_using_the_shared_application_codec(monkeypatch) -> None:
    _install_fake_router(monkeypatch)
    initiator = _FakeNode()
    responder = _FakeNode()
    ping_pong = _BlePingPong(
        initiator,
        18,
        responder,
        2,
        initiator_name="vtol",
        interval_s=5.0,
        timeout_s=0.5,
    )
    ping_pong.start()

    try:
        ping = encode_message(
            "diagnostics.ping",
            {"sequence": 7, "sent_at_ns": 123456789},
        )
        responder.comms.deliver(2, ping)

        assert _wait_until(lambda: bool(responder.comms.sent))
        peer_id, payload = responder.comms.sent[0]
        message_type, data = decode_message(payload)

        assert peer_id == 2
        assert message_type == "diagnostics.pong"
        assert data == {"sequence": 7, "sent_at_ns": 123456789}
    finally:
        ping_pong.stop()


def test_initiator_sends_only_one_unanswered_ping_and_waits_for_matching_pong(monkeypatch) -> None:
    _install_fake_router(monkeypatch)
    initiator = _FakeNode()
    responder = _FakeNode()
    ping_pong = _BlePingPong(
        initiator,
        18,
        responder,
        2,
        initiator_name="vtol",
        interval_s=0.25,
        timeout_s=0.5,
    )
    ping_pong.start()

    try:
        assert _wait_until(lambda: len(initiator.comms.sent) == 1)
        first_peer_id, first_payload = initiator.comms.sent[0]
        first_type, first_data = decode_message(first_payload)
        assert first_peer_id == 18
        assert first_type == "diagnostics.ping"
        assert first_data["sequence"] == 1

        time.sleep(0.05)
        assert len(initiator.comms.sent) == 1

        pong = encode_message(
            "diagnostics.pong",
            {
                "sequence": first_data["sequence"],
                "sent_at_ns": first_data["sent_at_ns"],
            },
        )
        initiator.comms.deliver(18, pong)

        assert _wait_until(lambda: len(initiator.comms.sent) == 2, timeout_s=0.5)
        second_type, second_data = decode_message(initiator.comms.sent[1][1])
        assert second_type == "diagnostics.ping"
        assert second_data["sequence"] == 2
    finally:
        ping_pong.stop()


def test_initiator_stops_after_timeout_instead_of_queuing_more_pings(monkeypatch) -> None:
    _install_fake_router(monkeypatch)
    initiator = _FakeNode()
    responder = _FakeNode()
    ping_pong = _BlePingPong(
        initiator,
        18,
        responder,
        2,
        initiator_name="vtol",
        interval_s=0.05,
        timeout_s=0.08,
    )
    ping_pong.start()

    try:
        assert _wait_until(lambda: len(initiator.comms.sent) == 1)
        time.sleep(0.2)
        assert len(initiator.comms.sent) == 1
    finally:
        ping_pong.stop()
