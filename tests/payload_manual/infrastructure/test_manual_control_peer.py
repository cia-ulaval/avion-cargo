from types import SimpleNamespace

import pytest

from payload.domain.body_velocity import BodyVelocity
from payload_manual.domain.manual_control import ManualControlCommand
from payload_manual.infrastructure.d3kble.manual_control_peer import (
    D3kbleManualControlPeer,
    ManualControlProtocolError,
    decode_manual_control_message,
    encode_manual_control_message,
)
from shared.infrastructure.d3kble.router import D3kbleMessageRouter


class FakeTicket:
    def wait_ack(self, timeout):
        self.timeout = timeout
        return True


class FakeComms:
    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def send(self, peer_id, payload):
        self.sent = (peer_id, payload)
        return FakeTicket()

    def on_message(self, callback=None):
        self.message_handler = callback

    def on_request(self, callback=None):
        self.request_handler = callback


class FakeNode:
    def __init__(self):
        self.comms = FakeComms()


def test_manual_control_protocol_round_trips_drive_and_state_commands():
    command = ManualControlCommand("test-session", 7, "drive", BodyVelocity(0.12, -0.04, 0.5))
    assert decode_manual_control_message(encode_manual_control_message(command)) == command

    for name in ("arm", "stop", "disarm", "status"):
        command = ManualControlCommand("test-session", 8, name)
        assert decode_manual_control_message(encode_manual_control_message(command)) == command


@pytest.mark.parametrize(
    "payload",
    [
        {"protocol": "wrong", "version": 1, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": True, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": 2, "type": "arm", "session_id": "s", "sequence": 0},
        {"protocol": "payload-manual-control", "version": 1, "type": "arm", "session_id": "s", "sequence": True},
        {"protocol": "payload-manual-control", "version": 1, "type": "arm", "sequence": 0},
        {
            "protocol": "payload-manual-control", "version": 1,
            "type": "drive", "session_id": "s", "sequence": 0,
            "vx_m_s": float("nan"), "vy_m_s": 0.0, "yaw_rate_rad_s": 0.0,
        },
        {
            "protocol": "payload-manual-control", "version": 1,
            "type": "drive", "session_id": "s", "sequence": 0,
            "vx_m_s": True, "vy_m_s": 0.0, "yaw_rate_rad_s": 0.0,
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
    handler(SimpleNamespace(
        peer_id=51,
        payload=encode_manual_control_message(ManualControlCommand("test-session", 1, "arm")),
    ))
    handler(SimpleNamespace(
        peer_id=50,
        payload=encode_manual_control_message(
            ManualControlCommand("test-session", 2, "drive", BodyVelocity(0.1, 0, 0))
        ),
    ))
    handler(SimpleNamespace(
        peer_id=50,
        payload=encode_manual_control_message(
            ManualControlCommand("test-session", 3, "drive", BodyVelocity(0.2, 0, 0))
        ),
    ))

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
    router.close()
