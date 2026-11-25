from __future__ import annotations

import asyncio
import math
from collections import deque
from collections.abc import AsyncIterator
from typing import Any

from d3kble import D3KNode
from loguru import logger

from payload.domain.body_velocity import BodyVelocity
from payload_manual.domain.manual_control import ManualControlCommand
from shared.infrastructure.d3kble.router import D3kbleMessageRouter, D3kbleRouteRegistration

PROTOCOL_NAME = "payload-manual-control"
PROTOCOL_VERSION = 1
_EVENTS_CLOSED = object()


class ManualControlProtocolError(ValueError):
    """An incoming manual-control message does not match the wire contract."""


def encode_manual_control_message(command: ManualControlCommand) -> dict[str, Any]:
    """Encode a command as a JSON-compatible d3kble application payload."""
    data: dict[str, Any] = {
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "type": command.command_type,
        "session_id": command.session_id,
        "sequence": command.sequence,
    }
    if command.velocity is not None:
        data.update(
            {
                "vx_m_s": command.velocity.vx_m_s,
                "vy_m_s": command.velocity.vy_m_s,
                "yaw_rate_rad_s": command.velocity.yaw_rate_rad_s,
            }
        )
    return data


def decode_manual_control_message(payload: Any) -> ManualControlCommand:
    if not isinstance(payload, dict):
        raise ManualControlProtocolError("manual-control payload must be an object")
    if payload.get("protocol") != PROTOCOL_NAME:
        raise ManualControlProtocolError("unexpected manual-control protocol")
    version = payload.get("version")
    if isinstance(version, bool) or not isinstance(version, int) or version != PROTOCOL_VERSION:
        raise ManualControlProtocolError(f"unsupported manual-control protocol version: {version!r}")

    command_type = payload.get("type")
    if not isinstance(command_type, str) or command_type not in {"arm", "drive", "stop", "disarm", "status"}:
        raise ManualControlProtocolError(f"unsupported manual-control command: {command_type!r}")
    session_id = payload.get("session_id")
    if not isinstance(session_id, str) or not session_id or len(session_id) > 64:
        raise ManualControlProtocolError(
            "manual-control session_id must be a non-empty string of at most 64 characters"
        )
    sequence = payload.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ManualControlProtocolError("manual-control sequence must be a non-negative integer")

    velocity_fields = {"vx_m_s", "vy_m_s", "yaw_rate_rad_s"}
    common_fields = {"protocol", "version", "type", "session_id", "sequence"}
    if command_type == "drive":
        expected = common_fields | velocity_fields
        if set(payload) != expected:
            raise ManualControlProtocolError("drive command must contain exactly the three velocity fields")
        values: list[float] = []
        for name in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s"):
            value = payload[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ManualControlProtocolError(f"{name} must be a finite number")
            values.append(float(value))
        velocity = BodyVelocity(*values)
    else:
        if set(payload) != common_fields:
            raise ManualControlProtocolError(f"{command_type} command must not include velocity fields")
        velocity = None

    return ManualControlCommand(session_id, sequence, command_type, velocity)


class D3kbleManualControlPeer:
    """Receive one-way manual-control messages from one configured BLE peer."""

    def __init__(
        self,
        node: D3KNode,
        router: D3kbleMessageRouter,
        *,
        controller_peer_id: int,
        queue_size: int = 64,
    ) -> None:
        if (
            isinstance(controller_peer_id, bool)
            or not isinstance(controller_peer_id, int)
            or not 1 <= controller_peer_id <= 65534
        ):
            raise ValueError("controller_peer_id must be an integer in 1..65534")
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        self._node = node
        self._router = router
        self.controller_peer_id = controller_peer_id
        self._queue_size = queue_size
        self._pending: deque[ManualControlCommand | object] = deque()
        self._available: asyncio.Event | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._registration: D3kbleRouteRegistration | None = None
        self._consumer_active = False
        self._closed = False
        self.dropped_commands = 0
        self.invalid_commands = 0

    async def start(self) -> None:
        if self._closed:
            raise RuntimeError("manual-control peer is closed")
        loop = asyncio.get_running_loop()
        if self._registration is not None:
            if self._loop is not loop:
                raise RuntimeError("manual-control peer is bound to another event loop")
            return
        await self._router.start()
        self._loop = loop
        self._available = asyncio.Event()
        self._registration = self._router.register_peer(
            self.controller_peer_id,
            message_handler=self._on_message,
        )

    async def send_status(self, session_id: str, sequence: int, armed: bool, session_active: bool) -> None:
        """Report the payload's actual arm state to the connected controller node."""
        payload = {
            "protocol": "payload-manual-control-status",
            "version": 1,
            "type": "status",
            "session_id": session_id,
            "sequence": sequence,
            "armed": bool(armed),
            "session_active": bool(session_active),
        }
        try:
            ticket = self._node.comms.send(self.controller_peer_id, payload)
        except Exception:
            logger.exception(
                "could not queue manual-control state for peer {}",
                self.controller_peer_id,
            )
            return

        def report_delivery_result(future) -> None:
            try:
                future.result()
            except Exception as exc:
                logger.warning(
                    "manual-control state delivery to peer {} failed: {}",
                    self.controller_peer_id,
                    exc,
                )

        ticket.ack_future.add_done_callback(report_delivery_result)

    async def events(self) -> AsyncIterator[ManualControlCommand]:
        await self.start()
        if self._consumer_active:
            raise RuntimeError("manual-control peer supports one active events() consumer")
        self._consumer_active = True
        assert self._available is not None
        try:
            while True:
                if not self._pending:
                    self._available.clear()
                    # No await occurred between the empty check and clear, so
                    # an enqueue cannot be lost on this event loop.
                    if not self._pending:
                        await self._available.wait()
                while self._pending:
                    event = self._pending.popleft()
                    if event is _EVENTS_CLOSED:
                        return
                    yield event  # type: ignore[misc]
        finally:
            self._consumer_active = False

    def _on_message(self, message: Any) -> None:
        try:
            command = decode_manual_control_message(message.payload)
        except Exception:
            self.invalid_commands += 1
            logger.exception("discarding invalid manual-control message from peer %s", self.controller_peer_id)
            return
        loop = self._loop
        if loop is None or loop.is_closed() or self._closed:
            return
        try:
            loop.call_soon_threadsafe(self._enqueue, command)
        except RuntimeError:
            logger.debug("event loop closed while receiving manual-control message")

    def _enqueue(self, command: ManualControlCommand) -> None:
        if self._available is None or self._closed:
            return

        # Drive messages are setpoints, not a history to replay. Replace an
        # already-pending drive command with the newest value while retaining
        # arm/stop/disarm transitions in order.
        if command.command_type == "drive":
            for index in range(len(self._pending) - 1, -1, -1):
                pending = self._pending[index]
                if isinstance(pending, ManualControlCommand) and pending.command_type == "drive":
                    self._pending[index] = command
                    self._available.set()
                    return

        if command.command_type == "disarm":
            # Disarm supersedes every queued command, including an earlier arm.
            self.dropped_commands += len(self._pending)
            self._pending.clear()
        elif command.command_type == "arm":
            # A fresh arm must not be preceded by an old queued drive target.
            retained = deque(
                pending
                for pending in self._pending
                if not (isinstance(pending, ManualControlCommand) and pending.command_type == "drive")
            )
            self.dropped_commands += len(self._pending) - len(retained)
            self._pending = retained
        elif command.command_type == "stop":
            retained = deque(
                pending
                for pending in self._pending
                if not (isinstance(pending, ManualControlCommand) and pending.command_type == "drive")
            )
            self.dropped_commands += len(self._pending) - len(retained)
            self._pending = retained
            if len(self._pending) >= self._queue_size:
                self.dropped_commands += len(self._pending)
                self._pending.clear()
        elif len(self._pending) >= self._queue_size:
            drive_index = next(
                (
                    index
                    for index, pending in enumerate(self._pending)
                    if isinstance(pending, ManualControlCommand) and pending.command_type == "drive"
                ),
                None,
            )
            if drive_index is not None:
                del self._pending[drive_index]
                self.dropped_commands += 1
            else:
                self.dropped_commands += 1
                logger.warning("manual-control mailbox full; dropping command sequence {}", command.sequence)
                return

        self._pending.append(command)
        self._available.set()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._registration is not None:
            self._registration.close()
            self._registration = None

        def finish_close() -> None:
            self._pending.clear()
            self._pending.append(_EVENTS_CLOSED)
            if self._available is not None:
                self._available.set()

        loop = self._loop
        if loop is None or loop.is_closed():
            return
        try:
            running_loop = asyncio.get_running_loop()
        except RuntimeError:
            running_loop = None
        if running_loop is loop:
            finish_close()
        else:
            try:
                loop.call_soon_threadsafe(finish_close)
            except RuntimeError:
                pass
