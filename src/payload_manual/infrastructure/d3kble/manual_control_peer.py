from __future__ import annotations

import asyncio
import math
import time
from collections import deque
from collections.abc import AsyncIterator
from typing import Any

from d3kble import D3KNode
from loguru import logger

from payload_manual.domain.manual_control import ReferenceVelocity
from payload_manual.domain.manual_control import ManualControlCommand
from shared.infrastructure.d3kble.router import D3kbleMessageRouter, D3kbleRouteRegistration

PROTOCOL_NAME = "payload-manual-control"
PROTOCOL_VERSION = 2
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
        velocity = ReferenceVelocity(*values)
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
        self._positioning_frame_sent = False
        self._positioning_frame_sent_at = 0.0
        # Keep at most one positioning-related application message in flight.
        # New range events replace this latest snapshot instead of accumulating
        # stale position messages in D3K's FIFO BLE transmit queue.
        self._positioning_ticket = None
        self._positioning_ticket_kind: str | None = None
        self._positioning_ticket_sequence: int | None = None
        self._positioning_ticket_status: tuple[str, int, int] | None = None
        self._positioning_ticket_sent_at = 0.0
        self._latest_positioning_snapshot = None
        self._last_positioning_sequence_acked: int | None = None
        self._last_positioning_status_acked: tuple[str, int, int] | None = None
        self._last_positioning_status_acked_at = 0.0
        self._last_positioning_acked_at = 0.0
        self._positioning_retry_after = 0.0
        self._positioning_delivery_failures = 0
        self.coalesced_positioning_updates = 0
        self._positioning_calls_since_log = 0
        self._last_positioning_diagnostic_log_at = 0.0
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

    async def send_positioning(self, snapshot) -> None:
        """Send the newest positioning state with single-flight backpressure.

        D3K application messages are reliable and its BLE sender has a FIFO
        transmit queue. Sending a new position on every timer tick without
        waiting for the previous application ACK turns a live stream into a
        minutes-old backlog when BLE throughput drops below the producer rate.
        Keep only the newest snapshot while one message is in flight.
        """
        self._latest_positioning_snapshot = snapshot
        now = time.monotonic()
        self._positioning_calls_since_log += 1

        # Retire the previous message only after the receiver has acknowledged
        # that it has accepted it. Its ticket is the sole positioning message
        # allowed into the D3K queue at any time.
        if self._positioning_ticket is not None:
            ticket = self._positioning_ticket
            if not ticket.ack_future.done():
                self.coalesced_positioning_updates += 1
                if now - self._last_positioning_diagnostic_log_at >= 1.0:
                    self._positioning_calls_since_log = 0
                    self._last_positioning_diagnostic_log_at = now
                return

            kind = self._positioning_ticket_kind
            sequence = self._positioning_ticket_sequence
            status_key = self._positioning_ticket_status
            self._positioning_ticket = None
            self._positioning_ticket_kind = None
            self._positioning_ticket_sequence = None
            self._positioning_ticket_status = None
            ack_latency_ms = (now - self._positioning_ticket_sent_at) * 1000.0
            try:
                ticket.ack_future.result()
            except Exception as exc:
                self._positioning_delivery_failures += 1
                delay = min(5.0, 0.25 * (2 ** min(self._positioning_delivery_failures - 1, 5)))
                self._positioning_retry_after = now + delay
                if kind == "frame":
                    self._positioning_frame_sent = False
                    self._positioning_frame_sent_at = 0.0
                logger.warning(
                    "[positioning-tx] delivery failed: peer={} kind={} sequence={} ack_wait_ms={:.1f} "
                    "retry_in_s={:.2f} newest_estimate_sequence={} error={}",
                    self.controller_peer_id,
                    kind,
                    sequence,
                    ack_latency_ms,
                    delay,
                    getattr(getattr(snapshot, "tag_estimate", None), "sequence", None),
                    exc,
                )
            else:
                self._positioning_delivery_failures = 0
                self._positioning_retry_after = 0.0
                if kind == "frame":
                    self._positioning_frame_sent = True
                    self._positioning_frame_sent_at = now
                elif kind == "position" and sequence is not None:
                    self._last_positioning_sequence_acked = sequence
                    # Rate-limit from enqueue time. If an ACK took longer than
                    # the target interval, transmit the newest pending state
                    # immediately rather than adding another 100 ms delay.
                    self._last_positioning_acked_at = self._positioning_ticket_sent_at
                elif kind == "status" and status_key is not None:
                    self._last_positioning_status_acked = status_key
                    self._last_positioning_status_acked_at = now

        if now < self._positioning_retry_after:
            return

        snapshot = self._latest_positioning_snapshot
        # Send the static frame once at startup and then occasionally so a UI
        # that reconnects can recover its map geometry. Frame refreshes share
        # the same single-flight gate as positions and status messages.
        frame_refresh_due = (
            not self._positioning_frame_sent
            or now - self._positioning_frame_sent_at >= 30.0
        )
        if frame_refresh_due:
            frame_payload = {
                "protocol": "payload-manual-position-frame",
                "version": 1,
                "type": "reference_frame",
                "anchors_xy_m": [
                    {"id": anchor_id, "x": round(x, 3), "y": round(y, 3)}
                    for anchor_id, x, y in snapshot.anchors_xy_m
                ],
                "payload_to_tag_xy_m": [round(value, 3) for value in snapshot.payload_to_tag_xy_m],
                "tag_height_m": round(snapshot.reference_origin_world_m[2], 3),
                "origin_world_m": [round(value, 3) for value in snapshot.reference_origin_world_m],
                "x_axis_world": [round(value, 4) for value in snapshot.reference_x_axis_world],
                "y_axis_world": [round(value, 4) for value in snapshot.reference_y_axis_world],
            }
            self._queue_positioning_message(frame_payload, kind="frame")
            return

        estimate = snapshot.tag_estimate
        if estimate is None or snapshot.payload_position is None:
            status_key = (
                str(snapshot.status),
                int(getattr(snapshot, "range_count", 0)),
                int(getattr(snapshot, "rejected_range_count", 0)),
            )
            # Report state changes immediately, with a 1 Hz heartbeat while the
            # estimator is waiting. Do not queue an endless stream of identical
            # status messages if the receiver or radio is unavailable.
            if (
                status_key != self._last_positioning_status_acked
                or now - self._last_positioning_status_acked_at >= 1.0
            ):
                status_payload = {
                    "protocol": "payload-manual-position-status",
                    "version": 1,
                    "type": "status",
                    "status": status_key[0],
                    "range_count": status_key[1],
                    "rejected_ranges": status_key[2],
                }
                self._queue_positioning_message(
                    status_payload, kind="status", status=status_key
                )
            return

        # Do not resend a position until the previous one was acknowledged.
        # A newer estimate supersedes every intermediate estimate received
        # while that ACK was pending.
        if (
            self._last_positioning_sequence_acked is not None
            and estimate.sequence <= self._last_positioning_sequence_acked
        ):
            return
        if now - self._last_positioning_acked_at < 0.1:
            return

        payload = {
            "protocol": "payload-manual-position",
            "version": 1,
            "type": "position",
            "sequence": estimate.sequence,
            "position_m": {
                "x": round(snapshot.payload_position.x, 4),
                "y": round(snapshot.payload_position.y, 4),
            },
            "tag_position_m": {
                "x": round(estimate.position.x, 4),
                "y": round(estimate.position.y, 4),
            },
            "residual_m": round(estimate.residual_m, 4) if estimate.residual_m is not None else None,
            "covariance_xy_m2": (
                [round(value, 6) for value in estimate.covariance_xy_m2]
                if estimate.covariance_xy_m2 is not None else None
            ),
            "device_monotonic_us": estimate.device_monotonic_us,
            "heading_rad": round(snapshot.heading.angle_rad, 4),
            "heading_confidence": round(snapshot.heading.confidence, 3),
            "heading_observations": snapshot.heading.observations,
            "range_count": int(getattr(snapshot, "range_count", len(estimate.anchor_ids))),
            "rejected_ranges": int(getattr(snapshot, "rejected_range_count", 0)),
            "quality": estimate.quality,
        }
        self._queue_positioning_message(
            payload, kind="position", sequence=estimate.sequence
        )

    def _queue_positioning_message(
        self,
        payload: dict[str, Any],
        *,
        kind: str,
        sequence: int | None = None,
        status: tuple[str, int, int] | None = None,
    ) -> None:
        """Queue one positioning message and retain its ACK as a backpressure gate."""
        try:
            ticket = self._node.comms.send(self.controller_peer_id, payload)
        except Exception as exc:
            self._positioning_delivery_failures += 1
            delay = min(5.0, 0.25 * (2 ** min(self._positioning_delivery_failures - 1, 5)))
            self._positioning_retry_after = time.monotonic() + delay
            logger.warning(
                "could not queue manual positioning {} message; retrying in {:.2f}s: {}",
                kind,
                delay,
                exc,
            )
            return
        self._positioning_ticket = ticket
        self._positioning_ticket_kind = kind
        self._positioning_ticket_sequence = sequence
        self._positioning_ticket_status = status
        self._positioning_ticket_sent_at = time.monotonic()

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
        received_at = time.monotonic()
        try:
            loop.call_soon_threadsafe(self._enqueue, command, received_at)
        except RuntimeError:
            logger.debug("event loop closed while receiving manual-control message")

    def _enqueue(self, command: ManualControlCommand, received_at: float | None = None) -> None:
        if self._available is None or self._closed:
            return
        if received_at is not None:
            pass

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
                logger.warning(
                    "[manual-command-queue] mailbox full; dropping command type={} sequence={} "
                    "pending_depth={} dropped_total={}",
                    command.command_type,
                    command.sequence,
                    len(self._pending),
                    self.dropped_commands,
                )
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
