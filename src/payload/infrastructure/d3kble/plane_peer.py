from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator
from typing import Any

from payload.domain.docking.plane_peer import PlanePeer
from shared.contracts.geometry import PositionEstimate
from shared.contracts.messages import (
    CommandReceipt,
    DockRequest,
    FailureReason,
    PayloadOperationEvent,
    PayloadPositionUpdated,
    PayloadStateUpdated,
    PayloadStatusReport,
    PayloadStatusRequested,
    PlaneEvent,
    UndockRequest,
)
from shared.contracts.payload_state import PayloadState
from shared.infrastructure.d3kble.codec import (
    decode_command_request,
    decode_message,
    decode_plane_event,
    encode_command_receipt,
    encode_payload_event,
    encode_status_report,
)
from shared.infrastructure.d3kble.errors import D3kbleCommunicationError
from shared.infrastructure.d3kble.router import D3kbleMessageRouter, D3kbleRouteRegistration
from d3kble import D3KNode, IncomingRequest
from loguru import logger

_EVENTS_CLOSED = object()


class D3kblePlanePeer(PlanePeer):
    """
    Receive plane telemetry/commands and report payload telemetry/results using the d3kble api.
    """

    def __init__(
        self,
        node: D3KNode,
        router: D3kbleMessageRouter,
        *,
        plane_peer_id: int,
        command_ack_timeout: float = 0.6,
        status_response_timeout: float = 0.6,
        send_timeout: float = 2.0,
    ):
        if (
            isinstance(plane_peer_id, bool)
            or not isinstance(plane_peer_id, int)
            or not 1 <= plane_peer_id <= 65534
        ):
            raise ValueError("plane_peer_id must be an integer in 1..65534")
        if command_ack_timeout <= 0:
            raise ValueError("command_ack_timeout must be greater than zero")
        if status_response_timeout <= 0:
            raise ValueError("status_response_timeout must be greater than zero")
        if send_timeout <= 0:
            raise ValueError("send_timeout must be greater than zero")

        self._node = node
        self._router = router
        self._plane_peer_id = plane_peer_id
        self._command_ack_timeout = command_ack_timeout
        self._status_response_timeout = status_response_timeout
        self._send_timeout = send_timeout
        self._events: asyncio.Queue[PlaneEvent] = asyncio.Queue()
        self._pending_commands: dict[str, asyncio.Future[CommandReceipt]] = {}
        self._pending_status_requests: dict[str, asyncio.Future[PayloadStatusReport]] = {}
        self._loop: asyncio.AbstractEventLoop | None = None
        self._registration: D3kbleRouteRegistration | None = None
        self._events_active = False
        self._closed = False

    @property
    def plane_ble_peer_id(self) -> int:
        return self._plane_peer_id

    async def start(self) -> None:
        if self._closed:
            raise RuntimeError("peer adapter is closed")
        if self._registration is not None:
            self._assert_loop()
            return
        await self._router.start()
        self._loop = asyncio.get_running_loop()
        self._registration = self._router.register_peer(
            self._plane_peer_id,
            message_handler=self._on_message,
            request_handler=self._on_request,
        )

    async def events(self) -> AsyncIterator[PlaneEvent]:
        await self.start()
        if self._events_active:
            raise RuntimeError("D3kblePlanePeer supports one active events() consumer")
        self._events_active = True
        try:
            while True:
                event = await self._events.get()
                if event is _EVENTS_CLOSED:
                    return
                if isinstance(event, (DockRequest, UndockRequest)):
                    pending = self._pending_commands.get(event.command_id)
                    if pending is None or pending.done():
                        continue
                elif isinstance(event, PayloadStatusRequested):
                    pending = self._pending_status_requests.get(event.request_id)
                    if pending is None or pending.done():
                        continue
                yield event
        finally:
            self._events_active = False

    async def report_position(self, estimate: PositionEstimate) -> None:
        await self.start()
        message = encode_payload_event(PayloadPositionUpdated(estimate=estimate))
        await self._send(message, "position report")

    async def report_state(self, state: PayloadState, sequence: int) -> None:
        await self.start()
        message = encode_payload_event(PayloadStateUpdated(state=state, sequence=sequence))
        await self._send(message, "payload state report")

    async def acknowledge_command(self, receipt: CommandReceipt) -> None:
        await self.start()
        future = self._pending_commands.get(receipt.command_id)
        if future is None:
            raise KeyError(f"no pending command with ID {receipt.command_id!r}")
        if future.done():
            raise RuntimeError(f"command {receipt.command_id!r} has already been acknowledged or expired")
        if receipt.accepted and receipt.rejection_reason is not None:
            raise ValueError("an accepted command cannot have a rejection reason")
        future.set_result(receipt)

    async def respond_status_request(
        self, request_id: str, report: PayloadStatusReport
    ) -> None:
        await self.start()
        future = self._pending_status_requests.get(request_id)
        if future is None:
            raise KeyError(f"no pending status request with ID {request_id!r}")
        if future.done():
            raise RuntimeError(f"status request {request_id!r} has already been answered or expired")
        encode_status_report(report)  # Validate the response size before completing the request.
        future.set_result(report)

    async def report_operation_event(self, event: PayloadOperationEvent) -> None:
        await self.start()
        await self._send(encode_payload_event(event), "payload operation report")

    async def _on_request(self, request: IncomingRequest) -> dict[str, Any]:
        message_type, _data = decode_message(request.payload)
        if message_type == "payload.request_status":
            report = await self._await_status_response(request.message_id)
            return encode_status_report(report)

        command = decode_command_request(request.payload)
        if not isinstance(command, (DockRequest, UndockRequest)):
            raise ValueError(f"unsupported command {type(command).__name__}")
        receipt = await self._await_command_decision(command)
        return encode_command_receipt(receipt)

    async def _await_command_decision(self, command: DockRequest | UndockRequest) -> CommandReceipt:
        assert self._loop is asyncio.get_running_loop()
        existing = self._pending_commands.get(command.command_id)
        if existing is not None:
            future = existing
        else:
            future = self._loop.create_future()
            self._pending_commands[command.command_id] = future
            self._events.put_nowait(command)

        try:
            return await asyncio.wait_for(
                asyncio.shield(future),
                timeout=self._command_ack_timeout,
            )
        except asyncio.TimeoutError:
            return CommandReceipt(
                command_id=command.command_id,
                accepted=False,
                rejection_reason=FailureReason(
                    code="application_ack_timeout",
                    message="payload application did not acknowledge the command before the deadline",
                ),
            )
        finally:
            if self._pending_commands.get(command.command_id) is future:
                self._pending_commands.pop(command.command_id, None)

    async def _await_status_response(self, request_id: str) -> PayloadStatusReport:
        assert self._loop is asyncio.get_running_loop()
        future = self._loop.create_future()
        self._pending_status_requests[request_id] = future
        self._events.put_nowait(PayloadStatusRequested(request_id=request_id))
        try:
            return await asyncio.wait_for(
                asyncio.shield(future),
                timeout=self._status_response_timeout,
            )
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                "payload application did not answer the status request before the deadline"
            ) from exc
        finally:
            if self._pending_status_requests.get(request_id) is future:
                self._pending_status_requests.pop(request_id, None)

    def _on_message(self, message: Any) -> None:
        try:
            event = decode_plane_event(message.payload)
        except Exception:
            logger.exception("discarding invalid plane message from BLE peer %s", self._plane_peer_id)
            return
        loop = self._loop
        if loop is not None and not loop.is_closed() and not self._closed:
            try:
                loop.call_soon_threadsafe(self._enqueue_event, event)
            except RuntimeError:
                logger.debug("event loop closed while receiving plane event from peer %s", self._plane_peer_id)

    def _enqueue_event(self, event: PlaneEvent) -> None:
        if not self._closed:
            self._events.put_nowait(event)

    async def _send(self, message: dict[str, Any], operation: str) -> None:
        try:
            ticket = self._node.comms.send(self._plane_peer_id, message)
            await asyncio.to_thread(ticket.wait_ack, self._send_timeout)
        except Exception as exc:
            raise D3kbleCommunicationError(
                f"could not deliver {operation} to plane BLE peer {self._plane_peer_id}: {exc}"
            ) from exc

    def _assert_loop(self) -> None:
        if self._loop is not asyncio.get_running_loop():
            raise RuntimeError("D3kblePlanePeer must be used from its startup event loop")

    def close(self) -> None:
        if self._registration is not None:
            self._registration.close()
            self._registration = None
        self._closed = True

        def finish_close() -> None:
            for command_id, future in tuple(self._pending_commands.items()):
                if not future.done():
                    future.set_result(
                        CommandReceipt(
                            command_id=command_id,
                            accepted=False,
                            rejection_reason=FailureReason(
                                code="adapter_closed",
                                message="payload communication adapter closed before acknowledgement",
                            ),
                        )
                    )
            self._pending_commands.clear()
            for request_id, future in tuple(self._pending_status_requests.items()):
                if not future.done():
                    future.set_exception(
                        RuntimeError("D3kblePlanePeer closed before status response")
                    )
            self._pending_status_requests.clear()
            while not self._events.empty():
                try:
                    self._events.get_nowait()
                except asyncio.QueueEmpty:
                    break
            self._events.put_nowait(_EVENTS_CLOSED)

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
