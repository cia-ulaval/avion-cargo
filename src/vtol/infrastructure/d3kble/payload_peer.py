from __future__ import annotations

import asyncio
import logging
import uuid
from collections.abc import AsyncIterator
from typing import Any

from shared.contracts.geometry import Heading, Position
from shared.contracts.messages import (
    CommandReceipt,
    DLZState,
    PayloadEvent,
    PayloadStatusReport,
    PlaneHeadingUpdated,
    DLZStateUpdated,
)
from shared.infrastructure.d3kble.codec import (
    decode_command_receipt,
    decode_payload_event,
    decode_status_report,
    encode_dock_request,
    encode_plane_event,
    encode_status_request,
    encode_undock_request,
)
from shared.infrastructure.d3kble.errors import D3kbleCommunicationError
from shared.infrastructure.d3kble.router import D3kbleMessageRouter, D3kbleRouteRegistration
from shared.infrastructure.d3kble.types import D3KNodeLike
from vtol.domain.payload_peer import PayloadPeer

logger = logging.getLogger(__name__)
_EVENTS_CLOSED = object()


class D3kblePayloadPeer(PayloadPeer):
    """
    Communicate with one payload over the local node's BLE application link.
    """

    def __init__(
        self,
        node: D3KNodeLike,
        router: D3kbleMessageRouter,
        *,
        payload_id: str,
        peer_id: int,
        send_timeout: float = 2.0,
    ):
        if not isinstance(payload_id, str) or not payload_id:
            raise ValueError("payload_id must be a non-empty string")
        
        if isinstance(peer_id, bool) or not isinstance(peer_id, int) or not 1 <= peer_id <= 65534:
            raise ValueError("peer_id must be an integer in 1..65534")
        
        if send_timeout <= 0:
            raise ValueError("send_timeout must be greater than zero")

        self._node = node
        self._router = router
        self._payload_id = payload_id
        self._peer_id = peer_id
        self._send_timeout = send_timeout
        self._events: asyncio.Queue[PayloadEvent] = asyncio.Queue()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._registration: D3kbleRouteRegistration | None = None
        self._events_active = False
        self._closed = False

    @property
    def payload_id(self) -> str:
        return self._payload_id

    @property
    def ble_peer_id(self) -> int:
        return self._peer_id

    async def start(self) -> None:
        if self._closed:
            raise RuntimeError("peer adapter is closed")
        
        if self._registration is not None:
            self._assert_loop()
            return
        
        await self._router.start()
        self._loop = asyncio.get_running_loop()
        self._registration = self._router.register_peer(self._peer_id, message_handler=self._on_message)

    async def events(self) -> AsyncIterator[PayloadEvent]:
        await self.start()

        if self._events_active:
            raise RuntimeError("D3kblePayloadPeer supports one active events() consumer")
        
        self._events_active = True

        try:
            while True:
                event = await self._events.get()
                if event is _EVENTS_CLOSED:
                    return
                
                yield event
        finally:
            self._events_active = False

    async def request_undock(self, target_position: Position) -> CommandReceipt:
        await self.start()
        command_id = uuid.uuid4().hex
        response = await self._request(encode_undock_request(command_id, target_position), "undock")
        receipt = decode_command_receipt(response)
        self._validate_receipt(receipt, command_id)
        return receipt

    async def request_dock(self) -> CommandReceipt:
        await self.start()
        command_id = uuid.uuid4().hex
        response = await self._request(encode_dock_request(command_id), "dock")
        receipt = decode_command_receipt(response)
        self._validate_receipt(receipt, command_id)
        return receipt

    async def request_status(self) -> PayloadStatusReport:
        await self.start()
        response = await self._request(encode_status_request(), "status")
        return decode_status_report(response)

    async def publish_heading(self, heading: Heading, sequence: int) -> None:
        await self.start()
        message = encode_plane_event(PlaneHeadingUpdated(heading=heading, sequence=sequence))
        await self._send(message, "heading update")

    async def publish_dlz_state(self, state: DLZState) -> None:
        await self.start()
        message = encode_plane_event(DLZStateUpdated(state=state))
        await self._send(message, "DLZ state update")

    async def _request(self, message: dict[str, Any], operation: str) -> Any:
        try:
            response = await self._node.comms.request(self._peer_id, message)
        except Exception as exc:
            error_message = f"{operation} request to payload {self._payload_id} failed: {exc}"
            raise D3kbleCommunicationError(error_message) from exc
        if response.status != "ok":
            error_message = f"{operation} request to payload {self._payload_id} returned an error response: {response.error}"
            raise D3kbleCommunicationError(error_message)
        return response.content

    async def _send(self, message: dict[str, Any], operation: str) -> None:
        try:
            ticket = self._node.comms.send(self._peer_id, message)
            await asyncio.to_thread(ticket.wait_ack, self._send_timeout)
        except Exception as exc:
            error_message = f"could not deliver {operation} to payload {self._payload_id}: {exc}"
            raise D3kbleCommunicationError(error_message) from exc

    def _on_message(self, message: Any) -> None:
        try:
            event = decode_payload_event(message.payload)
        except Exception:
            logger.exception("discarding invalid payload message from BLE peer %s", self._peer_id)
            return
        
        loop = self._loop

        if loop is not None and not loop.is_closed() and not self._closed:
            try:
                loop.call_soon_threadsafe(self._enqueue_event, event)
            except RuntimeError:
                logger.debug("event loop closed while receiving payload event from %s", self._payload_id)

    def _enqueue_event(self, event: PayloadEvent) -> None:
        if not self._closed:
            self._events.put_nowait(event)

    @staticmethod
    def _validate_receipt(receipt: CommandReceipt, expected_command_id: str) -> None:
        if receipt.command_id != expected_command_id:
            raise D3kbleCommunicationError("payload returned a command receipt for a different command ID")

    def _assert_loop(self) -> None:
        if self._loop is not asyncio.get_running_loop():
            raise RuntimeError("D3kblePayloadPeer must be used from its startup event loop")

    def close(self) -> None:
        if self._registration is not None:
            self._registration.close()
            self._registration = None
        
        self._closed = True

        def finish_close() -> None:
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
