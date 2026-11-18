from __future__ import annotations

from collections.abc import Awaitable, Callable
from typing import Any, Protocol


class CommsResponseLike(Protocol):
    status: str
    error: str | None
    content: Any


class RequestTicketLike(Protocol):
    def __await__(self): ...


class SendTicketLike(Protocol):
    def wait_ack(self, timeout: float | None = None) -> Any: ...


class IncomingRequestLike(Protocol):
    peer_id: int
    payload: Any
    message_id: str

    def fail(self, reason: str, content: Any = None) -> Any: ...


MessageHandler = Callable[[Any], None]
RequestHandler = Callable[[IncomingRequestLike], Awaitable[Any] | Any]


class CommsManagerLike(Protocol):
    def bind_asyncio_loop(self, loop: Any) -> None: ...

    def on_message(self, callback: MessageHandler | None = None): ...

    def on_request(self, callback: RequestHandler | None = None): ...

    def send(self, peer_id: int, payload: Any) -> SendTicketLike: ...

    def request(self, peer_id: int, payload: Any) -> RequestTicketLike: ...


class MeasurementSubscriptionLike(Protocol):
    dropped_count: int

    def get(self, timeout: float | None = None) -> Any: ...


class MeasurementStreamLike(Protocol):
    def subscribe(self, maxsize: int = 256, overflow: str = "drop_oldest") -> MeasurementSubscriptionLike: ...

    def unsubscribe(self, subscription: MeasurementSubscriptionLike) -> None: ...


class UwbManagerLike(Protocol):
    measurements: MeasurementStreamLike

    def start_ranging(self) -> None: ...

    def stop_ranging(self) -> None: ...


class D3KNodeLike(Protocol):
    comms: CommsManagerLike
    uwb: UwbManagerLike

    def on(self, event: str, callback: Callable[[Any], None]): ...

    def off(self, event: str, callback: Callable[[Any], None]) -> None: ...
