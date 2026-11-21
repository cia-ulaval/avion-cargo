import queue
from dataclasses import dataclass
from types import SimpleNamespace

from shared.contracts.messages import CommandReceipt, PayloadStatusReport
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus
from shared.infrastructure.d3kble.codec import (
    decode_message,
    encode_command_receipt,
    encode_status_report,
)


@dataclass
class FakeCommsResponse:
    status: str
    error: str | None
    content: object


class FakeRequestTicket:
    def __init__(self, response):
        self.response = response

    def __await__(self):
        async def wait():
            return self.response

        return wait().__await__()


class FakeSendTicket:
    def __init__(self, peer_id):
        self.peer_id = peer_id

    def wait_ack(self, timeout=None):
        return {"accepted": True, "peer_id": self.peer_id}


class FakeComms:
    def __init__(self):
        self.message_handlers = []
        self.request_handlers = []
        self.loop = None
        self.sent = []
        self.requests = []
        self.status_report = PayloadStatusReport(
            state=PayloadState(PayloadStatus.UNKNOWN, DockingLatchState.UNKNOWN),
            position=None,
            sequence=0,
        )

    def bind_asyncio_loop(self, loop):
        self.loop = loop

    def on_message(self, callback=None):
        self.message_handlers.append(callback)
        return callback

    def on_request(self, callback=None):
        self.request_handlers.append(callback)
        return callback

    def send(self, peer_id, payload):
        self.sent.append((peer_id, payload))
        return FakeSendTicket(peer_id)

    def request(self, peer_id, payload):
        self.requests.append((peer_id, payload))
        message_type, data = decode_message(payload)
        if message_type in {"plane.command.dock", "plane.command.undock"}:
            content = encode_command_receipt(CommandReceipt(data["command_id"], accepted=True))
        elif message_type == "payload.request_status":
            content = encode_status_report(self.status_report)
        else:
            return FakeRequestTicket(FakeCommsResponse("error", "unsupported fake request", None))
        return FakeRequestTicket(FakeCommsResponse("ok", None, content))


class FakeSubscription:
    def __init__(self, maxsize=256):
        self.queue = queue.Queue(maxsize=maxsize)
        self.dropped_count = 0
        self.closed = False

    def get(self, timeout=None):
        if self.closed and self.queue.empty():
            raise RuntimeError("subscription closed")
        return self.queue.get(timeout=timeout)

    def publish(self, value):
        try:
            self.queue.put_nowait(value)
        except queue.Full:
            self.dropped_count += 1
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            self.queue.put_nowait(value)

    def close(self):
        self.closed = True


class FakeMeasurementStream:
    def __init__(self):
        self.subscriptions = []

    def subscribe(self, maxsize=256, overflow="drop_oldest"):
        subscription = FakeSubscription(maxsize)
        self.subscriptions.append(subscription)
        return subscription

    def unsubscribe(self, subscription):
        subscription.close()


class FakeUwb:
    def __init__(self):
        self.measurements = FakeMeasurementStream()
        self.started = 0
        self.stopped = 0

    def start_ranging(self):
        self.started += 1

    def stop_ranging(self):
        self.stopped += 1


class FakeNode:
    def __init__(self):
        self.comms = FakeComms()
        self.uwb = FakeUwb()
        self.callbacks = {}

    def on(self, event, callback):
        self.callbacks.setdefault(event, []).append(callback)
        return callback

    def off(self, event, callback):
        self.callbacks.get(event, []).remove(callback)

    def emit(self, event, value):
        for callback in tuple(self.callbacks.get(event, ())):
            callback(value)


class FakeIncomingRequest:
    def __init__(self, peer_id, payload):
        self.peer_id = peer_id
        self.payload = payload
        self.message_id = "fake-message"
        self.failure = None

    def fail(self, reason, content=None):
        self.failure = (reason, content)


def fake_measurement(
    target_id=200,
    distance_m=1.25,
    sample_sequence=7,
    status=0,
    quality_flags=0,
):
    return SimpleNamespace(
        target_id=target_id,
        distance_m=distance_m,
        sample_sequence=sample_sequence,
        status=status,
        quality_flags=quality_flags,
        valid=distance_m is not None and status == 0,
    )
