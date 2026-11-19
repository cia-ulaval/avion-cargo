from __future__ import annotations

import asyncio
import logging
import queue
import threading
from collections.abc import AsyncIterator, Mapping
from typing import Any

from d3kble import D3KNode

from payload.domain.positioning.distance_sensor import DistanceSensor
from payload.domain.positioning.events import DistanceEvent, DistanceFailure, DistanceSample
from shared.contracts.messages import FailureReason
from d3kble import D3KNode

logger = logging.getLogger(__name__)


class D3kbleDistanceSensor(DistanceSensor):
    """
    Adapt d3kble's synchronous measurement subscriptions to async events.
    """

    def __init__(
        self,
        node: D3KNode,
        anchor_id_by_target_id: Mapping[int, str],
        *,
        subscription_size: int = 256,
        event_queue_size: int = 256,
        poll_interval: float = 0.1,
        event_overflow: str = "drop_oldest",
    ):
        if not anchor_id_by_target_id:
            raise ValueError("at least one UWB target-to-anchor mapping is required")
        if any(isinstance(target, bool) or not isinstance(target, int) or not 0 <= target <= 65535
               for target in anchor_id_by_target_id):
            raise ValueError("UWB target IDs must be integers in 0..65535")
        if any(not isinstance(anchor, str) or not anchor for anchor in anchor_id_by_target_id.values()):
            raise ValueError("anchor IDs must be non-empty strings")
        if len(set(anchor_id_by_target_id.values())) != len(anchor_id_by_target_id):
            raise ValueError("each UWB target must map to a distinct application anchor ID")
        if subscription_size < 1 or event_queue_size < 1:
            raise ValueError("queue sizes must be positive")
        if poll_interval <= 0:
            raise ValueError("poll_interval must be greater than zero")
        if event_overflow not in {"drop_oldest", "drop_newest"}:
            raise ValueError("event_overflow must be 'drop_oldest' or 'drop_newest'")

        self._node = node
        self._anchor_by_target = dict(anchor_id_by_target_id)
        self._subscription_size = subscription_size
        self._event_queue_size = event_queue_size
        self._poll_interval = poll_interval
        self._event_overflow = event_overflow
        self.dropped_events = 0

    async def start_ranging(self) -> None:
        """Start the device's configured ranging schedule off the event loop."""
        await asyncio.to_thread(self._node.uwb.start_ranging)

    async def stop_ranging(self) -> None:
        """Stop the device's ranging schedule off the event loop."""
        await asyncio.to_thread(self._node.uwb.stop_ranging)

    async def events(self) -> AsyncIterator[DistanceEvent]:
        loop = asyncio.get_running_loop()
        event_queue: asyncio.Queue[DistanceEvent] = asyncio.Queue(maxsize=self._event_queue_size)
        stop_worker = threading.Event()
        subscription = self._node.uwb.measurements.subscribe(
            maxsize=self._subscription_size,
            overflow="drop_oldest",
        )

        def offer(event: DistanceEvent) -> None:
            if event_queue.full():
                self.dropped_events += 1
                if self._event_overflow == "drop_newest":
                    return
                try:
                    event_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            event_queue.put_nowait(event)

        def range_failed(failure: Any) -> None:
            if stop_worker.is_set():
                return
            anchor_id = self._anchor_by_target.get(int(failure.target_id))
            if anchor_id is None:
                return
            reason = FailureReason(
                code=f"uwb_range_failed_{failure.status}",
                message=f"UWB ranging failed with firmware status {failure.status}",
            )
            try:
                loop.call_soon_threadsafe(offer, DistanceFailure(anchor_id=anchor_id, reason=reason))
            except RuntimeError:
                logger.debug("event loop closed while receiving a UWB range failure")

        def pump_measurements() -> None:
            observed_source_drops = 0
            while not stop_worker.is_set():
                try:
                    measurement = subscription.get(timeout=self._poll_interval)
                except queue.Empty:
                    continue
                except RuntimeError:
                    return

                source_drops = subscription.dropped_count
                if source_drops > observed_source_drops:
                    self.dropped_events += source_drops - observed_source_drops
                    observed_source_drops = source_drops

                anchor_id = self._anchor_by_target.get(int(measurement.target_id))
                if anchor_id is None:
                    continue
                if measurement.valid and measurement.distance_m is not None:
                    event: DistanceEvent = DistanceSample(
                        anchor_id=anchor_id,
                        distance_m=float(measurement.distance_m),
                        sequence=int(measurement.sample_sequence),
                    )
                else:
                    event = DistanceFailure(
                        anchor_id=anchor_id,
                        reason=FailureReason(
                            code=f"uwb_measurement_status_{measurement.status}",
                            message=(
                                "UWB measurement was invalid "
                                f"(status={measurement.status}, quality_flags={measurement.quality_flags})"
                            ),
                        ),
                    )
                try:
                    loop.call_soon_threadsafe(offer, event)
                except RuntimeError:
                    return

        self._node.on("range_failed", range_failed)
        worker = asyncio.create_task(asyncio.to_thread(pump_measurements))
        try:
            while True:
                yield await event_queue.get()
        finally:
            stop_worker.set()
            self._node.off("range_failed", range_failed)
            self._node.uwb.measurements.unsubscribe(subscription)
            try:
                await asyncio.shield(worker)
            except asyncio.CancelledError:
                await asyncio.shield(worker)
                raise
