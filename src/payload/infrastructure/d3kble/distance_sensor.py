from __future__ import annotations

import asyncio
import queue
import threading
import time
from collections import Counter
from collections.abc import AsyncIterator, Mapping
from typing import Any

from d3kble import D3KNode
from loguru import logger

from payload.domain.positioning.distance_sensor import DistanceSensor
from payload.domain.positioning.events import DistanceEvent, DistanceFailure, DistanceSample
from shared.contracts.messages import FailureReason


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
        if any(
            isinstance(target, bool) or not isinstance(target, int) or not 0 <= target <= 65535
            for target in anchor_id_by_target_id
        ):
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
        # Keep stage timestamps with each event so we can separate callback
        # scheduling delay from time spent waiting in the asyncio queue.
        event_queue: asyncio.Queue[tuple[DistanceEvent, float, float]] = asyncio.Queue(maxsize=self._event_queue_size)
        stop_worker = threading.Event()
        subscription = self._node.uwb.measurements.subscribe(
            maxsize=self._subscription_size,
            overflow="drop_oldest",
        )

        def offer(event: DistanceEvent, worker_received_at: float) -> None:
            offered_at = time.monotonic()
            if event_queue.full():
                self.dropped_events += 1
                if self._event_overflow == "drop_newest":
                    return
                try:
                    event_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            event_queue.put_nowait((event, worker_received_at, offered_at))

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
                loop.call_soon_threadsafe(
                    offer,
                    DistanceFailure(
                        anchor_id=anchor_id,
                        reason=reason,
                        device_monotonic_us=getattr(failure, "device_monotonic_us", None),
                        epoch_us=getattr(failure, "epoch_us", None),
                        received_at=getattr(failure, "received_at", None),
                        status=getattr(failure, "status", None),
                        quality_flags=getattr(failure, "quality_flags", None),
                    ),
                    time.monotonic(),
                )
            except RuntimeError:
                logger.debug("event loop closed while receiving a UWB range failure")

        def pump_measurements() -> None:
            observed_source_drops = 0
            counts: Counter[str] = Counter()
            invalid_counts: Counter[str] = Counter()
            last_sequence: dict[str, int] = {}
            last_device_us: dict[str, int] = {}
            last_summary_at = time.monotonic()
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
                counts[anchor_id] += 1
                sequence = int(measurement.sample_sequence)
                last_sequence[anchor_id] = sequence
                device_us = getattr(measurement, "device_monotonic_us", None)
                if device_us is not None:
                    last_device_us[anchor_id] = int(device_us)
                if measurement.valid and measurement.distance_m is not None:
                    event: DistanceEvent = DistanceSample(
                        anchor_id=anchor_id,
                        distance_m=float(measurement.distance_m),
                        sequence=int(measurement.sample_sequence),
                        device_monotonic_us=getattr(measurement, "device_monotonic_us", None),
                        epoch_us=getattr(measurement, "epoch_us", None),
                        received_at=getattr(measurement, "received_at", None),
                        status=getattr(measurement, "status", None),
                        quality_flags=getattr(measurement, "quality_flags", None),
                    )
                else:
                    invalid_counts[anchor_id] += 1
                    event = DistanceFailure(
                        anchor_id=anchor_id,
                        device_monotonic_us=getattr(measurement, "device_monotonic_us", None),
                        epoch_us=getattr(measurement, "epoch_us", None),
                        received_at=getattr(measurement, "received_at", None),
                        status=getattr(measurement, "status", None),
                        quality_flags=getattr(measurement, "quality_flags", None),
                        reason=FailureReason(
                            code=f"uwb_measurement_status_{measurement.status}",
                            message=(
                                "UWB measurement was invalid "
                                f"(status={measurement.status}, quality_flags={measurement.quality_flags})"
                            ),
                        ),
                    )
                worker_received_at = time.monotonic()
                try:
                    loop.call_soon_threadsafe(offer, event, worker_received_at)
                except RuntimeError:
                    return

                stats_now = time.monotonic()
                if stats_now - last_summary_at >= 1.0:
                    counts.clear()
                    invalid_counts.clear()
                    last_summary_at = stats_now

        self._node.on("range_failed", range_failed)
        worker = asyncio.create_task(asyncio.to_thread(pump_measurements))
        try:
            while True:
                event, _, _ = await event_queue.get()
                yield event
        finally:
            stop_worker.set()
            self._node.off("range_failed", range_failed)
            self._node.uwb.measurements.unsubscribe(subscription)
            try:
                await asyncio.shield(worker)
            except asyncio.CancelledError:
                await asyncio.shield(worker)
                raise
