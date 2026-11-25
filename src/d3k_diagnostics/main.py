from __future__ import annotations

import asyncio
import json
import math
import queue
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import click
from loguru import logger

from shared.infrastructure.d3kble.codec import decode_message, encode_message


@dataclass(frozen=True, slots=True)
class DeviceConfiguration:
    name: str
    uart_device: str
    baudrate: int = 460800
    ready_timeout_s: float = 10.0
    peer_id: int | None = None
    peer_ids: tuple[int, ...] = ()

    @property
    def selected_peer_ids(self) -> tuple[int, ...]:
        if self.peer_ids:
            return self.peer_ids
        return (self.peer_id,) if self.peer_id is not None else ()


@dataclass(frozen=True, slots=True)
class DiagnosticsConfiguration:
    devices: tuple[DeviceConfiguration, ...]
    poll_interval_s: float = 2.0

    @classmethod
    def from_file(cls, path: str | Path) -> DiagnosticsConfiguration:
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = json.load(stream)

        if not isinstance(raw, dict):
            raise ValueError("D3K diagnostics configuration must be a JSON object")
        raw_devices = raw.get("devices")
        if not isinstance(raw_devices, list) or not raw_devices:
            raise ValueError("devices must be a non-empty list")

        devices: list[DeviceConfiguration] = []
        for index, item in enumerate(raw_devices):
            prefix = f"devices[{index}]"
            if not isinstance(item, dict):
                raise ValueError(f"{prefix} must be an object")
            name = item.get("name")
            uart_device = item.get("uart_device")
            baudrate = item.get("baudrate", 460800)
            ready_timeout_s = item.get("ready_timeout_s", 10.0)
            peer_id = item.get("peer_id")
            raw_peer_ids = item.get("peer_ids")
            if peer_id is not None and raw_peer_ids is not None:
                raise ValueError(f"{prefix} must specify peer_id or peer_ids, not both")
            if raw_peer_ids is None:
                raw_peer_ids = []
            if not isinstance(raw_peer_ids, list):
                raise ValueError(f"{prefix}.peer_ids must be a list")
            if any(
                isinstance(peer, bool) or not isinstance(peer, int) or not 1 <= peer <= 65534 for peer in raw_peer_ids
            ):
                raise ValueError(f"{prefix}.peer_ids must contain integers in 1..65534")
            if len(raw_peer_ids) != len(set(raw_peer_ids)):
                raise ValueError(f"{prefix}.peer_ids must contain unique IDs")
            if not isinstance(name, str) or not name.strip():
                raise ValueError(f"{prefix}.name must be a non-empty string")
            if not isinstance(uart_device, str) or not uart_device.strip():
                raise ValueError(f"{prefix}.uart_device must be a non-empty string")
            if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
                raise ValueError(f"{prefix}.baudrate must be a positive integer")
            if (
                isinstance(ready_timeout_s, bool)
                or not isinstance(ready_timeout_s, (int, float))
                or not math.isfinite(ready_timeout_s)
                or ready_timeout_s <= 0
            ):
                raise ValueError(f"{prefix}.ready_timeout_s must be finite and positive")
            if peer_id is not None and (
                isinstance(peer_id, bool) or not isinstance(peer_id, int) or not 1 <= peer_id <= 65534
            ):
                raise ValueError(f"{prefix}.peer_id must be an integer in 1..65534")
            devices.append(
                DeviceConfiguration(
                    name=name,
                    uart_device=uart_device,
                    baudrate=baudrate,
                    ready_timeout_s=float(ready_timeout_s),
                    peer_id=peer_id,
                    peer_ids=tuple(raw_peer_ids),
                )
            )

        names = [device.name for device in devices]
        uart_devices = [device.uart_device for device in devices]
        if len(names) != len(set(names)):
            raise ValueError("device names must be unique")
        if len(uart_devices) != len(set(uart_devices)):
            raise ValueError("UART device paths must be unique")

        poll_interval_s = raw.get("poll_interval_s", 2.0)
        if (
            isinstance(poll_interval_s, bool)
            or not isinstance(poll_interval_s, (int, float))
            or not math.isfinite(poll_interval_s)
            or poll_interval_s <= 0
        ):
            raise ValueError("poll_interval_s must be finite and positive")

        return cls(tuple(devices), float(poll_interval_s))


@dataclass(frozen=True, slots=True)
class _RangingSummary:
    sample_count: int
    mean_distance_m: float | None
    stddev_distance_m: float | None
    min_distance_m: float | None
    max_distance_m: float | None
    failures_by_status: tuple[tuple[str, int], ...]
    invalid_measurements: int
    dropped_measurements: int

    @property
    def has_errors(self) -> bool:
        return bool(self.failures_by_status or self.invalid_measurements or self.dropped_measurements)


class _RangingStatistics:
    """Aggregate ranging results for one configured remote peer."""

    def __init__(self, peer_id: int | None) -> None:
        self.peer_id = peer_id
        self._lock = threading.Lock()
        self._sample_count = 0
        self._mean_distance_m = 0.0
        self._sum_squared_deviations = 0.0
        self._min_distance_m: float | None = None
        self._max_distance_m: float | None = None
        self._failures_by_status: dict[str, int] = {}
        self._invalid_measurements = 0
        self._dropped_measurements = 0

    def add_measurement(self, measurement: Any) -> None:
        if self.peer_id is None or int(measurement.target_id) != self.peer_id:
            return

        distance = measurement.distance_m
        try:
            distance_m = float(distance)
        except (TypeError, ValueError, OverflowError):
            distance_m = math.nan

        with self._lock:
            if not measurement.valid or not math.isfinite(distance_m):
                self._invalid_measurements += 1
                return

            self._sample_count += 1
            delta = distance_m - self._mean_distance_m
            self._mean_distance_m += delta / self._sample_count
            self._sum_squared_deviations += delta * (distance_m - self._mean_distance_m)
            if self._min_distance_m is None or distance_m < self._min_distance_m:
                self._min_distance_m = distance_m
            if self._max_distance_m is None or distance_m > self._max_distance_m:
                self._max_distance_m = distance_m

    def add_failure(self, failure: Any) -> None:
        if self.peer_id is None or int(failure.target_id) != self.peer_id:
            return
        status = str(failure.status)
        with self._lock:
            self._failures_by_status[status] = self._failures_by_status.get(status, 0) + 1

    def add_dropped_measurements(self, count: int) -> None:
        if count <= 0:
            return
        with self._lock:
            self._dropped_measurements += count

    def snapshot_and_reset(self) -> _RangingSummary:
        with self._lock:
            count = self._sample_count
            summary = _RangingSummary(
                sample_count=count,
                mean_distance_m=self._mean_distance_m if count else None,
                stddev_distance_m=(math.sqrt(self._sum_squared_deviations / count) if count else None),
                min_distance_m=self._min_distance_m,
                max_distance_m=self._max_distance_m,
                failures_by_status=tuple(sorted(self._failures_by_status.items())),
                invalid_measurements=self._invalid_measurements,
                dropped_measurements=self._dropped_measurements,
            )
            self._sample_count = 0
            self._mean_distance_m = 0.0
            self._sum_squared_deviations = 0.0
            self._min_distance_m = None
            self._max_distance_m = None
            self._failures_by_status.clear()
            self._invalid_measurements = 0
            self._dropped_measurements = 0
            return summary

    def log_summary(self, device_name: str, interval_s: float) -> None:
        if self.peer_id is None:
            return

        summary = self.snapshot_and_reset()

        def format_distance(value: float | None) -> str:
            return "n/a" if value is None else f"{value:.3f} m"

        failures = ", ".join(f"status {status}: {count}" for status, count in summary.failures_by_status) or "0"
        log = logger.warning if summary.has_errors else logger.info
        log(
            "D3K [{}] UWB peer {} over {:.1f}s: samples={}, mean={}, stddev={}, min={}, max={}, "
            "range_failures={}, invalid_measurements={}, dropped_measurements={}",
            device_name,
            self.peer_id,
            interval_s,
            summary.sample_count,
            format_distance(summary.mean_distance_m),
            format_distance(summary.stddev_distance_m),
            format_distance(summary.min_distance_m),
            format_distance(summary.max_distance_m),
            failures,
            summary.invalid_measurements,
            summary.dropped_measurements,
        )


def _attach_range_failure_collection(
    node: Any,
    statistics_by_peer: dict[int, _RangingStatistics],
) -> Any:
    def collect(failure: Any) -> None:
        try:
            peer_id = int(failure.target_id)
        except (AttributeError, TypeError, ValueError):
            return
        statistics = statistics_by_peer.get(peer_id)
        if statistics is not None:
            statistics.add_failure(failure)

    node.on("range_failed", collect)
    return collect


_EVENT_NAMES = (
    "connected",
    "ready",
    "disconnected",
    "connection_error",
    "history_overrun",
    "state_changed",
    "device_ready",
    "error",
    "ble_state_changed",
    "ble_peer_found",
    "ble_peer_connected",
    "ble_peer_disconnected",
    "ble_connection_failed",
    "ble_message",
    "ble_error",
    "comms_received",
    "comms_ack",
    "comms_response",
    "comms_processed",
    "comms_timeout",
    "comms_failed",
    "comms_error",
    "comms_handler_error",
    "reconciliation_complete",
    "reconciliation_failed",
    "reconciliation_error",
)

_ERROR_EVENTS = {
    "disconnected",
    "connection_error",
    "error",
    "ble_connection_failed",
    "ble_error",
    "comms_timeout",
    "comms_failed",
    "comms_error",
    "comms_handler_error",
    "reconciliation_failed",
    "reconciliation_error",
}


def _attach_event_logging(node: Any, device_name: str) -> None:
    for event_name in _EVENT_NAMES:

        def report(value: Any, name: str = event_name) -> None:
            log = logger.warning if name in _ERROR_EVENTS else logger.info
            log("D3K [{}] {}: {}", device_name, name, value)

        node.on(event_name, report)


def _handle_ping_request(request: Any) -> dict[str, Any]:
    """Validate a diagnostics PING request and return its correlated PONG body."""
    message_type, data = decode_message(request.payload)
    if message_type != "diagnostics.ping":
        raise ValueError(f"unexpected diagnostics request: {message_type}")

    sequence = data.get("sequence")
    sent_at_ns = data.get("sent_at_ns")
    if (
        isinstance(sequence, bool)
        or not isinstance(sequence, int)
        or sequence < 1
        or isinstance(sent_at_ns, bool)
        or not isinstance(sent_at_ns, int)
        or sent_at_ns <= 0
    ):
        raise ValueError("PING sequence or timestamp is invalid")

    return encode_message(
        "diagnostics.pong",
        {"sequence": sequence, "sent_at_ns": sent_at_ns},
    )


async def _exchange_ping(
    node: Any,
    peer_id: int,
    sequence: int,
    timeout_s: float,
) -> float:
    """Send one request and validate the PONG returned on its request ticket."""
    sent_at_ns = time.monotonic_ns()
    payload = encode_message(
        "diagnostics.ping",
        {"sequence": sequence, "sent_at_ns": sent_at_ns},
        kind="request",
    )
    ticket = node.comms.request(peer_id, payload)
    response = await asyncio.wait_for(ticket, timeout=timeout_s)

    if response.status != "ok":
        raise RuntimeError(response.error or f"peer {peer_id} returned an error response")

    message_type, data = decode_message(response.content)
    if message_type != "diagnostics.pong":
        raise ValueError(f"peer {peer_id} returned {message_type}, not diagnostics.pong")
    if data.get("sequence") != sequence:
        raise ValueError(f"peer {peer_id} returned a mismatched PONG sequence")
    if data.get("sent_at_ns") != sent_at_ns:
        raise ValueError(f"peer {peer_id} returned a mismatched PONG timestamp")

    return (time.monotonic_ns() - sent_at_ns) / 1_000_000


class _BleDiagnostics:
    """Own one router per local node and independently probe each configured link."""

    def __init__(
        self,
        sessions: list[tuple[DeviceConfiguration, Any, Any | None]],
        *,
        interval_s: float = 1.0,
        timeout_s: float = 1.5,
    ) -> None:
        if interval_s <= 0 or timeout_s <= 0:
            raise ValueError("BLE ping interval and timeout must be positive")
        self._sessions = sessions
        self._interval_s = interval_s
        self._timeout_s = timeout_s
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_async: asyncio.Event | None = None
        self._ready = threading.Event()
        self._startup_error: BaseException | None = None
        self._routers: list[Any] = []
        self._registrations: list[Any] = []
        self._tasks: set[asyncio.Task] = set()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("BLE diagnostics have already been started")
        self._thread = threading.Thread(
            target=self._run_loop,
            name="d3k-ble-diagnostics",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(timeout=5.0):
            self.stop()
            raise TimeoutError("BLE diagnostics event loop did not start")
        if self._startup_error is not None:
            raise RuntimeError("could not start BLE diagnostics") from self._startup_error

    def _run_loop(self) -> None:
        try:
            asyncio.run(self._serve())
        except BaseException as exc:
            if not self._ready.is_set():
                self._startup_error = exc
            logger.exception("D3K BLE diagnostics event loop stopped unexpectedly")
        finally:
            self._ready.set()

    @staticmethod
    def _device_id(node: Any, device_name: str) -> int:
        state = node.get_state(refresh=True)
        runtime = getattr(state, "runtime", None)
        values = getattr(runtime, "values", {})
        device_id = values.get("device_id") if hasattr(values, "get") else None
        if isinstance(device_id, bool) or not isinstance(device_id, int) or not 1 <= device_id <= 65534:
            raise ValueError(f"D3K [{device_name}] returned an invalid device_id: {device_id!r}")
        return device_id

    async def _serve(self) -> None:
        from shared.infrastructure.d3kble.router import D3kbleMessageRouter

        self._loop = asyncio.get_running_loop()
        self._stop_async = asyncio.Event()
        sessions_by_id: dict[int, tuple[DeviceConfiguration, Any, Any | None]] = {}
        device_ids: dict[str, int] = {}
        try:
            for session in self._sessions:
                device, node, _subscription = session
                device_id = self._device_id(node, device.name)
                if device_id in sessions_by_id:
                    raise ValueError(f"duplicate connected D3K device_id: {device_id}")
                device_ids[device.name] = device_id
                sessions_by_id[device_id] = session

            for device, node, _subscription in self._sessions:
                router = D3kbleMessageRouter(node)
                await router.start()
                self._routers.append(router)
                for peer_id in device.selected_peer_ids:
                    self._registrations.append(router.register_peer(peer_id, request_handler=_handle_ping_request))

            # Probe each physical link once, from the lower device ID. Both ends
            # must list the other endpoint so routing is configured in both directions.
            for device, node, _subscription in self._sessions:
                local_id = device_ids[device.name]
                for peer_id in device.selected_peer_ids:
                    remote_session = sessions_by_id.get(peer_id)
                    if remote_session is None or local_id >= peer_id:
                        continue
                    remote_device = remote_session[0]
                    if local_id not in remote_device.selected_peer_ids:
                        logger.warning(
                            "D3K BLE link is not reciprocal: {} (ID {}) -> peer {}",
                            device.name,
                            local_id,
                            peer_id,
                        )
                        continue
                    task = asyncio.create_task(self._run_peer_loop(node, peer_id, device.name))
                    self._tasks.add(task)
                    task.add_done_callback(self._tasks.discard)

            self._ready.set()
            if not self._tasks:
                logger.warning("No reciprocal in-process D3K BLE peer links found for PING/PONG")
            await self._stop_async.wait()
        finally:
            for task in tuple(self._tasks):
                task.cancel()
            if self._tasks:
                await asyncio.gather(*self._tasks, return_exceptions=True)
            for registration in self._registrations:
                registration.close()
            for router in self._routers:
                router.close()

    async def _run_peer_loop(self, node: Any, peer_id: int, device_name: str) -> None:
        sequence = 0
        while True:
            sequence += 1
            try:
                rtt_ms = await _exchange_ping(
                    node,
                    peer_id,
                    sequence,
                    self._timeout_s,
                )
                logger.info(
                    "D3K [{}] BLE ping/pong peer {} sequence {} RTT={:.1f}ms",
                    device_name,
                    peer_id,
                    sequence,
                    rtt_ms,
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                # A failed peer remains isolated to this task. The next peer
                # and the next scheduled attempt are not cancelled.
                logger.warning(
                    "D3K [{}] BLE exchange with peer {} failed: {}",
                    device_name,
                    peer_id,
                    exc,
                )
            await asyncio.sleep(self._interval_s)

    def stop(self) -> None:
        loop = self._loop
        stop_event = self._stop_async
        if loop is not None and loop.is_running() and stop_event is not None:
            try:
                loop.call_soon_threadsafe(stop_event.set)
            except RuntimeError:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)


def _start_ble_diagnostics(
    sessions: list[tuple[DeviceConfiguration, Any, Any | None]],
) -> _BleDiagnostics | None:
    if not any(device.selected_peer_ids for device, _node, _subscription in sessions):
        logger.info("No BLE peers configured; skipping BLE request diagnostics")
        return None
    diagnostics = _BleDiagnostics(sessions)
    try:
        diagnostics.start()
    except Exception:
        diagnostics.stop()
        logger.exception("Could not start multi-peer BLE diagnostics")
        return None
    return diagnostics


def _poll_device(node: Any, device: DeviceConfiguration) -> None:
    state = node.get_state(refresh=True)
    logger.info("D3K [{}] readiness: connected={}, ready={}", device.name, node.is_connected, node.is_ready)
    logger.info("D3K [{}] state/configuration: {}", device.name, state)
    logger.info("D3K [{}] host diagnostics: {}", device.name, node.get_diagnostics())

    for peer_id in device.selected_peer_ids:
        try:
            peer_status = node.ble.get_peer_status(peer_id)
            logger.info("D3K [{}] BLE peer {} status: {}", device.name, peer_id, peer_status)
        except Exception:
            logger.exception("D3K [{}] could not read BLE peer {} status", device.name, peer_id)


def run_diagnostics(config: DiagnosticsConfiguration, stop_event: threading.Event) -> None:
    try:
        from d3kble import D3KNode
    except ImportError as exc:
        raise RuntimeError("The d3kble package is required to run D3K diagnostics") from exc

    sessions: list[tuple[DeviceConfiguration, Any, Any | None]] = []
    dropped_counts: dict[str, int] = {}
    ranging_statistics: dict[str, dict[int, _RangingStatistics]] = {}
    range_failure_handlers: dict[str, Any] = {}
    ble_diagnostics: _BleDiagnostics | None = None
    try:
        for device in config.devices:
            logger.info("Connecting D3K [{}] through {}", device.name, device.uart_device)
            node = D3KNode(device.uart_device, baudrate=device.baudrate)
            sessions.append((device, node, None))
            _attach_event_logging(node, device.name)
            statistics_by_peer = {peer_id: _RangingStatistics(peer_id) for peer_id in device.selected_peer_ids}
            ranging_statistics[device.name] = statistics_by_peer
            range_failure_handlers[device.name] = _attach_range_failure_collection(node, statistics_by_peer)
            subscription = node.uwb.measurements.subscribe(maxsize=256, overflow="drop_oldest")
            sessions[-1] = (device, node, subscription)
            dropped_counts[device.name] = 0
            node.start()
            node.wait_until_ready(timeout=device.ready_timeout_s)
            logger.info("D3K [{}] ready; initial state: {}", device.name, node.get_state(refresh=True))

        ble_diagnostics = _start_ble_diagnostics(sessions)

        next_poll = 0.0
        next_ranging_summary = time.monotonic() + config.poll_interval_s
        last_ranging_summary = time.monotonic()
        while not stop_event.is_set():
            for device, node, subscription in sessions:
                while True:
                    try:
                        measurement = subscription.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        target_id = int(measurement.target_id)
                    except (AttributeError, TypeError, ValueError):
                        continue
                    statistics = ranging_statistics[device.name].get(target_id)
                    if statistics is not None:
                        statistics.add_measurement(measurement)
                dropped = subscription.dropped_count
                if dropped > dropped_counts[device.name]:
                    delta = dropped - dropped_counts[device.name]
                    logger.warning(
                        "D3K [{}] measurement queue dropped {} item(s); target attribution is unavailable",
                        device.name,
                        delta,
                    )
                    dropped_counts[device.name] = dropped

            now = time.monotonic()
            if now >= next_ranging_summary:
                interval_s = now - last_ranging_summary
                for device, _node, _subscription in sessions:
                    for statistics in ranging_statistics[device.name].values():
                        statistics.log_summary(device.name, interval_s)
                last_ranging_summary = now
                next_ranging_summary = now + config.poll_interval_s

            if now >= next_poll:
                for device, node, _subscription in sessions:
                    try:
                        _poll_device(node, device)
                    except Exception:
                        logger.exception("D3K [{}] diagnostics poll failed", device.name)
                next_poll = now + config.poll_interval_s
            stop_event.wait(0.1)
    finally:
        if ble_diagnostics is not None:
            ble_diagnostics.stop()

        for device, node, subscription in reversed(sessions):
            handler = range_failure_handlers.get(device.name)
            if handler is not None:
                try:
                    node.off("range_failed", handler)
                except Exception:
                    logger.exception("Could not detach D3K [{}] range-failure handler", device.name)
            if subscription is not None:
                try:
                    node.uwb.measurements.unsubscribe(subscription)
                except Exception:
                    logger.exception("Could not unsubscribe D3K [{}] measurement stream", device.name)
            try:
                node.close()
            except Exception:
                logger.exception("Could not close D3K [{}] node", device.name)


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "--config",
    "config_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Path to the D3K diagnostics JSON configuration",
)
def main(config_path: Path) -> None:
    config = DiagnosticsConfiguration.from_file(config_path)
    stop_event = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _signum, _frame: stop_event.set())

    logger.info("Starting D3K diagnostics for {} device(s)", len(config.devices))
    run_diagnostics(config, stop_event)
    logger.info("D3K diagnostics stopped")


if __name__ == "__main__":
    main()
