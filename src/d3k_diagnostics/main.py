from __future__ import annotations

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


@dataclass(frozen=True, slots=True)
class DeviceConfiguration:
    name: str
    uart_device: str
    baudrate: int = 460800
    ready_timeout_s: float = 10.0
    peer_id: int | None = None


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
        return bool(
            self.failures_by_status
            or self.invalid_measurements
            or self.dropped_measurements
        )


class _RangingStatistics:
    """Aggregate ranging results for the one peer expected by this application."""

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
                stddev_distance_m=(
                    math.sqrt(self._sum_squared_deviations / count) if count else None
                ),
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

        failures = ", ".join(
            f"status {status}: {count}" for status, count in summary.failures_by_status
        ) or "0"
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


def _attach_range_failure_collection(node: Any, statistics: _RangingStatistics) -> Any:
    def collect(failure: Any) -> None:
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


def _poll_device(node: Any, device: DeviceConfiguration) -> None:
    state = node.get_state(refresh=True)
    logger.info("D3K [{}] readiness: connected={}, ready={}", device.name, node.is_connected, node.is_ready)
    logger.info("D3K [{}] state/configuration: {}", device.name, state)
    logger.info("D3K [{}] host diagnostics: {}", device.name, node.get_diagnostics())

    if device.peer_id is not None:
        try:
            peer_status = node.ble.get_peer_status(device.peer_id)
            logger.info("D3K [{}] BLE peer {} status: {}", device.name, device.peer_id, peer_status)
        except Exception:
            logger.exception("D3K [{}] could not read BLE peer {} status", device.name, device.peer_id)


def run_diagnostics(config: DiagnosticsConfiguration, stop_event: threading.Event) -> None:
    try:
        from d3kble import D3KNode
    except ImportError as exc:
        raise RuntimeError("The d3kble package is required to run D3K diagnostics") from exc

    sessions: list[tuple[DeviceConfiguration, Any, Any | None]] = []
    dropped_counts: dict[str, int] = {}
    ranging_statistics: dict[str, _RangingStatistics] = {}
    range_failure_handlers: dict[str, Any] = {}
    try:
        for device in config.devices:
            logger.info("Connecting D3K [{}] through {}", device.name, device.uart_device)
            node = D3KNode(device.uart_device, baudrate=device.baudrate)
            sessions.append((device, node, None))
            _attach_event_logging(node, device.name)
            statistics = _RangingStatistics(device.peer_id)
            ranging_statistics[device.name] = statistics
            range_failure_handlers[device.name] = _attach_range_failure_collection(node, statistics)
            subscription = node.uwb.measurements.subscribe(maxsize=256, overflow="drop_oldest")
            sessions[-1] = (device, node, subscription)
            dropped_counts[device.name] = 0
            node.start()
            node.wait_until_ready(timeout=device.ready_timeout_s)
            logger.info("D3K [{}] ready; initial state: {}", device.name, node.get_state(refresh=True))

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
                    ranging_statistics[device.name].add_measurement(measurement)
                dropped = subscription.dropped_count
                if dropped > dropped_counts[device.name]:
                    ranging_statistics[device.name].add_dropped_measurements(
                        dropped - dropped_counts[device.name]
                    )
                    dropped_counts[device.name] = dropped

            now = time.monotonic()
            if now >= next_ranging_summary:
                interval_s = now - last_ranging_summary
                for device, _node, _subscription in sessions:
                    ranging_statistics[device.name].log_summary(device.name, interval_s)
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
