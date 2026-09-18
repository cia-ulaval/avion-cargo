import time
from dataclasses import asdict
from math import isfinite
from threading import Event, Lock, Thread, current_thread
from typing import Any, Callable

from loguru import logger

from application.tracking_service import TrackingService
from domain.buffer import DroneStatusBufferPort, FrameBufferPort, PoseBufferPort
from domain.content_streamer import ContentStreamer
from domain.drone import Drone


class DroneAutolandingService:
    def __init__(
        self,
        drone: Drone,
        tracker: TrackingService,
        *,
        content_streamer: ContentStreamer,
        frame_buffer: FrameBufferPort,
        pose_buffer: PoseBufferPort,
        drone_status_buffer: DroneStatusBufferPort,
        telemetry_dps: float = 5,
        shutdown_timeout_s: float = 5,
    ):
        if not isfinite(telemetry_dps) or telemetry_dps <= 0:
            raise ValueError("telemetry_dps must be finite and positive")
        if not isfinite(shutdown_timeout_s) or shutdown_timeout_s <= 0:
            raise ValueError("shutdown_timeout_s must be finite and positive")
        self.drone = drone
        self.aruco_tracker = tracker
        self.frame_buffer = frame_buffer
        self.pose_buffer = pose_buffer
        self.drone_status_buffer = drone_status_buffer
        self.content_streamer = content_streamer
        self.telemetry_dps = telemetry_dps
        self.shutdown_timeout_s = shutdown_timeout_s
        self._threads: dict[str, Thread] = dict()
        self._tracking_started: bool = False
        self._stop_event = Event()
        self._failure_lock = Lock()
        self._worker_failure: tuple[str, Exception] | None = None
        self._closed = False

    def request_stop(self) -> None:
        self._tracking_started = False
        self._stop_event.set()

    def _raise_worker_failure(self) -> None:
        with self._failure_lock:
            failure = self._worker_failure
        if failure is not None:
            name, error = failure
            raise RuntimeError(f"Autolander worker '{name}' failed") from error

    def _run_worker(self, name: str, work: Callable[[], None]) -> None:
        try:
            work()
            if not self._stop_event.is_set():
                raise RuntimeError("Worker exited unexpectedly")
        except Exception as error:
            if self._stop_event.is_set() and self._closed:
                logger.opt(exception=error).warning("Worker {} interrupted during shutdown", name)
                return
            logger.opt(exception=error).error("Autolander worker {} failed", name)
            with self._failure_lock:
                if self._worker_failure is None:
                    self._worker_failure = name, error
            self.request_stop()

    def _start_worker(self, name: str, work: Callable[[], None]) -> None:
        thread = Thread(target=self._run_worker, args=(name, work), name=f"autolander-{name}", daemon=True)
        self._threads[name] = thread
        thread.start()

    def _tracking_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())
        previous_status = None

        while self._tracking_started:
            start_time = time.monotonic()

            vis, tracking_result = self.aruco_tracker.track_target()
            if self._stop_event.is_set():
                break
            self.frame_buffer.set_value(vis, tracking_result.to_dict(), timestamp=tracking_result.captured_at_s)
            self.pose_buffer.set_value(tracking_result.pose, timestamp=tracking_result.captured_at_s)

            self.pose_buffer.set_uav_pose_value(tracking_result.uav_pose, timestamp=tracking_result.captured_at_s)
            if tracking_result.status != previous_status:
                logger.info("Target tracking: {}", tracking_result.status.name)
                previous_status = tracking_result.status

            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            remaining_time = waiting_period - elapsed_time

            if remaining_time > 0:
                self._stop_event.wait(remaining_time)

    def _telemetry_loop(self):
        waiting_period = 1.0 / self.telemetry_dps

        while self._tracking_started:
            start_time = time.monotonic()

            self.content_streamer.send_data(self._build_telemetry_payload())

            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            remaining_time = waiting_period - elapsed_time

            if remaining_time > 0:
                self._stop_event.wait(remaining_time)

    def _build_telemetry_payload(self) -> dict[str, Any]:
        _frame, tracking_metadata = self.frame_buffer.get_value()
        payload: dict[str, Any] = dict(tracking_metadata or {})

        drone_status = self.drone_status_buffer.get_value()
        if drone_status is not None:
            payload["drone"] = asdict(drone_status)
            payload["drone"]["connected"] = drone_status.connected
            payload["drone"].pop("last_heartbeat_monotonic_s", None)

        return payload

    def _landing_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())
        self._raise_worker_failure()
        logger.info("Requesting LAND mode")
        self.drone.activate_land_mode()
        logger.info("Landing target emission active at {} Hz", 1 / waiting_period)

        while self._tracking_started:
            start_time = time.monotonic()
            self._raise_worker_failure()
            drone_status = self.drone.get_status()
            self.drone_status_buffer.set_value(drone_status)

            uav_pose = self.pose_buffer.get_uav_pose_value()
            if uav_pose is not None and drone_status.connected:
                target = self.aruco_tracker.get_target()
                target_size = target.length, target.length
                self.drone.land_on_target(uav_pose, target_size)

            self._stop_event.wait(max(0, waiting_period - (time.monotonic() - start_time)))
        self._raise_worker_failure()

    def track_target(self):
        if self._closed or self._threads:
            raise RuntimeError("Landing service has already been started or stopped")
        logger.info("Opening tracking camera")
        self.aruco_tracker.camera.open()
        self._stop_event.clear()
        self._tracking_started = True
        self._start_worker("tracking", self._tracking_target_loop)
        self._start_worker("telemetry", self._telemetry_loop)
        logger.info("Tracking started; telemetry cadence {} Hz", self.telemetry_dps)

    def stream_video(self):
        self._raise_worker_failure()
        if not self._tracking_started:
            raise RuntimeError("Target tracking must be started before streaming")
        if "streaming" in self._threads:
            raise RuntimeError("Video streaming is already started")
        self._start_worker("streaming", self.content_streamer.stream_video)

    def perform_precision_landing(self):
        self._raise_worker_failure()
        if not self._tracking_started:
            raise SystemError("The target's tracking is not started yet. Cannot start precision landing")

        self._landing_target_loop()

    def stop_streaming(self):
        self.content_streamer.stop()
        self._join_threads(["streaming"], time.monotonic() + self.shutdown_timeout_s)

    def stop_tracking(self):
        self.request_stop()
        self._join_threads(["tracking", "telemetry"], time.monotonic() + self.shutdown_timeout_s)

    def _join_threads(self, names: list[str], deadline: float) -> None:
        pending = []
        for name in names:
            thread = self._threads.get(name)
            if thread is None or thread.ident is None or thread is current_thread():
                continue
            thread.join(timeout=max(0, deadline - time.monotonic()))
            if thread.is_alive():
                pending.append(name)
        if pending:
            raise TimeoutError(f"Shutdown timed out for: {', '.join(pending)}")

    def stop(self):
        if self._closed:
            return
        self._closed = True
        logger.info("Stopping tracking, camera, WebRTC and MAVLink")
        self.request_stop()
        deadline = time.monotonic() + self.shutdown_timeout_s
        errors: list[Exception] = []

        def close_resource(name: str, close: Callable[[], None]) -> None:
            try:
                close()
            except Exception as error:
                logger.opt(exception=error).error("Could not close {}", name)
                with self._failure_lock:
                    errors.append(error)

        # Drivers can block while closing. All resources get a shutdown request
        # and share a single bounded deadline, even if another close fails.
        for name, close in (
            ("streaming", self.content_streamer.stop),
            ("camera", self.aruco_tracker.camera.close),
            ("drone", self.drone.close),
        ):
            thread = Thread(target=close_resource, args=(name, close), name=f"autolander-close-{name}", daemon=True)
            self._threads[f"close-{name}"] = thread
            thread.start()
        try:
            self._join_threads(list(self._threads), deadline)
        except TimeoutError:
            logger.exception("Autolander shutdown did not finish within {} seconds", self.shutdown_timeout_s)
            raise
        if errors:
            raise RuntimeError("Autolander resource cleanup failed") from errors[0]
        logger.info("Autolander stopped")
