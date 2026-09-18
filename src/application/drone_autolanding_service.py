import time
from dataclasses import asdict
from math import isfinite
from threading import Event, Thread
from typing import Any

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
    ):
        if not isfinite(telemetry_dps) or telemetry_dps <= 0:
            raise ValueError("telemetry_dps must be finite and positive")
        self.drone = drone
        self.aruco_tracker = tracker
        self.frame_buffer = frame_buffer
        self.pose_buffer = pose_buffer
        self.drone_status_buffer = drone_status_buffer
        self.content_streamer = content_streamer
        self.telemetry_dps = telemetry_dps
        self._threads: dict[str, Thread] = dict()
        self._tracking_started: bool = False
        self._stop_event = Event()

    def _tracking_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())

        while self._tracking_started:
            start_time = time.monotonic()

            vis, tracking_result = self.aruco_tracker.track_target()
            self.frame_buffer.set_value(vis, tracking_result.to_dict())
            self.pose_buffer.set_value(tracking_result.pose)

            self.pose_buffer.set_uav_pose_value(tracking_result.uav_pose)

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

        return payload

    def _landing_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())
        try:
            self.drone.activate_land_mode()
        except Exception as e:
            logger.warning(f"Could not activate LAND mode: {e}")

        while self._tracking_started:
            start_time = time.monotonic()
            drone_status = self.drone.get_status()
            self.drone_status_buffer.set_value(drone_status)

            uav_pose = self.pose_buffer.get_uav_pose_value()
            if uav_pose is not None:
                target = self.aruco_tracker.get_target()
                target_size = target.length, target.length
                self.drone.land_on_target(uav_pose, target_size)

            self._stop_event.wait(max(0, waiting_period - (time.monotonic() - start_time)))

    def track_target(self):
        self._stop_event.clear()
        self._tracking_started = True
        self.aruco_tracker.camera.open()
        tracking_thread = Thread(target=self._tracking_target_loop, daemon=True)
        telemetry_thread = Thread(target=self._telemetry_loop, daemon=True)
        self._threads["tracking"] = tracking_thread
        self._threads["telemetry"] = telemetry_thread
        tracking_thread.start()
        telemetry_thread.start()

    def stream_video(self):
        if not self._tracking_started:
            logger.warning("The target's tracking is not started yet. Streaming video")

        video_streaming_thread = Thread(target=self.content_streamer.stream_video, daemon=True)
        self._threads["streaming"] = video_streaming_thread
        video_streaming_thread.start()

    def perform_precision_landing(self):
        if not self._tracking_started:
            raise SystemError("The target's tracking is not started yet. Cannot start precision landing")

        self._landing_target_loop()

    def stop_streaming(self):
        streaming_thread = self._threads.get("streaming")
        if streaming_thread:
            streaming_thread.join()

    def stop_tracking(self):
        if not self._tracking_started:
            return
        self._tracking_started = False
        self._stop_event.set()
        tracking_thread = self._threads.get("tracking")
        telemetry_thread = self._threads.get("telemetry")
        tracking_thread.join()
        if telemetry_thread:
            telemetry_thread.join()

    def stop(self):
        self.stop_streaming()
        self.stop_tracking()
