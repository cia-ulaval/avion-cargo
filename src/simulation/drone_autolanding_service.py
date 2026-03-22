import time
from dataclasses import asdict
from threading import Thread
from typing import Optional

from loguru import logger

from application.tracking_service import TrackingService
from domain.drone import Drone
from domain.models import Pose3D
from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.communication.webrtc_content_diffuser import WebRTCConfig, WebRTCContentStreamer
from infrastructure.vision.pose_buffer import PoseBuffer


class DroneAutolandingService:
    def __init__(self, drone: Drone, tracker: TrackingService, content_streamer_config: WebRTCConfig):
        self.drone = drone
        self.aruco_tracker = tracker
        self.frame_buffer = FrameBuffer()
        self.pose_buffer = PoseBuffer()
        self.content_streamer = WebRTCContentStreamer(self.frame_buffer, content_streamer_config)
        self._threads: dict[str, Thread] = dict()
        self._tracking_started: bool = False

    def _tracking_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())

        while self._tracking_started:
            start_time = time.monotonic()

            vis, tracking_result = self.aruco_tracker.track_target()
            self.frame_buffer.set_value(vis, asdict(tracking_result))
            self.pose_buffer.set_value(tracking_result.pose)

            uav_pose = self._to_uav_pose(tracking_result.pose)
            self.pose_buffer.set_uav_pose_value(uav_pose)

            self.content_streamer.send_data(tracking_result.to_dict())

            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            remaining_time = waiting_period - elapsed_time

            if remaining_time > 0:
                time.sleep(remaining_time)

    @staticmethod
    def _to_uav_pose(estimated_pose: Optional[Pose3D]) -> Optional[Pose3D]:
        if estimated_pose is None:
            return None

        return Pose3D(
            x=-estimated_pose.y,
            y=estimated_pose.x,
            z=estimated_pose.z,
        )

    def _landing_target_loop(self):
        waiting_period = 1.0 / 30.0
        try:
            self.drone.activate_land_mode()
        except Exception as e:
            logger.warning(f"Could not activate LAND mode: {e}")

        while self._tracking_started:
            start_time = time.monotonic()

            uav_pose = self.pose_buffer.get_uav_pose_value()
            if uav_pose is not None:
                logger.info(f"UAV pose sent is {uav_pose}")
                self.drone.land_on_target(uav_pose)

            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            remaining_time = waiting_period - elapsed_time

            if remaining_time > 0:
                time.sleep(remaining_time)

    def track_target(self):
        self._tracking_started = True
        self.aruco_tracker.camera.open()
        tracking_thread = Thread(target=self._tracking_target_loop, daemon=True)
        self._threads["tracking"] = tracking_thread
        tracking_thread.start()

    def stream_landing_video(self):
        if not self._tracking_started:
            logger.warning("The target's tracking is not started yet. Streaming video")

        video_streaming_thread = Thread(target=self.content_streamer.stream_video, daemon=True)
        self._threads["streaming"] = video_streaming_thread
        video_streaming_thread.start()

    def land(self):
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
        tracking_thread = self._threads.get("tracking")
        tracking_thread.join()
        self._tracking_started = False

    def stop(self):
        self.stop_streaming()
        self.stop_tracking()
