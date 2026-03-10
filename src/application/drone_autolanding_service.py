import time
from dataclasses import asdict
from threading import Thread
from typing import Optional

from application.tracking_service import TrackingService
from loguru import logger

from domain.drone import Drone
from infrastructure.camera.frame_buffer import FrameBuffer
from infrastructure.communication.webrtc_content_diffuser import WebRTCContentStreamer, WebRTCConfig
from infrastructure.vision.pose_buffer import PoseBuffer


class DroneAutolandingService:
    def __init__(self, drone: Drone, tracker: TrackingService, content_streamer_config: WebRTCConfig):
        self.drone = drone
        self.aruco_tracker = tracker
        self.frame_buffer = FrameBuffer()
        self.pose_buffer = PoseBuffer()
        self.content_streamer = WebRTCContentStreamer(self.frame_buffer, content_streamer_config)
        self._threads : dict[str, Thread] = dict()
        self._tracking_started: bool = False

    def _tracking_target_loop(self):
        waiting_period = 1.0 / max(1, self.aruco_tracker.camera.get_fps())

        while self._tracking_started:
            start_time = time.monotonic()

            vis, tracking_result = self.aruco_tracker.track_target()
            self.frame_buffer.set_value(vis, asdict(tracking_result))
            self.content_streamer.send_data(asdict(tracking_result))
            self.pose_buffer.set_value(tracking_result.pose)

            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            remaining_time = waiting_period - elapsed_time

            if remaining_time > 0:
                time.sleep(remaining_time)

    def _landing_target_loop(self):
        while self._tracking_started:
            pass

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

    def stop(self, what:Optional[str] = None):
        if what:
            thread_to_stop = self._threads.get(what)
            thread_to_stop.join()

        else :
            for thread in self._threads:
                self._threads[thread].join()