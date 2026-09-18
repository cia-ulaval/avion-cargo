import time
from typing import Any, Dict

import numpy as np
from picamera2 import Picamera2

from domain.camera import Camera

MICROSECONDS_PER_SECOND = 1_000_000


class PiCameraAdapter(Camera):
    def __init__(self, fps: int, width: int = 1280, height: int = 720, rgb: bool = True):
        self._cam = None
        self.last_capture_time_s = None
        self._cam_config = self.__make_config(width=width, height=height, is_rgb_cam=rgb)
        self._cam_controls = self.__make_controls(fps=fps)
        self._started = False
        self._fps = fps

    @staticmethod
    def __make_config(width: int, height: int, is_rgb_cam: bool) -> Dict[str, Any]:
        config = {
            "size": (width, height),
            "format": "RGB888" if is_rgb_cam else "BGR888",
        }

        return config

    @staticmethod
    def __make_controls(fps: int) -> Dict[str, Any]:
        frame_duration = int(MICROSECONDS_PER_SECOND / fps)
        controls = {"FrameDurationLimits": (frame_duration, frame_duration)}
        return controls

    def open(self) -> None:
        if self._started:
            return

        if self._cam is None:
            self._cam = Picamera2()
        config = self._cam.create_preview_configuration(main=self._cam_config, controls=self._cam_controls)
        self._cam.configure(config)
        self._cam.start()
        self._started = True

    def close(self) -> None:
        camera, self._cam = self._cam, None
        self._started = False
        if camera is not None:
            camera.close()

    def get_frame(self) -> np.ndarray:
        if not self._started:
            self.open()
        captured_at_s = time.monotonic()
        frame = self._cam.capture_array()
        self.last_capture_time_s = captured_at_s
        return frame

    def get_fps(self) -> int:
        return self._fps
