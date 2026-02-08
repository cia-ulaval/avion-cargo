from typing import Any, Dict

import numpy as np
from picamera2 import Picamera2

from domain.camera import Camera

NUMBER_OF_MILLISECONDS_IN_ONE_SECOND = 1_000_000


class PiCameraAdapter(Camera):
    def __init__(self, fps: int, width: int = 1280, height: int = 720, rgb: bool = True):
        self._cam = Picamera2()
        self._cam_config = self.__make_config(width=width, height=height, is_rgb_cam=rgb)
        self._cam_controls = self.__make_controls(fps=fps)
        self._started = False

    def __make_config(self, width: int, height: int, is_rgb_cam: bool) -> Dict[str, Any]:
        config = {
            "size": (width, height),
            "format": "RGB888" if is_rgb_cam else "BGR888",
        }

        return config

    def __make_controls(self, fps: int) -> Dict[str, Any]:
        frame_duration = int(NUMBER_OF_MILLISECONDS_IN_ONE_SECOND / fps)
        controls = {"FrameDurationsLimits": (frame_duration, frame_duration)}
        return controls

    def open(self) -> None:
        if self._started:
            return

        config = self._cam.create_preview_configuration(main=self._cam_config, controls=self._cam_controls)
        self._cam.configure(config)
        self._cam.start()
        self._started = True

    def close(self) -> None:
        if self._started:
            self._cam.stop()
            self._started = False

    def get_frame(self) -> np.ndarray:
        if not self._started:
            self.open()
        frame = self._cam.capture_array()
        return frame
