import numpy as np
from picamera2 import Picamera2

from domain.camera import Camera


class PiCameraAdapter(Camera):
    def __init__(self, width: int = 1280, height: int = 720, rgb: bool = True):
        self._cam = Picamera2()
        self._width = width
        self._height = height
        self._rgb = rgb
        self._started = False

    def open(self) -> None:
        if self._started:
            return
        config = self._cam.create_preview_configuration(
            main={
                "size": (self._width, self._height),
                "format": "RGB888" if self._rgb else "BGR888",
            }
        )
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
