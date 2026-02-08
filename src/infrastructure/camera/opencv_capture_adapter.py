from dataclasses import dataclass

import cv2
import numpy as np

from domain.camera import Camera


@dataclass(slots=True)
class OpenCVCamera(Camera):
    """
    Camera adapter using OpenCV VideoCapture.

    source:
      - int (0,1,2...) for webcams
    """

    source: int | str = 0
    width: int = 1280
    height: int = 720
    fps: int | None = None
    rgb: bool = False  # OpenCV reads BGR by default

    _cap: cv2.VideoCapture | None = None

    def open(self) -> None:
        if self._cap is not None and self._cap.isOpened():
            return

        self._cap = cv2.VideoCapture(self.source)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open video source: {self.source}")

        # Best effort settings (drivers may ignore)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        if self.fps is not None:
            self._cap.set(cv2.CAP_PROP_FPS, float(self.fps))

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def get_frame(self) -> np.ndarray:
        if self._cap is None or not self._cap.isOpened():
            self.open()

        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read frame from VideoCapture")

        # frame is BGR. Convert to RGB if you standardize on RGB.
        if self.rgb:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return frame
