from picamera2 import Picamera2
from typing import Tuple

from camera import Camera

class PiCamera(Picamera2, Camera):
    def __init__(self, resolution: Tuple[int, int] = (640, 480), framerate: int = 30, _format: str = "RGB888"):
        super().__init__()
        self.video_config = super().create_video_configuration(
            main={"size": resolution, "format": _format},
            controls = {"FrameRate": float(framerate)},
        )
        self.configure(self.video_config)

    def open(self):
        self.start()

    def close(self):
        self.stop()

