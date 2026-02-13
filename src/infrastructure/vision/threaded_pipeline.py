from threading import Thread
from typing import Optional
import time
from domain.camera import Camera, LastestFrameBuffer
from domain.frame_processor import FrameProcessor


class ThreadedPipeline:
    def __init__(self, camera: Camera, frame_processor: FrameProcessor, frame_buffer: LastestFrameBuffer) -> None:
        self._camera = camera
        self._frame_processor = frame_processor
        self._frame_buffer = frame_buffer
        self._running = False
        self._thread: Optional[Thread] = None

    def start(self) -> None:
        self._camera.open()
        self._running = True
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._camera.close()

    def _loop(self) -> None:
        waiting_period = 1.0/max(1, self._camera.get_fps())
        while self._running:
            start_time = time.time()
            frame = self._camera.get_frame()
            vis, frame = self._frame_processor.apply(frame)
            self._frame_buffer.set(vis, frame)

            dt = time.time() - start_time
            if dt < waiting_period:
                time.sleep(waiting_period - dt)