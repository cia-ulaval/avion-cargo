import time
from threading import Thread
from typing import Callable, Optional, Tuple, Any

import numpy as np
from loguru import logger

from domain.camera import Camera, LastestFrameBuffer
from domain.tracking import TrackingResult


class ThreadedPipeline:
    def __init__(
        self,
        camera: Camera,
        frame_processor: Callable[[], Tuple[np.ndarray, TrackingResult]],
        frame_buffer: LastestFrameBuffer,
        com_channel:Optional[Callable[[dict[str, Any]], None]] = None,
    ) -> None:
        self._camera = camera
        self._frame_processor = frame_processor
        self._frame_buffer = frame_buffer
        self._running = False
        self._thread: Optional[Thread] = None
        self._com_channel = com_channel

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
        waiting_period = 1.0 / max(1, self._camera.get_fps())
        i = 0
        while self._running:
            start_time = time.time()
            vis = self._frame_processor()
            self._frame_buffer.set(vis, {"tracking_result": i})
            self._com_channel({"tracking_result": i})
            dt = time.time() - start_time
            if dt < waiting_period:
                time.sleep(waiting_period - dt)

            i += 1