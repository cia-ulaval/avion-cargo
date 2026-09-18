import threading
import time
from typing import Optional

import numpy as np
import rclpy
from loguru import logger
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from domain.camera import Camera


class _GazeboCameraNode(Node):
    def __init__(self, topic_name: str) -> None:
        super().__init__("gazebo_camera_node")
        self._last_frame = None
        self._captured_at_s = 0.0
        self._last_stamp_ns = None
        self._lock = threading.Lock()
        self._first_frame_event = threading.Event()
        # A latest-image consumer must also accept best-effort sensor publishers.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscription = self.create_subscription(Image, topic_name, self._on_image, qos)

    def _on_image(self, msg: Image) -> None:
        captured_at_s = time.monotonic()
        frame = self._image_msg_to_bgr(msg)
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        with self._lock:
            # Zero stamps mean the publisher has no source timestamp. For stamped
            # streams, duplicates/out-of-order samples must not refresh freshness.
            if stamp_ns and self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
                return
            if stamp_ns:
                self._last_stamp_ns = stamp_ns
            self._last_frame = frame
            self._captured_at_s = captured_at_s
            self._first_frame_event.set()

    @staticmethod
    def _image_msg_to_bgr(msg: Image) -> np.ndarray:
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
        if channels is None:
            raise RuntimeError(f"Unsupported encoding: {msg.encoding}")
        if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * channels:
            raise RuntimeError("Invalid image dimensions or row stride")
        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = msg.height * msg.step
        if data.size != expected_size:
            raise RuntimeError(f"Unexpected image size: got {data.size}, expected {expected_size}")
        rows = data.reshape(msg.height, msg.step)
        frame = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if channels == 1:
            frame = np.repeat(frame, 3, axis=2)
        else:
            frame = frame[:, :, :3]
            if msg.encoding in ("rgb8", "rgba8"):
                frame = frame[:, :, ::-1]
        return frame.copy()

    def wait_first_frame(self, timeout_sec: float) -> bool:
        return self._first_frame_event.wait(timeout_sec)

    def take_latest_frame(self):
        with self._lock:
            self._first_frame_event.clear()
            return (None if self._last_frame is None else self._last_frame.copy(), self._captured_at_s)


class GazeboCamera(Camera):
    """ROS 2 latest-image adapter; freshness uses host monotonic time, not /clock."""

    def __init__(
        self, topic_name: str, fps: int = 10, first_frame_timeout_sec: float = 5.0, frame_timeout_sec: float = 1.0
    ) -> None:
        if fps <= 0 or not 0 < frame_timeout_sec < float("inf"):
            raise ValueError("Camera fps and frame timeout must be positive")
        self._topic_name = topic_name
        self._fps = fps
        self._first_frame_timeout_sec = first_frame_timeout_sec
        self._frame_timeout_sec = frame_timeout_sec
        self._node: Optional[_GazeboCameraNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._rclpy_initialized_here = False
        self._failure: Exception | None = None
        self.last_capture_time_s: float | None = None

    def open(self) -> None:
        if self._running:
            return
        self._failure = None
        try:
            if not rclpy.ok():
                rclpy.init()
                self._rclpy_initialized_here = True
            self._node = _GazeboCameraNode(self._topic_name)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._running = True
            self._thread = threading.Thread(target=self._spin, name="autolander-ros-camera", daemon=True)
            self._thread.start()
            if not self._node.wait_first_frame(self._first_frame_timeout_sec):
                raise TimeoutError(f"No image received on {self._topic_name} within {self._first_frame_timeout_sec}s")
            self._raise_failure()
            logger.info("ROS camera ready on {} at {} Hz", self._topic_name, self._fps)
        except BaseException:
            self.close()
            raise

    def _raise_failure(self):
        if self._failure is not None:
            raise RuntimeError("ROS camera executor failed") from self._failure

    def _spin(self) -> None:
        try:
            while self._running:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            if self._running:
                self._failure = error
                logger.exception("ROS camera executor failed")
        finally:
            if self._node is not None:
                self._node._first_frame_event.set()

    def close(self) -> None:
        self._running = False
        if self._node is not None:
            self._node._first_frame_event.set()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                raise TimeoutError("ROS camera executor did not stop")
            self._thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._executor = None
        if self._rclpy_initialized_here:
            if rclpy.ok():
                rclpy.shutdown()
            self._rclpy_initialized_here = False

    def get_frame(self) -> np.ndarray:
        node = self._node
        if node is None or not self._running:
            raise RuntimeError("GazeboCamera is not open")
        self._raise_failure()
        if not node.wait_first_frame(self._frame_timeout_sec):
            raise TimeoutError(f"No fresh image on {self._topic_name} within {self._frame_timeout_sec}s")
        self._raise_failure()
        if not self._running:
            raise RuntimeError("GazeboCamera is closing")
        frame, captured_at_s = node.take_latest_frame()
        if frame is None or time.monotonic() - captured_at_s >= self._frame_timeout_sec:
            raise TimeoutError("ROS camera image has expired")
        self.last_capture_time_s = captured_at_s
        return frame

    def get_fps(self) -> int:
        return self._fps
