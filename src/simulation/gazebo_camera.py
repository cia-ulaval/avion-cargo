import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from domain.camera import Camera


class _GazeboCameraNode(Node):
    def __init__(self, topic_name: str) -> None:
        super().__init__("gazebo_camera_node")
        self._last_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._first_frame_event = threading.Event()

        self._subscription = self.create_subscription(
            Image,
            topic_name,
            self._on_image,
            10,
        )

    def _on_image(self, msg: Image) -> None:
        frame = self._image_msg_to_bgr(msg)
        with self._lock:
            self._last_frame = frame
        self._first_frame_event.set()

    @staticmethod
    def _image_msg_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise RuntimeError(f"Unsupported encoding: {msg.encoding}")

        channels = 3
        expected_size = msg.height * msg.width * channels
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if data.size != expected_size:
            raise RuntimeError(f"Unexpected image size: got {data.size}, expected {expected_size}")

        frame = data.reshape((msg.height, msg.width, channels))

        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]  # RGB -> BGR

        return frame.copy()

    def wait_first_frame(self, timeout_sec: float) -> bool:
        return self._first_frame_event.wait(timeout=timeout_sec)

    def get_latest_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._last_frame is None else self._last_frame.copy()


class GazeboCamera(Camera):
    """
    ROS 2 camera subscriber reading sensor_msgs/Image and returning BGR frames.
    No cv_bridge required.
    """

    def __init__(
        self,
        topic_name: str,
        fps: int = 10,
        first_frame_timeout_sec: float = 5.0,
    ) -> None:
        self._topic_name = topic_name
        self._fps = fps
        self._first_frame_timeout_sec = first_frame_timeout_sec

        self._node: Optional[_GazeboCameraNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._rclpy_initialized_here = False

    def open(self) -> None:
        if self._running:
            return

        if not rclpy.ok():
            rclpy.init()
            self._rclpy_initialized_here = True

        self._node = _GazeboCameraNode(self._topic_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        if not self._node.wait_first_frame(self._first_frame_timeout_sec):
            self.close()
            raise RuntimeError(
                f"No image received on topic '{self._topic_name}' " f"within {self._first_frame_timeout_sec} seconds."
            )

    def _spin(self) -> None:
        assert self._executor is not None
        while self._running:
            self._executor.spin_once(timeout_sec=0.1)

    def close(self) -> None:
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._executor = None

        if self._rclpy_initialized_here and rclpy.ok():
            rclpy.shutdown()
            self._rclpy_initialized_here = False

    def get_frame(self) -> np.ndarray:
        if self._node is None:
            raise RuntimeError("GazeboCamera is not open.")

        frame = self._node.get_latest_frame()
        if frame is None:
            raise RuntimeError("No frame available yet.")

        return frame

    def get_fps(self) -> int:
        return self._fps
