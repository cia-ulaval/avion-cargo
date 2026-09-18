import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest


@pytest.fixture
def camera_module(monkeypatch):
    for name in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg'):
        monkeypatch.setitem(__import__('sys').modules, name, ModuleType(name))
    class Node:
        def __init__(self, _name):
            self.create_subscription = Mock()
    sys.modules['rclpy.node'].Node = Node
    sys.modules['rclpy.executors'].SingleThreadedExecutor = Mock()
    sys.modules['rclpy.qos'].QoSProfile = lambda **kw: SimpleNamespace(**kw)
    sys.modules['rclpy.qos'].HistoryPolicy = SimpleNamespace(KEEP_LAST=1)
    sys.modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=2)
    sys.modules['sensor_msgs.msg'].Image = object
    spec = importlib.util.spec_from_file_location('gazebo_under_test', Path(__file__).parents[2] / 'src/simulation/gazebo_camera.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def message(encoding='rgb8', step=8, data=bytes([1, 2, 3, 4, 5, 6, 99, 99]), stamp=1):
    return SimpleNamespace(encoding=encoding, height=1, width=2, step=step, data=data,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))


def test_padded_rgb_rows_decode_without_padding(camera_module):
    image = camera_module._GazeboCameraNode._image_msg_to_bgr(message())
    np.testing.assert_array_equal(image, [[[3, 2, 1], [6, 5, 4]]])


def test_mono_images_expand_to_bgr(camera_module):
    image = camera_module._GazeboCameraNode._image_msg_to_bgr(message('mono8', 3, bytes([3, 4, 99])))
    np.testing.assert_array_equal(image, [[[3, 3, 3], [4, 4, 4]]])


def test_qos_and_duplicate_source_timestamps(camera_module):
    node = camera_module._GazeboCameraNode('/image')
    qos = node.create_subscription.call_args.args[3]
    assert qos.depth == 1 and qos.reliability == 2
    node._on_image(message())
    assert node.wait_first_frame(0)
    node.take_latest_frame()
    node._on_image(message())
    assert not node.wait_first_frame(0)
    node._on_image(message(stamp=2))
    assert node.wait_first_frame(0)


def test_camera_never_reuses_an_old_frame(camera_module):
    camera = camera_module.GazeboCamera('/image', frame_timeout_sec=.01)
    camera._running = True
    camera._node = camera_module._GazeboCameraNode('/image')
    camera._node._on_image(message())
    camera.get_frame()
    with pytest.raises(TimeoutError, match='fresh'):
        camera.get_frame()


def test_executor_failure_reaches_camera_consumer(camera_module):
    camera = camera_module.GazeboCamera('/image')
    camera._running = True
    camera._node = camera_module._GazeboCameraNode('/image')
    camera._executor = Mock()
    camera._executor.spin_once.side_effect = RuntimeError('callback failed')
    camera._spin()
    with pytest.raises(RuntimeError, match='executor failed'):
        camera.get_frame()
