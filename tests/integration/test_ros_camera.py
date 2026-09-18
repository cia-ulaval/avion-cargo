"""Optional real ROS 2 publisher/subscriber test; no Gazebo or camera required."""

import time
from threading import Event, Thread

import pytest


def test_ros_sensor_qos_padding_freshness_and_shutdown():
    rclpy = pytest.importorskip("rclpy", reason="Source a compatible ROS 2 environment to run this integration test")
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    from simulation.gazebo_camera import GazeboCamera

    rclpy.init()
    publisher_node = Node("autolander_integration_publisher")
    publisher = publisher_node.create_publisher(Image, "/autolander_test/image", qos_profile_sensor_data)
    done = Event()
    camera = GazeboCamera("/autolander_test/image", first_frame_timeout_sec=3, frame_timeout_sec=0.2)

    def publish():
        sequence = 0
        while not done.is_set():
            sequence += 1
            msg = Image()
            msg.header.stamp.sec = sequence
            msg.height, msg.width, msg.step = 1, 2, 8
            msg.encoding = "rgb8"
            msg.data = [1, 2, 3, 4, 5, 6, 99, 99]
            publisher.publish(msg)
            done.wait(0.02)

    thread = Thread(target=publish, daemon=True)
    thread.start()
    try:
        camera.open()
        assert camera.get_frame().tolist() == [[[3, 2, 1], [6, 5, 4]]]
        done.set()
        thread.join(1)
        time.sleep(0.3)
        with pytest.raises(TimeoutError):
            camera.get_frame()
    finally:
        done.set()
        thread.join(1)
        camera.close()
        publisher_node.destroy_node()
        rclpy.shutdown()
    assert camera._thread is None
