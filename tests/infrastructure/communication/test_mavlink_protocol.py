"""Decode actual packets to verify the public MAVLink contract."""

import math
from unittest.mock import Mock

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from domain.models import Pose3D
from infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams


def connected_drone():
    drone = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", 14550))
    drone.connection = mavutil.mavfile(None, "memory", source_component=191, input=False)
    drone.connection.write = Mock()
    drone._configure_mavlink2()
    return drone


def last_packet(drone):
    packet = drone.connection.write.call_args.args[0]
    assert packet[0] == 0xFD
    return mavlink2.MAVLink(None).parse_char(packet)


def test_mavlink2_extensions_survive_without_environment_variable(monkeypatch):
    monkeypatch.delenv("MAVLINK20", raising=False)
    drone = connected_drone()
    # Receiving a MAVLink 1 heartbeat must not downgrade our outbound messages.
    drone.connection.auto_mavlink_version(bytes([0xFE]))
    drone.land_on_target(Pose3D(0.3, 0.2, 2.0), (0.4, 0.4))
    message = last_packet(drone)
    assert message.get_type() == "LANDING_TARGET"
    assert message.get_srcComponent() == 191
    assert message.position_valid == 1
    assert message.frame == mavlink2.MAV_FRAME_BODY_FRD
    assert message.z == 2.0


def test_companion_heartbeat_uses_protocol_defined_version_field():
    drone = connected_drone()
    drone._send_heartbeat()
    message = last_packet(drone)
    assert message.type == mavlink2.MAV_TYPE_ONBOARD_CONTROLLER
    assert message.autopilot == mavlink2.MAV_AUTOPILOT_INVALID
    assert message.mavlink_version == 3


def test_position_branch_keeps_metric_values_and_omits_unknown_image_angles():
    drone = connected_drone()
    drone.land_on_target(Pose3D(0.3, -0.2, 2.0), (0.4, 0.4))
    message = last_packet(drone)
    assert (message.x, message.y, message.z) == pytest.approx((0.3, -0.2, 2.0))
    assert message.distance == pytest.approx(math.sqrt(4.13))
    assert (message.angle_x, message.angle_y, message.size_x, message.size_y) == (0, 0, 0, 0)
    assert message.position_valid == 1


@pytest.mark.parametrize("z", [0, -1])
def test_target_at_or_above_vehicle_is_not_transmitted(z):
    drone = connected_drone()
    with pytest.raises(ValueError, match="below the vehicle"):
        drone.land_on_target(Pose3D(0.2, 0.3, z), (0.4, 0.4))
    drone.connection.write.assert_not_called()


def test_target_requires_a_connection():
    drone = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", 14550))
    with pytest.raises(RuntimeError, match="not connected"):
        drone.land_on_target(Pose3D(0, 0, 2), (0.4, 0.4))
