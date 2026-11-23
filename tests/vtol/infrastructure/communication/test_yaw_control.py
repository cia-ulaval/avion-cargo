from dataclasses import replace
from unittest.mock import Mock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from vtol.domain.drone import DroneMode, HoldPosition
from .test_mavlink_protocol import connected_drone, last_packet


@pytest.fixture
def vehicle(sample_drone_status):
    drone = connected_drone()
    drone.connection.close = Mock()
    drone.connection.recv_match = Mock(return_value=None)
    drone.status = replace(sample_drone_status(), mode=DroneMode.GUIDED, armed=True)
    drone._target_system, drone._target_component = 7, 1
    state = drone.connection.sysid_state[drone.connection.sysid]
    state.mav_type = mavlink2.MAV_TYPE_FIXED_WING
    state.mav_autopilot = mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA
    return drone


def parameters(vehicle, *, channel=4, reversed_=0):
    values = {
        "Q_ENABLE": 1,
        "Q_GUIDED_MODE": 1,
        "STICK_MIXING": 1,
        "RCMAP_YAW": channel,
        "RC_OVERRIDE_TIME": 0.5,
        "RC_OPTIONS": 0,
        "Q_WVANE_ENABLE": 0,
        "MAV_GCS_SYSID": vehicle.connection.source_system,
        f"RC{channel}_MIN": 1000,
        f"RC{channel}_TRIM": 1500,
        f"RC{channel}_MAX": 2000,
        f"RC{channel}_REVERSED": reversed_,
        f"RC{channel}_DZ": 30,
    }
    for name, value in values.items():
        vehicle._handle_message(
            mavlink2.MAVLink_param_value_message(
                name.encode("ascii"), value, mavlink2.MAV_PARAM_TYPE_REAL32, len(values), 0
            )
        )


def test_preflight_requests_actual_parameters_and_attitude_without_flight_commands(vehicle):
    assert not vehicle.prepare_yaw_control()
    packets = [mavlink2.MAVLink(None).parse_char(call.args[0]) for call in vehicle.connection.write.call_args_list]
    assert {packet.get_type() for packet in packets} == {"COMMAND_LONG", "PARAM_REQUEST_READ"}
    command = next(packet for packet in packets if packet.get_type() == "COMMAND_LONG")
    assert command.command == mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL
    assert command.param1 == mavlink2.MAVLINK_MSG_ID_ATTITUDE
    parameters(vehicle)
    assert vehicle.prepare_yaw_control()


@pytest.mark.parametrize(
    "name,value",
    [
        ("Q_ENABLE", 0),
        ("Q_GUIDED_MODE", 0),
        ("STICK_MIXING", 0),
        ("Q_WVANE_ENABLE", 1),
        ("RC_OVERRIDE_TIME", -1),
        ("RC_OVERRIDE_TIME", 3),
        ("RC_OPTIONS", 2),
        ("MAV_GCS_SYSID", 42),
        ("RC4_TRIM", 2500),
        ("RC4_DZ", 500),
        ("RCMAP_YAW", 17),
    ],
)
def test_incompatible_fc_settings_prevent_yaw_control(vehicle, name, value):
    parameters(vehicle)
    vehicle._flight_parameters[name] = value
    with pytest.raises(ValueError):
        vehicle.send_yaw_input(0.1)
    packets = [mavlink2.MAVLink(None).parse_char(call.args[0]) for call in vehicle.connection.write.call_args_list]
    assert all(packet.get_type() != "RC_CHANNELS_OVERRIDE" for packet in packets)


def test_mode_request_is_targeted_and_does_not_claim_mode_has_changed(vehicle):
    vehicle.status.mode = DroneMode.QLAND
    vehicle.request_mode(DroneMode.GUIDED)
    message = last_packet(vehicle)
    assert message.command == mavlink2.MAV_CMD_DO_SET_MODE
    assert (message.target_system, message.target_component) == (7, 1)
    assert message.param1 == mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert message.param2 == 15  # ArduPlane GUIDED, rather than Copter's 4.
    assert vehicle.status.mode is DroneMode.QLAND


@pytest.mark.parametrize("channel", [4, 10])
@pytest.mark.parametrize("reversed_", [0, 1])
def test_only_mapped_yaw_is_overridden_and_released_with_correct_wire_sentinel(vehicle, channel, reversed_):
    parameters(vehicle, channel=channel, reversed_=reversed_)
    vehicle.send_yaw_input(0.2)
    message = last_packet(vehicle)
    assert message.get_type() == "RC_CHANNELS_OVERRIDE"
    for other in range(1, 19):
        value = getattr(message, f"chan{other}_raw")
        if other == channel:
            assert value == (1376 if reversed_ else 1624)
        else:
            assert value == 65535
    vehicle.send_yaw_input(0)
    assert getattr(last_packet(vehicle), f"chan{channel}_raw") == 1500
    vehicle.release_yaw_control()
    message = last_packet(vehicle)
    assert getattr(message, f"chan{channel}_raw") == (0 if channel <= 8 else 65534)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 2, -2])
def test_invalid_yaw_input_is_never_transmitted(vehicle, value):
    with pytest.raises(ValueError):
        vehicle.send_yaw_input(value)
    vehicle.connection.write.assert_not_called()


def test_yaw_is_refused_outside_guided_and_close_releases_owned_channel(vehicle):
    parameters(vehicle)
    vehicle.send_yaw_input(0.1)
    vehicle.status.mode = DroneMode.QLAND
    with pytest.raises(RuntimeError, match="GUIDED"):
        vehicle.send_yaw_input(0.1)
    writer = vehicle.connection.write
    vehicle.close()
    packet = mavlink2.MAVLink(None).parse_char(writer.call_args.args[0])
    assert packet.get_type() == "RC_CHANNELS_OVERRIDE" and packet.chan4_raw == 0


def test_attitude_and_position_samples_have_monotonic_freshness(vehicle, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 123)
    vehicle._handle_message(mavlink2.MAVLink_attitude_message(0, 0.1, 0.2, 0.3, 0, 0, 0))
    vehicle._handle_message(mavlink2.MAVLink_global_position_int_message(0, 0, 0, 0, 0, 0, 0, 0, 300))
    assert (vehicle.status.roll_rad, vehicle.status.pitch_rad, vehicle.status.yaw_rad) == (0.1, 0.2, 0.3)
    assert vehicle.status.attitude_timestamp_s == vehicle.status.position_timestamp_s == 123


def test_mode_request_rechecks_pending_failsafe_heartbeat_before_sending(vehicle):
    heartbeat = mavlink2.MAVLink_heartbeat_message(
        mavlink2.MAV_TYPE_FIXED_WING,
        mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink2.MAV_MODE_FLAG_SAFETY_ARMED,
        21,
        mavlink2.MAV_STATE_CRITICAL,
        3,
    )
    heartbeat._header.srcSystem, heartbeat._header.srcComponent = 7, 1
    vehicle.connection.recv_match.side_effect = [heartbeat, None]
    assert not vehicle.request_mode(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    assert vehicle.status.mode is DroneMode.QRTL
    vehicle.connection.write.assert_not_called()


def test_hold_command_fixes_all_position_axes_without_forcing_mode(vehicle):
    target = HoldPosition(46.78, -71.27, 12.5)
    assert vehicle.request_position_hold(target)
    message = last_packet(vehicle)
    assert message.get_type() == "COMMAND_INT"
    assert message.command == mavlink2.MAV_CMD_DO_REPOSITION
    assert message.frame == mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert (message.target_system, message.target_component) == (7, 1)
    assert (message.x, message.y, message.z) == (467800000, -712700000, 12.5)
    assert message.param2 == 0  # Reject if another mode has taken control.
    assert vehicle.position_hold_accepted() is None


@pytest.mark.parametrize(
    "result,accepted",
    [
        (mavlink2.MAV_RESULT_ACCEPTED, True),
        (mavlink2.MAV_RESULT_DENIED, False),
        (mavlink2.MAV_RESULT_IN_PROGRESS, None),
    ],
)
def test_hold_acceptance_requires_ack_and_resets_for_next_command(vehicle, result, accepted):
    target = HoldPosition(46.78, -71.27, 12.5)
    ack = mavlink2.MAVLink_command_ack_message(mavlink2.MAV_CMD_DO_REPOSITION, result)
    vehicle._handle_message(ack)
    assert vehicle.position_hold_accepted() is None
    vehicle.request_position_hold(target)
    vehicle._handle_message(ack)
    assert vehicle.position_hold_accepted() is accepted
    vehicle.request_position_hold(target)
    assert vehicle.position_hold_accepted() is None


def test_hold_rejects_other_recipient_ack_and_other_flight_mode(vehicle):
    target = HoldPosition(46.78, -71.27, 12.5)
    vehicle.request_position_hold(target)
    ack = mavlink2.MAVLink_command_ack_message(
        mavlink2.MAV_CMD_DO_REPOSITION,
        mavlink2.MAV_RESULT_ACCEPTED,
        target_system=42,
        target_component=42,
    )
    vehicle._handle_message(ack)
    assert vehicle.position_hold_accepted() is None
    vehicle.status.mode = DroneMode.QRTL
    vehicle.connection.write.reset_mock()
    assert not vehicle.request_position_hold(target)
    vehicle.connection.write.assert_not_called()


@pytest.mark.parametrize("values", [(float("nan"), 0, 5), (91, 0, 5), (0, 181, 5), (0, 0, float("inf"))])
def test_invalid_hold_coordinates_are_rejected(values):
    with pytest.raises(ValueError):
        HoldPosition(*values)
