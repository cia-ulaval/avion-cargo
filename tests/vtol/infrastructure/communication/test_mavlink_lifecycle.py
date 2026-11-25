from collections import deque
from dataclasses import replace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from vtol.domain.drone import DroneMode, VtolState
from vtol.domain.rigid_transform import RigidTransform
from vtol.infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


class MemoryConnection(mavutil.mavfile):
    def __init__(self, clock):
        super().__init__(None, "memory", source_component=191, input=False)
        self.clock = clock
        self.incoming = deque()
        self.outgoing = []
        self.receive_calls = []
        self.close_count = 0

    def recv_match(self, **kwargs):
        self.receive_calls.append(kwargs)
        if self.incoming:
            return self.incoming.popleft()
        if kwargs.get("blocking"):
            self.clock.now += kwargs["timeout"]
        return None

    def write(self, packet):
        self.outgoing.append(mavlink2.MAVLink(None).parse_char(packet))

    def close(self):
        self.close_count += 1


def sourced(message, system=1, component=1):
    message._header.srcSystem = system
    message._header.srcComponent = component
    return message


def heartbeat(
    mode=9,
    system=1,
    component=1,
    autopilot=mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
    vehicle_type=mavlink2.MAV_TYPE_QUADROTOR,
):
    return sourced(
        mavlink2.MAVLink_heartbeat_message(
            vehicle_type,
            autopilot,
            mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
            mavlink2.MAV_STATE_ACTIVE,
            3,
        ),
        system,
        component,
    )


@pytest.fixture
def link(monkeypatch):
    clock = Clock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    connection = MemoryConnection(clock)
    drone = DroneMavlinkUDPConnector(MavlinkConnectionParams("127.0.0.1", 14550, timeout=0.5))
    monkeypatch.setattr(drone, "_init_mavlink_connection", lambda: setattr(drone, "connection", connection))
    return drone, connection, clock


@pytest.mark.parametrize("fail_on_timeout", [True, False])
def test_connect_accepts_only_autopilot_and_initializes_status_from_first_heartbeat(link, fail_on_timeout):
    drone, connection, _ = link
    drone.parameters = replace(drone.parameters, fail_on_timeout=fail_on_timeout)
    connection.incoming.extend([heartbeat(system=42, autopilot=mavlink2.MAV_AUTOPILOT_INVALID), heartbeat(system=7)])
    drone.connect()
    assert drone.status.connected
    assert drone.status.mode is DroneMode.LAND
    assert (connection.target_system, connection.target_component) == (7, 1)
    # No additional blocking read once the autopilot heartbeat is accepted.
    assert len(connection.receive_calls) == 2
    requests = [message for message in connection.outgoing if message.get_type() == "COMMAND_LONG"]
    assert {int(message.param1): int(message.param2) for message in requests} == {
        mavlink2.MAVLINK_MSG_ID_VFR_HUD: 200_000,
        mavlink2.MAVLINK_MSG_ID_SYS_STATUS: 1_000_000,
        mavlink2.MAVLINK_MSG_ID_GPS_RAW_INT: 500_000,
        mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 100_000,
        mavlink2.MAVLINK_MSG_ID_MISSION_CURRENT: 1_000_000,
        mavlink2.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 500_000,
    }
    assert all((message.target_system, message.target_component) == (7, 1) for message in requests)


def test_fixed_wing_heartbeat_maps_manual_mode_from_pymavlink(link):
    drone, connection, _ = link
    connection.incoming.append(
        heartbeat(mode=0, vehicle_type=mavlink2.MAV_TYPE_FIXED_WING)
    )

    drone.connect()

    assert drone.status.mode_name == "MANUAL"
    assert drone.status.mode is DroneMode.MANUAL


def test_missing_heartbeat_times_out_and_closes_transport(link):
    drone, connection, clock = link
    with pytest.raises(TimeoutError, match="No autopilot heartbeat"):
        drone.connect()
    assert clock.now == pytest.approx(100.5)
    assert all(0 < call["timeout"] <= 0.2 for call in connection.receive_calls)
    assert connection.close_count == 1
    assert drone.connection is None
    assert not drone.status.connected


def test_optional_timeout_keeps_transport_open_and_status_disconnected(link):
    drone, connection, clock = link
    drone.parameters = replace(drone.parameters, fail_on_timeout=False)

    drone.connect()

    assert clock.now == pytest.approx(100.5)
    assert drone.connection is connection
    assert connection.close_count == 0
    assert not drone.status.connected
    assert drone.status.mode is DroneMode.UNKNOWN
    assert [message.get_type() for message in connection.outgoing] == ["HEARTBEAT"]
    connection.receive_calls.clear()
    clock.now += 20
    assert not drone.get_status().connected
    assert all(call == {"blocking": False} for call in connection.receive_calls)
    drone.close()
    assert connection.close_count == 1


def test_optional_link_discovers_late_autopilot_and_requests_streams_once(link):
    drone, connection, _ = link
    drone.parameters = replace(drone.parameters, fail_on_timeout=False)
    drone.connect()
    connection.incoming.extend(
        [
            sourced(mavlink2.MAVLink_vfr_hud_message(0, 12, 0, 0, 42, 0), system=7),
            heartbeat(system=42, autopilot=mavlink2.MAV_AUTOPILOT_INVALID),
            heartbeat(system=0),
        ]
    )
    assert not drone.get_status().connected

    connection.incoming.extend(
        [heartbeat(system=7), sourced(mavlink2.MAVLink_vfr_hud_message(0, 12, 0, 0, 42, 0), system=7)]
    )
    status = drone.get_status()

    assert status.connected
    assert status.mode is DroneMode.LAND
    assert status.alt_m == 42
    assert (connection.target_system, connection.target_component) == (7, 1)
    requests = [message for message in connection.outgoing if message.get_type() == "COMMAND_LONG"]
    assert len(requests) == 6
    assert all((message.target_system, message.target_component) == (7, 1) for message in requests)
    connection.incoming.extend([heartbeat(system=8, mode=4), heartbeat(system=7)])
    assert drone.get_status().mode is DroneMode.LAND
    requests_before = [message for message in connection.outgoing if message.get_type() == "COMMAND_LONG"]
    assert len(requests_before) == 6

    connection.incoming.extend([heartbeat(system=8, mode=4), heartbeat(system=7)])

    assert drone.get_status().mode is DroneMode.LAND

    requests_after = [message for message in connection.outgoing if message.get_type() == "COMMAND_LONG"]
    assert len(requests_after) == len(requests_before)


def test_optional_timeout_does_not_suppress_transport_errors(link, monkeypatch):
    drone, connection, _ = link
    drone.parameters = replace(drone.parameters, fail_on_timeout=False)

    def fail_to_open():
        raise OSError("transport unavailable")

    monkeypatch.setattr(drone, "_init_mavlink_connection", fail_to_open)
    with pytest.raises(OSError, match="transport unavailable"):
        drone.connect()
    assert drone.connection is None


def test_status_poll_does_not_wait_for_traffic_and_maintains_heartbeat(link):
    drone, connection, clock = link
    connection.incoming.append(heartbeat())
    drone.connect()
    connection.receive_calls.clear()
    clock.now += 1.1
    snapshot = drone.get_status()
    assert all(call == {"blocking": False} for call in connection.receive_calls)
    assert len([m for m in connection.outgoing if m.get_type() == "HEARTBEAT"]) == 2
    drone.status.alt_m = 42
    assert snapshot.alt_m == 0
    clock.now += 2
    assert not snapshot.connected
    assert not drone.get_status().connected


def test_other_systems_and_components_do_not_refresh_status(link):
    drone, connection, clock = link
    connection.incoming.append(heartbeat())
    drone.connect()
    clock.now += 4
    connection.incoming.extend([heartbeat(mode=4, system=2), heartbeat(mode=4, component=100)])
    status = drone.get_status()
    assert not status.connected
    assert status.mode is DroneMode.LAND
    connection.incoming.append(heartbeat(mode=4))
    status = drone.get_status()
    assert status.connected
    assert status.mode is DroneMode.GUIDED


def test_receive_drain_is_bounded_even_when_traffic_never_stops(link):
    drone, connection, _ = link
    connection.incoming.append(heartbeat())
    drone.connect()
    connection.receive_calls.clear()
    connection.incoming.extend(heartbeat(system=2) for _ in range(1000))
    drone.get_status()
    assert len(connection.receive_calls) == drone.MAX_MESSAGES_PER_POLL
    assert len(connection.incoming) == 900


def test_close_is_idempotent_and_invalidates_link_status(link):
    drone, connection, _ = link
    connection.incoming.append(heartbeat())
    drone.connect()
    drone.close()
    drone.close()
    assert connection.close_count == 1
    assert not drone.status.connected
    with pytest.raises(RuntimeError, match="not connected"):
        drone.get_status()


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_connection_timeout_must_be_positive_and_finite(timeout):
    with pytest.raises(ValueError, match="timeout"):
        MavlinkConnectionParams("127.0.0.1", timeout=timeout)


@pytest.mark.parametrize(
    "vehicle_type,mode",
    [
        (mavlink2.MAV_TYPE_QUADROTOR, 0),
        (mavlink2.MAV_TYPE_QUADROTOR, 4),
        (mavlink2.MAV_TYPE_QUADROTOR, 9),
        (mavlink2.MAV_TYPE_FIXED_WING, 15),
        (mavlink2.MAV_TYPE_FIXED_WING, 19),
        (mavlink2.MAV_TYPE_FIXED_WING, 20),
    ],
)
def test_measurements_do_not_issue_flight_commands_regardless_of_observed_mode(link, vehicle_type, mode):
    drone, connection, clock = link
    connection.incoming.append(heartbeat(mode=mode, vehicle_type=vehicle_type))
    drone.connect()
    drone.get_status()
    transform = RigidTransform.from_vectors([0, 0, 0.4], [0.1, 0.2, 3])
    drone.send_landing_target(transform.position, (0.4, 0.4), body_from_marker=transform, captured_at_s=clock.now)
    drone.close()

    # Only telemetry subscriptions may use the command protocol.
    assert {message.get_type() for message in connection.outgoing} == {"HEARTBEAT", "COMMAND_LONG", "LANDING_TARGET"}
    commands = [message.command for message in connection.outgoing if message.get_type() == "COMMAND_LONG"]
    assert commands and set(commands) == {mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL}


def test_mission_progress_and_landed_state_are_observed_separately(link):
    from types import SimpleNamespace

    from vtol.domain.drone import LandedState

    drone, connection, _ = link
    connection.incoming.append(heartbeat())
    drone.connect()

    drone._handle_message(SimpleNamespace(get_type=lambda: "MISSION_CURRENT", seq=12, mission_id=71))
    status = drone.get_mission_execution_status()
    assert status.current_item_seq == 12
    assert status.last_reached_item_seq is None
    assert status.mission_id == 71

    drone._handle_message(SimpleNamespace(get_type=lambda: "MISSION_ITEM_REACHED", seq=11))
    status = drone.get_mission_execution_status()
    assert status.current_item_seq == 12
    assert status.last_reached_item_seq == 11

    drone._handle_message(SimpleNamespace(get_type=lambda: "MISSION_CURRENT", seq=0, mission_id=72))
    status = drone.get_mission_execution_status()
    assert status.current_item_seq == 0
    assert status.last_reached_item_seq is None
    assert status.mission_id == 72

    drone._handle_message(SimpleNamespace(get_type=lambda: "EXTENDED_SYS_STATE", vtol_state=3, landed_state=1))
    assert drone.status.landed_state is LandedState.ON_GROUND
    assert drone.status.vtol_state is VtolState.MULTICOPTER


def test_mission_observations_are_invalidated_when_heartbeat_expires(link):
    from types import SimpleNamespace

    drone, connection, clock = link
    connection.incoming.append(heartbeat())
    drone.connect()
    drone._handle_message(SimpleNamespace(get_type=lambda: "MISSION_CURRENT", seq=8, mission_id=5))
    drone._handle_message(SimpleNamespace(get_type=lambda: "EXTENDED_SYS_STATE", vtol_state=4, landed_state=2))
    assert drone.get_mission_execution_status().current_item_seq == 8
    assert drone.status.vtol_state is VtolState.FIXED_WING

    clock.now += 4.0
    assert not drone.get_status().connected
    assert drone.get_mission_execution_status().current_item_seq is None
    assert drone.status.vtol_state is VtolState.UNKNOWN


def test_close_resets_mission_execution_status(link):
    from types import SimpleNamespace

    drone, connection, _ = link
    connection.incoming.append(heartbeat())
    drone.connect()
    drone._handle_message(SimpleNamespace(get_type=lambda: "MISSION_CURRENT", seq=3, mission_id=4))
    drone._handle_message(SimpleNamespace(get_type=lambda: "EXTENDED_SYS_STATE", vtol_state=3, landed_state=1))
    drone.close()
    assert drone.get_mission_execution_status().current_item_seq is None
    assert drone.status.vtol_state is VtolState.UNKNOWN
