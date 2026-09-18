from collections import deque

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from domain.drone import DroneMode
from infrastructure.communication.mavlink import DroneMavlinkUDPConnector, MavlinkConnectionParams


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


def heartbeat(mode=9, system=1, component=1, autopilot=mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA):
    return sourced(
        mavlink2.MAVLink_heartbeat_message(
            mavlink2.MAV_TYPE_QUADROTOR,
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


def test_connect_accepts_only_autopilot_and_initializes_status_from_first_heartbeat(link):
    drone, connection, _ = link
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
    }
    assert all((message.target_system, message.target_component) == (7, 1) for message in requests)


def test_missing_heartbeat_times_out_and_closes_transport(link):
    drone, connection, clock = link
    with pytest.raises(TimeoutError, match="No autopilot heartbeat"):
        drone.connect()
    assert clock.now == pytest.approx(100.5)
    assert all(0 < call["timeout"] <= 0.2 for call in connection.receive_calls)
    assert connection.close_count == 1
    assert drone.connection is None
    assert not drone.status.connected


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
