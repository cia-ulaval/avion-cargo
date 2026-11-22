import asyncio

from vtol.application.flight_controller_runtime import FlightControllerObservationRuntime
from vtol.application.mission.observation_store import MissionObservationStore
from vtol.domain.drone import DroneMode, DroneStatus
from vtol.domain.mission.execution_status import MissionExecutionStatus


class FakeDrone:
    def __init__(self):
        self.connected = False
        self.closed = False
        self.poll_count = 0
        self.mission_status = MissionExecutionStatus(current_item_seq=4, last_reached_item_seq=3)
        self.status = DroneStatus(
            mode=DroneMode.QHOVER,
            alt_m=1.0,
            groundspeed_mps=0.0,
            battery_voltage_v=15.0,
            battery_remaining_pct=90,
            gps_fix_type=3,
            armed=True,
            last_heartbeat_s=1.0,
            last_signal_gpio_s=0.0,
            speed=0.0,
            relative_altitude=1.0,
            latitude=46.0,
            longitude=-71.0,
            relative_altitude_ms=1.0,
            heading_deg=0.0,
        )

    def connect(self):
        self.connected = True

    def get_status(self):
        self.poll_count += 1
        return self.status

    def get_mission_execution_status(self):
        return self.mission_status

    def close(self):
        self.closed = True


def test_runtime_owns_one_connection_and_publishes_mission_observations():
    async def scenario():
        drone = FakeDrone()
        observations = MissionObservationStore()
        runtime = FlightControllerObservationRuntime(drone, observations, poll_interval_s=0.005)
        stop_event = asyncio.Event()
        task = asyncio.create_task(runtime.run(stop_event))
        while drone.poll_count == 0:
            await asyncio.sleep(0)
        stop_event.set()
        await asyncio.wait_for(task, timeout=1)

        assert drone.connected
        assert drone.closed
        assert drone.poll_count >= 1
        snapshot = observations.capture_mission_snapshot()
        assert snapshot.drone_status == drone.status
        assert snapshot.drone_status is not drone.status
        assert snapshot.mission_execution_status == drone.mission_status

    asyncio.run(scenario())
