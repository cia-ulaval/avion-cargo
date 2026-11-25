from shared.contracts.geometry import Position, PositionEstimate
from shared.contracts.messages import PayloadPositionUpdated
from vtol.application.mission.observation_store import MissionObservationStore
from vtol.domain.drone import DroneMode, DroneStatus
from vtol.domain.mission.execution_status import MissionExecutionStatus


def make_drone_status():
    return DroneStatus(
        mode=DroneMode.QHOVER,
        alt_m=5.0,
        groundspeed_mps=0.0,
        battery_voltage_v=15.0,
        battery_remaining_pct=80,
        gps_fix_type=3,
        armed=True,
        last_heartbeat_s=1.0,
        last_signal_gpio_s=0.0,
        speed=0.0,
        relative_altitude=5.0,
        latitude=46.0,
        longitude=-71.0,
        relative_altitude_ms=5.0,
        heading_deg=90.0,
    )


def test_snapshot_contains_latest_flight_status_and_payload_events_without_aliasing():
    store = MissionObservationStore()
    status = make_drone_status()
    mission_status = MissionExecutionStatus(current_item_seq=8, last_reached_item_seq=7)
    store.set_flight_observation(status, mission_status)
    status.alt_m = 99.0

    event = PayloadPositionUpdated(
        estimate=PositionEstimate(position=Position(1.0, 2.0, 0.0), residual_m=0.1, anchor_ids=("a", "b"), sequence=1)
    )
    store.record_payload_event("payload-1", event)

    snapshot = store.capture_mission_snapshot()
    assert snapshot.drone_status.alt_m == 5.0
    assert snapshot.mission_execution_status == mission_status
    assert snapshot.payload_observations[0].payload_id == "payload-1"
    assert snapshot.payload_observations[0].event is event
