from vtol.domain.drone import DroneMode, LandedState
from vtol.domain.mission.execution_status import MissionExecutionStatus


def test_quadplane_modes_are_recognized_and_other_modes_remain_unknown():
    assert DroneMode.from_str("QHOVER") is DroneMode.QHOVER
    assert DroneMode.from_str("qloiter") is DroneMode.QLOITER
    assert DroneMode.from_str("QLAND") is DroneMode.QLAND
    assert DroneMode.from_str("unmapped-flight-mode") is DroneMode.UNKNOWN


def test_landed_state_maps_mavlink_values():
    assert LandedState.from_mavlink(1) is LandedState.ON_GROUND
    assert LandedState.from_mavlink(2) is LandedState.IN_AIR
    assert LandedState.from_mavlink(3) is LandedState.TAKEOFF
    assert LandedState.from_mavlink(4) is LandedState.LANDING
    assert LandedState.from_mavlink(0) is LandedState.UNKNOWN


def test_mission_execution_observation_exposes_freshness_without_conflating_sequences():
    status = MissionExecutionStatus(
        current_item_seq=9,
        last_reached_item_seq=8,
        last_updated_monotonic_s=10.0,
        mission_id=2,
    )
    assert status.is_fresh(max_age_s=2.0, now_s=11.0)
    assert not status.is_fresh(max_age_s=2.0, now_s=13.0)
    assert not MissionExecutionStatus().is_fresh()
