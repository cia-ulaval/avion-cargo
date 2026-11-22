from vtol.domain.drone import DroneMode, LandedState, VtolState
from vtol.domain.mission.execution_status import MissionExecutionStatus
import time


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


def test_vtol_state_maps_mavlink_values():
    assert VtolState.from_mavlink(0) is VtolState.UNDEFINED
    assert VtolState.from_mavlink(1) is VtolState.TRANSITION_TO_FIXED_WING
    assert VtolState.from_mavlink(2) is VtolState.TRANSITION_TO_MULTICOPTER
    assert VtolState.from_mavlink(3) is VtolState.MULTICOPTER
    assert VtolState.from_mavlink(4) is VtolState.FIXED_WING
    assert VtolState.from_mavlink(5) is VtolState.UNKNOWN
    assert VtolState.from_mavlink(None) is VtolState.UNKNOWN


def test_mission_execution_observation_exposes_freshness_without_conflating_sequences():
    status = MissionExecutionStatus(
        current_item_seq=9,
        last_reached_item_seq=8,
        last_updated_monotonic_s=10.0,
        mission_id=2,
    )
    assert _is_fresh(status, max_age_s=2.0, now_s=11.0)
    assert not _is_fresh(status, max_age_s=2.0, now_s=13.0)
    assert not _is_fresh(MissionExecutionStatus())


def _is_fresh(status: MissionExecutionStatus, max_age_s: float = 3.0, *, now_s: float | None = None) -> bool:
    if max_age_s <= 0:
        raise ValueError("max_age_s must be greater than zero")
    if status.last_updated_monotonic_s is None:
        return False
    now = time.monotonic() if now_s is None else now_s
    age = now - status.last_updated_monotonic_s
    return 0 <= age <= max_age_s
