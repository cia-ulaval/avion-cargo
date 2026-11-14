import math
from dataclasses import replace
from unittest.mock import Mock

import pytest

from application.yaw_alignment_supervisor import AlignmentState, YawAlignmentSupervisor
from domain.drone import Drone, DroneMode
from domain.rigid_transform import RigidTransform
from domain.tracking import TrackingResult
from domain.yaw_alignment import YawAlignmentConfig


@pytest.fixture
def scenario(monkeypatch, sample_drone_status):
    clock = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    drone = Mock(spec=Drone)
    drone.prepare_yaw_control.return_value = True
    supervisor = YawAlignmentSupervisor(drone, YawAlignmentConfig(enabled=True))

    def step(now=100.0, error=30, mode=DroneMode.QLAND, connected=True, armed=True, **kwargs):
        clock[0] = now
        status = replace(
            sample_drone_status(),
            mode=mode,
            armed=armed,
            groundspeed_mps=0,
            speed=0,
            roll_rad=0,
            pitch_rad=0,
            yaw_rad=0,
            attitude_timestamp_s=now,
            position_timestamp_s=now,
            last_heartbeat_monotonic_s=now if connected else now - 10,
            **kwargs,
        )
        tracking = None
        if error is not None:
            transform = RigidTransform.from_vectors([0, 0, math.radians(error)], [0, 0, 5])
            tracking = TrackingResult.detected(
                transform.position, 29, transform.position, captured_at_s=now, body_from_marker=transform
            )
        supervisor.step(status, tracking, now)
        return status, tracking

    return supervisor, drone, step


def test_alignment_waits_for_mode_confirmation_then_returns_after_stable_heading(scenario):
    supervisor, drone, step = scenario
    step()
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))
    step(100.2)
    drone.send_yaw_input.assert_not_called()
    step(100.3, mode=DroneMode.GUIDED)
    step(100.4, mode=DroneMode.GUIDED)
    assert 0 < drone.send_yaw_input.call_args.args[0] <= 0.25
    step(100.5, error=2, mode=DroneMode.GUIDED)
    step(101.6, error=2, mode=DroneMode.GUIDED)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    step(101.7, error=2)
    assert supervisor.state is AlignmentState.COOLDOWN
    step(102, error=30)
    assert drone.request_mode.call_count == 2


def test_target_lost_during_pending_guided_request_cancels_when_guided_arrives(scenario):
    supervisor, drone, step = scenario
    step()
    step(100.1, error=None)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    step(100.2, error=None, mode=DroneMode.GUIDED)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    drone.send_yaw_input.assert_not_called()
    step(100.3, error=None)
    assert supervisor.state is AlignmentState.FAILED


@pytest.mark.parametrize("mode", [DroneMode.QRTL, DroneMode.MANUAL, DroneMode.QLOITER])
def test_external_mode_change_is_never_overwritten(scenario, mode):
    supervisor, drone, step = scenario
    step()
    step(100.1, mode=DroneMode.GUIDED)
    status, _ = step(100.2, mode=mode)
    supervisor.stop(status)
    assert supervisor.state is AlignmentState.SUSPENDED
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))
    step(101, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.SUSPENDED


def test_missing_mode_confirmation_cancels_request_and_requires_new_heartbeat(scenario):
    supervisor, drone, step = scenario
    step()
    status, tracking = step(106)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    supervisor.step(status, tracking, 106.1)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    step(106.2)
    assert supervisor.state is AlignmentState.FAILED


def test_preparation_requests_attitude_even_before_first_attitude_sample(scenario):
    supervisor, drone, step = scenario
    drone.prepare_yaw_control.return_value = False
    status, tracking = step()
    status.attitude_timestamp_s = None
    supervisor.step(status, tracking, 100.1)
    assert supervisor.state is AlignmentState.PREPARING
    assert drone.prepare_yaw_control.call_count == 2
    drone.request_mode.assert_not_called()


def test_disconnect_preserves_session_ownership_and_returns_on_reconnection(scenario):
    supervisor, drone, step = scenario
    step()
    step(100.1, mode=DroneMode.GUIDED)
    step(100.2, mode=DroneMode.GUIDED, connected=False)
    step(100.3, mode=DroneMode.GUIDED)
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    assert supervisor.state is AlignmentState.WAIT_RETURN


def test_failed_preflight_does_not_change_flight_mode(scenario):
    supervisor, drone, step = scenario
    drone.prepare_yaw_control.side_effect = ValueError("Q_GUIDED_MODE must be enabled")
    step()
    assert supervisor.state is AlignmentState.FAILED
    drone.request_mode.assert_not_called()


def test_alignment_timeout_and_shutdown_release_yaw_and_request_original_mode(scenario):
    supervisor, drone, step = scenario
    step()
    step(100.1, mode=DroneMode.GUIDED)
    status, _ = step(116, mode=DroneMode.GUIDED)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    supervisor.stop(status)
    drone.release_yaw_control.assert_called()
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))


def test_low_altitude_and_small_error_do_not_trigger_mode_changes(scenario):
    supervisor, drone, step = scenario
    status, tracking = step(error=2)
    assert supervisor.state is AlignmentState.MONITORING
    transform = RigidTransform.from_vectors([0, 0, 1], [0, 0, 1])
    tracking = replace(tracking, body_from_marker=transform)
    supervisor.step(status, tracking, 100)
    drone.request_mode.assert_not_called()


def test_reusing_one_frame_cannot_confirm_a_stable_alignment(scenario, monkeypatch):
    supervisor, drone, step = scenario
    supervisor.config = replace(supervisor.config, max_age_s=3.0)
    step()
    step(100.1, mode=DroneMode.GUIDED)
    status, tracking = step(100.2, error=2, mode=DroneMode.GUIDED)
    monkeypatch.setattr("time.monotonic", lambda: 101.4)
    status = replace(status, attitude_timestamp_s=101.4, position_timestamp_s=101.4, last_heartbeat_monotonic_s=101.4)
    supervisor.step(status, tracking, 101.4)
    assert supervisor.state is AlignmentState.ALIGNING
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))
