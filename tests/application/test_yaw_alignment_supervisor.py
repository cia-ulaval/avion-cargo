import csv
import math
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import pytest

from application.yaw_alignment_supervisor import AlignmentState, YawAlignmentSupervisor
from domain.drone import Drone, DroneMode, HoldPosition
from domain.rigid_transform import RigidTransform
from domain.tracking import TrackingResult
from domain.yaw_alignment import YawAlignmentConfig


@pytest.fixture
def scenario(monkeypatch, sample_drone_status):
    clock = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    drone = Mock(spec=Drone)
    drone.prepare_yaw_control.return_value = True
    drone.request_position_hold.return_value = True
    drone.position_hold_accepted.return_value = True
    supervisor = YawAlignmentSupervisor(drone, YawAlignmentConfig(enabled=True))
    last_frame = [98.8]

    def observe(tracking, now):
        supervisor.observe_tracking(tracking, now)

    # Most scenarios start with an already visible target. Camera frames arrive
    # between supervision ticks, independently of the position/attitude samples.
    transform = RigidTransform.from_vectors([0, 0, math.radians(30)], [0, 0, 5])
    for index in range(30):
        captured = 98.8 + index * 0.04
        observe(
            TrackingResult.detected(transform.position, 29, captured_at_s=captured, body_from_marker=transform),
            captured,
        )
        last_frame[0] = captured

    def step(
        now=100.0,
        error=30,
        mode=DroneMode.GUIDED,
        connected=True,
        captured_at=None,
        not_found=False,
        frames=True,
        marker_position=(0, 0, 5),
        marker_id=29,
        target_identity=None,
        **kwargs,
    ):
        clock[0] = now
        status = replace(
            sample_drone_status(),
            mode=mode,
            armed=True,
            groundspeed_mps=0,
            speed=0,
            roll_rad=0,
            pitch_rad=0,
            yaw_rad=0,
            attitude_timestamp_s=now,
            position_timestamp_s=now,
            last_heartbeat_monotonic_s=now if connected else now - 10,
        )
        status = replace(status, **kwargs)
        tracking = TrackingResult.not_found(captured_at_s=now) if not_found else None
        if error is not None:
            transform = RigidTransform.from_vectors([0, 0, math.radians(error)], marker_position)
            tracking = TrackingResult.detected(
                transform.position,
                marker_id,
                transform.position,
                captured_at_s=now if captured_at is None else captured_at,
                body_from_marker=transform,
            )
            if target_identity is not None:
                tracking = replace(tracking, target_identity=target_identity)
        if frames and captured_at is None:
            while last_frame[0] + 0.04 < now:
                last_frame[0] += 0.04
                frame = replace(tracking, captured_at_s=last_frame[0]) if tracking else None
                observe(frame, last_frame[0])
        last_frame[0] = now
        supervisor.step(status, tracking, now)
        return status, tracking

    def align():
        step(mode=DroneMode.QLAND)
        step(100.1)
        step(100.2)
        step(100.8)
        assert supervisor.state is AlignmentState.ALIGNING

    return supervisor, drone, step, align


def test_stops_at_captured_position_and_altitude_before_yaw_then_lands_only_after_stable_heading(scenario):
    supervisor, drone, step, _ = scenario
    status, _ = step(mode=DroneMode.QLAND)
    target = HoldPosition(status.latitude, status.longitude, status.relative_altitude)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))
    step(100.1, mode=DroneMode.QLAND)
    drone.send_yaw_input.assert_not_called()
    step(100.2, relative_altitude=target.relative_altitude_m - 0.5, speed=1.0)
    drone.request_position_hold.assert_called_once_with(target)
    assert supervisor.state is AlignmentState.HOLDING
    step(100.3, speed=0.6)
    step(100.4)
    step(100.8)
    assert all(call.args == (0.0,) for call in drone.send_yaw_input.call_args_list)
    step(101)
    assert supervisor.state is AlignmentState.ALIGNING
    assert 0 < drone.send_yaw_input.call_args.args[0] <= 0.25
    step(101.1, error=2)
    step(102.2, error=2)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    step(102.3, error=2, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.COOLDOWN
    assert supervisor.to_dict()["hold_position"]["relative_altitude_m"] == target.relative_altitude_m


@pytest.mark.parametrize("ack,reason", [(None, "hold_ack_timeout"), (False, "hold_command_rejected")])
def test_hold_ack_missing_or_rejected_never_authorizes_yaw_or_landing(scenario, ack, reason):
    supervisor, drone, step, _ = scenario
    drone.position_hold_accepted.return_value = ack
    step(mode=DroneMode.QLAND)
    step(100.1)
    step(111)
    assert supervisor.state is AlignmentState.FAILED
    assert supervisor.reason == reason
    assert all(call.args == (0.0,) for call in drone.send_yaw_input.call_args_list)
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))


@pytest.mark.parametrize("not_found", [False, True])
def test_brief_loss_neutralizes_yaw_and_resumes_in_same_hover(scenario, not_found):
    supervisor, drone, step, align = scenario
    align()
    step(100.9, error=None, not_found=not_found)
    assert supervisor.state is AlignmentState.ALIGNING
    drone.send_yaw_input.assert_called_with(0.0)
    step(101)
    drone.send_yaw_input.assert_called_with(0.0)
    step(102.1)
    assert drone.send_yaw_input.call_args.args[0] > 0
    assert supervisor.state is AlignmentState.ALIGNING
    assert drone.request_mode.call_count == drone.request_position_hold.call_count == 1


def test_prolonged_target_loss_waits_in_hover_then_resumes_without_landing(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(101.2, error=None, not_found=True)
    assert supervisor.state is AlignmentState.WAIT_TARGET
    drone.send_yaw_input.assert_called_with(0.0)
    step(102)
    assert supervisor.state is AlignmentState.WAIT_TARGET
    step(102.3)
    assert supervisor.state is AlignmentState.ALIGNING
    assert drone.send_yaw_input.call_args.args[0] > 0
    assert drone.request_mode.call_count == drone.request_position_hold.call_count == 1


def test_target_loss_during_mode_transition_still_stops_descent(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND)
    step(100.4, error=None)
    assert supervisor.state is AlignmentState.HOLDING
    step(100.5, error=None)
    step(101.1, error=None)
    assert supervisor.state is AlignmentState.WAIT_TARGET
    assert drone.request_position_hold.call_count == 1
    assert drone.request_mode.call_count == 1


@pytest.mark.parametrize(
    "drift",
    [
        {"relative_altitude": 4.0},
        {"latitude": 46.78002},
        {"speed": 0.5},
        {"groundspeed_mps": 0.5},
        {"attitude_timestamp_s": 0},
        {"position_timestamp_s": 0},
        {"roll_rad": float("nan")},
    ],
)
def test_position_drift_or_invalid_telemetry_pauses_yaw_without_resuming_landing(scenario, drift):
    supervisor, drone, step, align = scenario
    align()
    target = drone.request_position_hold.call_args.args[0]
    step(101, **drift)
    assert supervisor.state is AlignmentState.HOLDING
    drone.send_yaw_input.assert_called_with(0.0)
    step(101.1)
    step(101.7)
    assert supervisor.state is AlignmentState.ALIGNING
    drone.request_position_hold.assert_called_once_with(target)
    assert drone.request_mode.call_count == 1


def test_alignment_timeout_resumes_landing_and_permits_retry(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(116)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    assert supervisor.reason == "alignment_timeout"
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    step(116.1, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.COOLDOWN
    step(117.2, mode=DroneMode.QLAND)
    step(117.3, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    assert supervisor.to_dict()["attempts"] == 2


def test_slow_stabilization_remains_recoverable_without_resuming_descent(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND)
    step(100.1)
    step(111, speed=1)
    assert supervisor.reason == "waiting_for_stable_hover"
    assert supervisor.state is AlignmentState.HOLDING
    drone.send_yaw_input.assert_called_with(0.0)
    step(111.1)
    step(111.7)
    assert supervisor.state is AlignmentState.ALIGNING
    assert drone.send_yaw_input.call_args.args[0] > 0
    assert drone.request_mode.call_count == 1


def test_recorded_quadplane_settling_eventually_starts_yaw(scenario):
    supervisor, drone, step, _ = scenario
    initial, _ = step(mode=DroneMode.QLAND)
    step(100.1)
    trace = Path(__file__).parents[1] / "fixtures" / "quadplane_hold_settling.csv"
    aligned_at = None
    with trace.open() as stream:
        samples = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    index = 0
    # The supervisor polls at 10 Hz while this flight's position stream is 4 Hz.
    # Preserve sample timestamps and the ticks between two incoming positions.
    for tick in range(2, 140):
        elapsed = tick / 10
        while index + 1 < len(samples) and samples[index + 1]["time_s"] <= elapsed:
            index += 1
        sample = samples[index]
        step(
            100.1 + elapsed,
            position_timestamp_s=100.1 + sample["time_s"],
            latitude=initial.latitude + math.degrees(sample["north_m"] / 6371000),
            longitude=initial.longitude
            + math.degrees(sample["east_m"] / (6371000 * math.cos(math.radians(initial.latitude)))),
            relative_altitude=initial.relative_altitude + sample["altitude_error_m"],
            groundspeed_mps=sample["speed_xy_mps"],
            speed=sample["speed_down_mps"],
        )
        assert supervisor.state is not AlignmentState.FAILED
        if supervisor.state is AlignmentState.ALIGNING:
            aligned_at = elapsed
            break
    assert aligned_at is not None and aligned_at > supervisor.config.hold_timeout_s
    assert drone.send_yaw_input.call_args.args[0] > 0
    assert drone.request_mode.call_count == drone.request_position_hold.call_count == 1


def test_guided_timeout_never_requests_landing_and_late_guided_keeps_hover(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND)
    step(106, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.FAILED
    assert supervisor.reason == "guided_mode_timeout"
    step(107)
    drone.request_position_hold.assert_called_once()
    drone.send_yaw_input.assert_called_with(0.0)
    assert drone.request_mode.call_count == 1


@pytest.mark.parametrize("mode", [DroneMode.QRTL, DroneMode.MANUAL, DroneMode.QLOITER, DroneMode.QLAND])
def test_external_mode_change_is_never_overwritten(scenario, mode):
    supervisor, drone, step, align = scenario
    align()
    status, _ = step(101, mode=mode)
    assert supervisor.state is AlignmentState.SUSPENDED
    supervisor.stop(status)
    assert drone.request_mode.call_count == 1


def test_disconnect_reconnection_and_shutdown_preserve_hover_without_landing(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(101, connected=False)
    status, _ = step(101.1)
    assert supervisor.state is AlignmentState.HOLDING
    supervisor.stop(status)
    drone.release_yaw_control.assert_called()
    assert drone.request_mode.call_count == 1


def test_missing_detection_restarts_heading_stability_confirmation(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(100.9, error=2)
    step(101, error=None)
    step(101.1, error=2)
    step(102, error=2)
    assert supervisor.state is AlignmentState.WAIT_TARGET
    step(102.2, error=2)
    assert supervisor.state is AlignmentState.ALIGNING
    step(103.3, error=2)
    assert supervisor.state is AlignmentState.WAIT_RETURN


def test_reusing_one_frame_cannot_confirm_heading(scenario):
    supervisor, drone, step, align = scenario
    supervisor.config = replace(supervisor.config, max_age_s=3)
    align()
    step(100.9, error=2)
    step(102, error=2, captured_at=100.9)
    assert supervisor.state is AlignmentState.ALIGNING
    assert drone.request_mode.call_count == 1


def test_reusing_one_position_sample_cannot_confirm_hover(scenario):
    supervisor, drone, step, _ = scenario
    supervisor.config = replace(supervisor.config, max_age_s=3)
    step(mode=DroneMode.QLAND)
    step(100.1)
    step(100.2)
    step(101, position_timestamp_s=100.2)
    assert supervisor.state is AlignmentState.HOLDING
    assert all(call.args == (0.0,) for call in drone.send_yaw_input.call_args_list)


def test_preparation_requests_attitude_before_first_sample(scenario):
    supervisor, drone, step, _ = scenario
    drone.prepare_yaw_control.return_value = False
    step(mode=DroneMode.QLAND, attitude_timestamp_s=None)
    assert supervisor.state is AlignmentState.PREPARING
    drone.prepare_yaw_control.assert_called_once()
    drone.request_mode.assert_not_called()


def test_incompatible_preflight_does_not_change_flight_mode(scenario):
    supervisor, drone, step, _ = scenario
    drone.prepare_yaw_control.side_effect = ValueError("Q_GUIDED_MODE must be enabled")
    step(mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.FAILED
    drone.request_mode.assert_not_called()


def test_small_heading_error_does_not_start_session(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND, error=2)
    assert supervisor.state is AlignmentState.MONITORING
    drone.request_mode.assert_not_called()


def test_disarming_clears_session_and_allows_new_alignment(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(101, armed=False)
    assert supervisor.state is AlignmentState.MONITORING
    assert supervisor.to_dict()["hold_position"] is None
    step(102, mode=DroneMode.QLAND)
    step(102.2, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.WAIT_GUIDED


def test_camera_flicker_between_supervision_ticks_does_not_interrupt_landing(scenario):
    supervisor, drone, step, _ = scenario
    step(99.99, armed=False, frames=False)
    for tick in range(31):
        now = 100 + tick / 10
        # Every supervision tick sees DETECTED, but the camera saw a loss in
        # between. Sampling only the latest frame would falsely qualify it.
        supervisor.observe_tracking(TrackingResult.not_found(captured_at_s=now - 0.04), now - 0.04)
        step(now, mode=DroneMode.QLAND, frames=False)
    drone.request_mode.assert_not_called()
    assert supervisor.to_dict()["attempts"] == 0
    step(103.9, mode=DroneMode.QLAND)
    drone.request_mode.assert_not_called()
    step(104.1, mode=DroneMode.QLAND)
    drone.request_mode.assert_called_once_with(DroneMode.GUIDED, expected_modes=(DroneMode.QLAND,))


def test_repeated_or_expired_frame_cannot_qualify_detection(scenario):
    supervisor, drone, step, _ = scenario
    step(99.99, armed=False, frames=False)
    for tick in range(21):
        step(100 + tick / 10, captured_at=100, mode=DroneMode.QLAND, frames=False)
    drone.request_mode.assert_not_called()
    assert supervisor.to_dict()["attempts"] == 0


@pytest.mark.parametrize("phase", ["WAIT_GUIDED", "HOLDING", "ALIGNING"])
def test_unstable_detection_resumes_landing_from_every_attempt_phase(scenario, phase):
    supervisor, drone, step, align = scenario
    if phase == "ALIGNING":
        align()
        last_stable = 100.8
    else:
        step(mode=DroneMode.QLAND)
        last_stable = 100
        if phase == "HOLDING":
            step(100.1)
            last_stable = 100.1
    step(last_stable + supervisor.config.detection_loss_timeout_s, error=None, not_found=True)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    assert supervisor.reason == "detection_unstable"
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))
    drone.release_yaw_control.assert_called()


def test_repeated_glimpses_do_not_extend_an_unstable_attempt(scenario):
    supervisor, drone, step, align = scenario
    align()
    for tick in range(1, 17):
        now = 100.8 + tick / 10
        supervisor.observe_tracking(TrackingResult.not_found(captured_at_s=now - 0.04), now - 0.04)
        step(now, frames=False)
        if supervisor.state is AlignmentState.WAIT_RETURN:
            break
        drone.send_yaw_input.assert_called_with(0.0)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    assert supervisor.reason == "detection_unstable"


def test_attempt_limit_survives_retries_and_resets_only_on_disarm(scenario):
    supervisor, drone, step, _ = scenario
    supervisor.config = replace(supervisor.config, max_attempts=2)
    step(mode=DroneMode.QLAND)
    step(100.1)
    step(101.6, error=None, not_found=True)
    step(101.7, mode=DroneMode.QLAND, error=None, not_found=True)
    assert supervisor.state is AlignmentState.COOLDOWN
    # The next attempt requires fresh continuity in QLAND, not pre-pause frames.
    step(102.8, mode=DroneMode.QLAND)
    step(102.9, mode=DroneMode.QLAND, relative_altitude=7)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    assert supervisor.to_dict()["attempts"] == 2
    step(103, relative_altitude=7)
    assert drone.request_position_hold.call_args.args[0].relative_altitude_m == 7
    step(104.5, error=None, not_found=True)
    step(104.6, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.EXHAUSTED
    assert supervisor.reason == "attempt_limit_reached"
    for now in (106, 110, 120):
        step(now, mode=DroneMode.QLAND)
    assert drone.request_mode.call_count == 4  # Two pauses and two returns.
    step(121, armed=False)
    assert supervisor.to_dict()["attempts"] == 0
    step(122.2, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    assert supervisor.to_dict()["attempts"] == 1


def test_successful_alignment_also_counts_toward_pause_limit(scenario):
    supervisor, drone, step, align = scenario
    supervisor.config = replace(supervisor.config, max_attempts=1)
    align()
    step(101, error=2)
    step(102.1, error=2)
    step(102.2, error=2, mode=DroneMode.QLAND)
    assert supervisor.state is AlignmentState.EXHAUSTED
    step(104, mode=DroneMode.QLAND)
    assert drone.request_mode.call_count == 2


def exhaust_retry_series(scenario):
    supervisor, drone, step, _ = scenario
    supervisor.config = replace(supervisor.config, max_attempts=1, retry_descent_m=2, min_height_m=4)
    step(mode=DroneMode.QLAND, relative_altitude=10, marker_position=(0, 0, 10))
    step(100.1, relative_altitude=10)
    step(101.6, error=None, not_found=True, relative_altitude=10)
    step(101.7, mode=DroneMode.QLAND, relative_altitude=10)
    assert supervisor.state is AlignmentState.EXHAUSTED
    assert supervisor.to_dict()["retry_below_altitude_m"] == 8
    assert drone.request_mode.call_count == 2
    return supervisor, drone, step


@pytest.mark.parametrize(
    "conditions",
    [
        {"relative_altitude": 8.01},
        {"relative_altitude": 12},
        {"relative_altitude": float("nan")},
        {"marker_position": (0, 0, 3.99)},
        {"marker_position": (3, 0, 8)},
        {"mode": DroneMode.QLOITER},
        {"position_timestamp_s": 100},
        {"attitude_timestamp_s": 100},
        {"captured_at": 100, "frames": False},
        {"error": 2},
        {"error": None, "not_found": True},
    ],
)
def test_exhausted_requires_descent_and_all_alignment_conditions(scenario, conditions):
    supervisor, drone, step = exhaust_retry_series(scenario)
    inputs = dict(mode=DroneMode.QLAND, relative_altitude=8, marker_position=(0, 0, 8))
    inputs.update(conditions)
    step(103, **inputs)
    assert supervisor.state is AlignmentState.EXHAUSTED
    assert drone.request_mode.call_count == 2


def test_exhausted_retries_lower_only_after_new_stable_detection(scenario):
    supervisor, drone, step = exhaust_retry_series(scenario)
    step(103, mode=DroneMode.QLAND, relative_altitude=8, error=None, not_found=True)
    step(103.1, mode=DroneMode.QLAND, relative_altitude=8, marker_position=(0, 0, 8))
    assert supervisor.state is AlignmentState.EXHAUSTED
    step(104.2, mode=DroneMode.QLAND, relative_altitude=8, marker_position=(0, 0, 8))
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    assert supervisor.to_dict()["attempts"] == 1
    assert supervisor.to_dict()["total_attempts"] == 2
    assert drone.request_mode.call_count == 3
    # A climb after rearming cannot grant another series at the same level.
    step(104.3, relative_altitude=10)
    step(105.8, relative_altitude=10, error=None, not_found=True)
    step(105.9, mode=DroneMode.QLAND, relative_altitude=10)
    assert supervisor.state is AlignmentState.EXHAUSTED
    assert supervisor.to_dict()["retry_below_altitude_m"] == 6
    step(107.1, mode=DroneMode.QLAND, relative_altitude=8)
    assert supervisor.state is AlignmentState.EXHAUSTED
    step(107.2, mode=DroneMode.QLAND, relative_altitude=6)
    assert supervisor.state is AlignmentState.WAIT_GUIDED
    assert supervisor.to_dict()["total_attempts"] == 3
    step(108, armed=False)
    assert supervisor.to_dict()["total_attempts"] == 0
    assert supervisor.to_dict()["retry_below_altitude_m"] is None


@pytest.mark.parametrize("height,starts", [(3.99, False), (4, True), (4.01, True)])
def test_configured_minimum_height_gates_first_attempt(scenario, height, starts):
    supervisor, drone, step, _ = scenario
    supervisor.config = replace(supervisor.config, min_height_m=4)
    step(mode=DroneMode.QLAND, marker_position=(0, 0, height))
    assert (supervisor.state is AlignmentState.WAIT_GUIDED) is starts
    assert drone.request_mode.call_count == int(starts)


@pytest.mark.parametrize("phase", ["WAIT_GUIDED", "HOLDING", "ALIGNING"])
def test_crossing_minimum_height_returns_to_landing(scenario, phase):
    supervisor, drone, step, align = scenario
    supervisor.config = replace(supervisor.config, min_height_m=4)
    if phase == "ALIGNING":
        align()
    else:
        step(mode=DroneMode.QLAND)
        if phase == "HOLDING":
            step(100.1)
    drone.send_yaw_input.reset_mock()
    step(101, marker_position=(0, 0, 3.9))
    assert supervisor.state is AlignmentState.WAIT_RETURN
    assert supervisor.reason == "below_min_height"
    drone.send_yaw_input.assert_not_called()
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))


@pytest.mark.parametrize("technical_failure", [False, True])
def test_descent_retry_does_not_rearm_command_failure_or_external_intervention(scenario, technical_failure):
    supervisor, drone, step, align = scenario
    supervisor.config = replace(supervisor.config, retry_descent_m=2, min_height_m=4)
    align()
    if technical_failure:
        drone.position_hold_accepted.return_value = False
        step(101)
        expected = AlignmentState.FAILED
    else:
        step(101, mode=DroneMode.QLOITER)
        expected = AlignmentState.SUSPENDED
    step(103, relative_altitude=4, marker_position=(0, 0, 4))
    assert supervisor.state is expected
    assert drone.request_mode.call_count == 1


def test_attempt_deadline_bounds_waiting_for_hover(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND)
    step(100.1)
    step(129.9, speed=1)
    assert supervisor.state is AlignmentState.HOLDING
    step(130, speed=1)
    assert supervisor.state is AlignmentState.WAIT_RETURN
    assert supervisor.reason == "attempt_timeout"
    drone.request_mode.assert_called_with(DroneMode.QLAND, expected_modes=(DroneMode.GUIDED, DroneMode.QLAND))


def test_loss_timeout_never_overwrites_external_mode(scenario):
    supervisor, drone, step, align = scenario
    align()
    step(103, error=None, mode=DroneMode.QRTL)
    assert supervisor.state is AlignmentState.SUSPENDED
    assert drone.request_mode.call_count == 1


def test_braking_tilt_does_not_turn_visible_target_into_detection_loss(scenario):
    supervisor, drone, step, _ = scenario
    step(mode=DroneMode.QLAND, marker_position=(0, 0, 20))
    step(100.1, marker_position=(0, 0, 20))
    roll = math.radians(10)
    # A target directly below a tilted aircraft is > 3 m off its body Z axis,
    # but remains horizontally centred in NED and continuously detected.
    position = (0, 20 * math.sin(roll), 20 * math.cos(roll))
    for tick in range(2, 21):
        step(100 + tick / 10, roll_rad=roll, marker_position=position, speed=1)
        assert supervisor.state is AlignmentState.HOLDING
        assert supervisor.error_deg is not None
    step(102.1, marker_position=(0, 0, 20))
    step(102.7, marker_position=(0, 0, 20))
    assert supervisor.state is AlignmentState.ALIGNING
    assert drone.send_yaw_input.call_args.args[0] > 0
    assert drone.request_mode.call_count == 1


@pytest.mark.parametrize("position", [(3, 0, 5), (0, 0, 2)])
def test_visible_but_geometrically_ineligible_target_never_starts_rotation(scenario, position):
    supervisor, drone, step, _ = scenario
    for tick in range(20):
        step(100 + tick / 10, mode=DroneMode.QLAND, marker_position=position)
    drone.request_mode.assert_not_called()
    assert supervisor.to_dict()["attempts"] == 0


@pytest.mark.parametrize("fractal", [True, False])
def test_fractal_level_change_preserves_target_identity_but_aruco_id_change_does_not(scenario, fractal):
    supervisor, drone, step, _ = scenario
    step(99.99, armed=False, frames=False)
    for tick in range(13):
        step(
            100 + tick / 10,
            mode=DroneMode.QLAND,
            marker_id=tick % 5,
            target_identity="faruco:FRACTAL_5L_6" if fractal else None,
        )
        if supervisor.state is AlignmentState.WAIT_GUIDED:
            break
    if fractal:
        assert supervisor.state is AlignmentState.WAIT_GUIDED
        assert supervisor.to_dict()["attempts"] == 1
    else:
        drone.request_mode.assert_not_called()
