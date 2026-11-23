from dataclasses import replace

import pytest

from vtol.domain.detection_continuity import DetectionContinuity
from vtol.domain.rigid_transform import RigidTransform
from vtol.domain.tracking import TrackingResult


def detected(captured, *, marker_id=29, target_identity=None):
    transform = RigidTransform.from_vectors([0, 0, 0], [0, 0, 5])
    return TrackingResult.detected(
        transform.position,
        marker_id,
        captured_at_s=captured,
        body_from_marker=transform,
        target_identity=target_identity,
    )


def observe(continuity, frame, now):
    continuity.observe(frame, now, max_age_s=0.5, stable_time_s=1.0)


def stable_detection():
    continuity = DetectionContinuity()
    for captured in (10, 10.25, 10.5, 10.75, 11):
        observe(continuity, detected(captured), captured)
    assert continuity.is_stable(11, max_age_s=0.5)
    return continuity


def test_stability_uses_new_capture_times_and_expires_at_the_freshness_boundary():
    continuity = stable_detection()
    observe(continuity, detected(11), 11.25)
    observe(continuity, detected(10.9), 11.25)
    assert continuity.is_stable(11.49, max_age_s=0.5)
    assert not continuity.is_stable(11.5, max_age_s=0.5)
    assert not continuity.loss_timed_out(12.49, timeout_s=1.5)
    assert continuity.loss_timed_out(12.5, timeout_s=1.5)


@pytest.mark.parametrize(
    "frame",
    [
        None,
        TrackingResult.not_found(captured_at_s=11.1),
        detected(None),
        detected(float("nan")),
        detected(10.5),
        detected(12),
        replace(detected(11.1), body_from_marker=None),
    ],
)
def test_invalid_observation_breaks_continuity_without_extending_loss_deadline(frame):
    continuity = stable_detection()
    observe(continuity, frame, 11.1)
    assert not continuity.is_stable(11.1, max_age_s=0.5)
    for captured in (11.25, 11.5, 11.75, 12):
        observe(continuity, detected(captured), captured)
        assert not continuity.is_stable(captured, max_age_s=0.5)
    assert continuity.loss_timed_out(12.5, timeout_s=1.5)
    observe(continuity, detected(12.25), 12.25)
    assert continuity.is_stable(12.25, max_age_s=0.5)
    assert not continuity.loss_timed_out(12.5, timeout_s=1.5)


def test_frame_gap_at_max_age_restarts_continuity_even_when_next_frame_is_fresh():
    continuity = stable_detection()
    observe(continuity, detected(11.5), 11.5)
    assert not continuity.is_stable(11.5, max_age_s=0.5)
    for captured in (11.75, 12, 12.25, 12.5):
        observe(continuity, detected(captured), captured)
    assert continuity.is_stable(12.5, max_age_s=0.5)


@pytest.mark.parametrize("same_physical_target", [True, False])
def test_marker_level_changes_only_preserve_continuity_with_a_shared_target_identity(same_physical_target):
    continuity = DetectionContinuity()
    identity = "faruco:landing-pad" if same_physical_target else None
    for captured in (10, 10.25, 10.5, 10.75):
        observe(continuity, detected(captured, target_identity=identity), captured)
    observe(continuity, detected(11, marker_id=30, target_identity=identity), 11)
    assert continuity.is_stable(11, max_age_s=0.5) is same_physical_target


def test_reset_requires_a_new_full_window_and_ignores_frames_at_or_before_reset():
    continuity = stable_detection()
    continuity.reset(11.25)
    for captured in (11, 11.25):
        observe(continuity, detected(captured), 11.25)
    assert not continuity.is_stable(11.25, max_age_s=0.5)
    assert continuity.loss_timed_out(11.25, timeout_s=1.5)
    for captured in (11.5, 11.75, 12, 12.25):
        observe(continuity, detected(captured), captured)
        assert not continuity.is_stable(captured, max_age_s=0.5)
    observe(continuity, detected(12.5), 12.5)
    assert continuity.is_stable(12.5, max_age_s=0.5)
