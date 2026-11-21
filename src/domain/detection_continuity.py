import math

from domain.tracking import TrackingResult, TrackingStatus


class DetectionContinuity:
    """Track visibility independently of flight geometry; the caller owns synchronization."""

    def __init__(self) -> None:
        self._last_sample_s: float | None = None
        self._since_s: float | None = None
        self._target_identity: str | None = None
        self._last_stable_s: float | None = None

    def observe(self, tracking: TrackingResult | None, now: float, *, max_age_s: float, stable_time_s: float) -> None:
        captured = tracking.captured_at_s if tracking else None
        if captured is None or not math.isfinite(captured) or not 0 <= now - captured < max_age_s:
            self._since_s = None
            return
        if self._last_sample_s is not None:
            if captured <= self._last_sample_s:
                return
            if captured - self._last_sample_s >= max_age_s:
                self._since_s = None
        self._last_sample_s = captured
        if tracking.status is not TrackingStatus.DETECTED or tracking.body_from_marker is None:
            self._since_s = None
            return
        identity = tracking.target_identity or f"marker:{tracking.marker_id}"
        if self._since_s is None or identity != self._target_identity:
            self._since_s = captured
        self._target_identity = identity
        if captured - self._since_s >= stable_time_s:
            self._last_stable_s = captured

    def is_stable(self, now: float, *, max_age_s: float) -> bool:
        return (
            self._since_s is not None
            and self._last_stable_s is not None
            and self._last_stable_s == self._last_sample_s
            and 0 <= now - self._last_stable_s < max_age_s
        )

    def loss_timed_out(self, now: float, *, timeout_s: float) -> bool:
        return self._last_stable_s is None or now - self._last_stable_s >= timeout_s

    def reset(self, now: float) -> None:
        """Require new observations captured strictly after this reset."""
        self._since_s = self._target_identity = self._last_stable_s = None
        self._last_sample_s = now
