import math
from dataclasses import asdict
from enum import Enum
from threading import RLock

from loguru import logger

from domain.detection_continuity import DetectionContinuity
from domain.drone import Drone, DroneMode, DroneStatus, HoldPosition
from domain.tracking import TrackingResult, TrackingStatus
from domain.yaw_alignment import YawAlignmentConfig, marker_heading_error_rad, marker_offset_ned


class AlignmentState(str, Enum):
    MONITORING = "MONITORING"
    PREPARING = "PREPARING"
    WAIT_GUIDED = "WAIT_GUIDED"
    HOLDING = "HOLDING"
    WAIT_TARGET = "WAIT_TARGET"
    ALIGNING = "ALIGNING"
    WAIT_RETURN = "WAIT_RETURN"
    COOLDOWN = "COOLDOWN"
    EXHAUSTED = "EXHAUSTED"
    SUSPENDED = "SUSPENDED"
    FAILED = "FAILED"


class YawAlignmentSupervisor:
    _HOVER_STATES = (AlignmentState.HOLDING, AlignmentState.WAIT_TARGET, AlignmentState.ALIGNING)
    _ATTEMPT_STATES = (AlignmentState.WAIT_GUIDED, *_HOVER_STATES)

    def __init__(self, drone: Drone, config: YawAlignmentConfig):
        self.drone = drone
        self.config = config
        self._lock = RLock()
        self.state = AlignmentState.MONITORING
        self.reason = None
        self.error_deg = None
        self._state_since = 0.0
        self._stable_since = None
        self._original_mode = None
        self._owns_session = False
        self._preflight_ready = False
        self._last_target_capture_s = None
        self._hold_position = None
        self._hold_stable_since = None
        self._hold_requested = False
        self._alignment_started_s = None
        self._attempt_started_s = None
        self._attempts = 0
        self._total_attempts = 0
        self._retry_reference_altitude_m = None
        self._detection = DetectionContinuity()

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "enabled": self.config.enabled,
                "state": self.state.value,
                "reason": self.reason,
                "error_deg": self.error_deg,
                "hold_position": asdict(self._hold_position) if self._hold_position else None,
                "attempts": self._attempts,
                "max_attempts": self.config.max_attempts,
                "total_attempts": self._total_attempts,
                "retry_below_altitude_m": (
                    self._retry_reference_altitude_m - self.config.retry_descent_m
                    if self._retry_reference_altitude_m is not None and self.config.retry_descent_m > 0
                    else None
                ),
            }

    def observe_tracking(self, tracking: TrackingResult | None, now: float) -> None:
        """Observe every camera result, including losses between supervisor ticks.

        This only records detection continuity; flight commands remain owned by step().
        Duplicate frames never extend the stability window.
        """
        with self._lock:
            self._detection.observe(
                tracking, now, max_age_s=self.config.max_age_s, stable_time_s=self.config.detection_stable_time_s
            )

    def _detection_stable(self, now: float) -> bool:
        return self._detection.is_stable(now, max_age_s=self.config.max_age_s)

    def _transition(self, state: AlignmentState, now: float, reason: str | None = None) -> None:
        logger.info("Yaw alignment: {} -> {} ({})", self.state.value, state.value, reason or "normal")
        self.state, self._state_since, self.reason = state, now, reason
        self._stable_since = None

    def _release(self) -> None:
        self.drone.release_yaw_control()

    def _return(self, now: float, reason: str | None = None) -> None:
        self._release()
        requested = self.drone.request_mode(self._original_mode, expected_modes=(DroneMode.GUIDED, self._original_mode))
        if requested is False:
            self._owns_session = False
            self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
            return
        self._transition(AlignmentState.WAIT_RETURN, now, reason)

    def _fail(self, now: float, reason: str) -> None:
        if not self._hold_requested:
            self._release()
        # A command failure needs operator attention; normal vision/time limits
        # use _return() and permit another bounded attempt during the descent.
        self._transition(AlignmentState.FAILED, now, reason)

    def _hold_settled(self, status: DroneStatus, now: float) -> bool:
        target = self._hold_position
        values = (status.latitude, status.longitude, status.relative_altitude, status.speed)
        if (
            target is None
            or not self._telemetry_valid(status, now)
            or not all(math.isfinite(v) for v in values)
            or target.horizontal_error_m(status) > self.config.hold_horizontal_tolerance_m
            or abs(status.relative_altitude - target.relative_altitude_m) > self.config.hold_altitude_tolerance_m
            or abs(status.speed) > self.config.hold_speed_mps
            or status.groundspeed_mps > self.config.hold_speed_mps
        ):
            self._hold_stable_since = None
            return False
        if self._hold_stable_since is None:
            self._hold_stable_since = status.position_timestamp_s
        return status.position_timestamp_s - self._hold_stable_since >= self.config.hold_stable_time_s

    def _telemetry_valid(self, status: DroneStatus, now: float) -> bool:
        timestamps = (status.attitude_timestamp_s, status.position_timestamp_s)
        if any(timestamp is None or not 0 <= now - timestamp < self.config.max_age_s for timestamp in timestamps):
            return False
        if any(
            value is None or not math.isfinite(value) for value in (status.roll_rad, status.pitch_rad, status.yaw_rad)
        ):
            return False
        return math.isfinite(status.groundspeed_mps) and 0 <= status.groundspeed_mps <= self.config.max_groundspeed_mps

    def _can_wait_for_target(self, status: DroneStatus, tracking: TrackingResult | None, now: float) -> bool:
        # Tolerate a missed detection, never an observed geometry/flight-limit
        # violation. Only retain a timestamp; no old heading drives the vehicle.
        if tracking is not None and tracking.status is not TrackingStatus.NOT_FOUND:
            return False
        last = self._last_target_capture_s
        return (
            self._telemetry_valid(status, now)
            and math.isfinite(status.speed)
            and last is not None
            and 0 <= now - last < min(self.config.target_loss_grace_s, self.config.max_age_s)
        )

    def _marker_offset(self, status: DroneStatus, tracking: TrackingResult | None, now: float):
        if not self._telemetry_valid(status, now) or tracking is None:
            return None
        if tracking.captured_at_s is None or not 0 <= now - tracking.captured_at_s < self.config.max_age_s:
            return None
        if tracking.status is not TrackingStatus.DETECTED or tracking.body_from_marker is None:
            return None
        try:
            offset = marker_offset_ned(tracking.body_from_marker, status)
            return offset if all(math.isfinite(value) for value in offset) else None
        except ValueError:
            return None

    def _error(self, status: DroneStatus, tracking: TrackingResult | None, now: float) -> float | None:
        offset = self._marker_offset(status, tracking, now)
        if offset is None:
            return None
        try:
            north, east, down = offset
            if down < self.config.min_height_m or math.hypot(north, east) > self.config.max_horizontal_distance_m:
                return None
            return marker_heading_error_rad(tracking.body_from_marker, status, self.config.marker_forward_deg)
        except ValueError:
            return None

    def _exhaust(self, status: DroneStatus, now: float) -> None:
        # Measure further descent independently of changing fractal levels. Never
        # raise this reference after a climb, which could recycle the same band.
        references = [self._retry_reference_altitude_m]
        if self._hold_position is not None:
            references.append(self._hold_position.relative_altitude_m)
        if self._telemetry_valid(status, now):
            references.append(status.relative_altitude)
        references = [value for value in references if value is not None and math.isfinite(value)]
        self._retry_reference_altitude_m = min(references) if references else None
        self._transition(AlignmentState.EXHAUSTED, now, "attempt_limit_reached")

    def step(self, status: DroneStatus | None, tracking: TrackingResult | None, now: float) -> None:
        """Advance the flight sequence, preserving the order of commands within a tick."""
        with self._lock:
            if not self.config.enabled:
                return
            self.observe_tracking(tracking, now)
            if self._handle_interruptions(status, now):
                return
            if self.state is AlignmentState.SUSPENDED:
                return
            if self.state is AlignmentState.EXHAUSTED:
                self._step_exhausted(status, tracking, now)
                if self.state is AlignmentState.EXHAUSTED:
                    return
                # A descent retry can prepare and request GUIDED in this same tick.
            if self.state is AlignmentState.FAILED:
                self._step_failed(status)
                return
            if self.state is AlignmentState.COOLDOWN:
                self._step_cooldown(now)
                return  # Even after MONITORING is restored, wait for the next tick.

            error = self._error(status, tracking, now)
            self.error_deg = None if error is None else math.degrees(error)
            if error is not None:
                self._last_target_capture_s = tracking.captured_at_s
            if self.state is AlignmentState.MONITORING:
                self._step_monitoring(status, error, now)
                if self.state is not AlignmentState.PREPARING:
                    return
                # Initial preparation also runs immediately, before ending the tick.
            if self.state is AlignmentState.PREPARING:
                self._step_preparing(status, error, now)
                return
            if self.state in self._ATTEMPT_STATES and self._check_attempt_limits(status, tracking, now):
                return
            if self.state is AlignmentState.WAIT_GUIDED:
                self._step_wait_guided(status, now)
                return
            if self.state in self._HOVER_STATES:
                self._step_hover_and_alignment(status, tracking, error, now)
                return
            if self.state is AlignmentState.WAIT_RETURN:
                self._step_wait_return(status, now)

    def _handle_interruptions(self, status: DroneStatus | None, now: float) -> bool:
        """Handle connection, disarm and external modes in priority order; true ends the tick."""
        if status is None or not status.connected:
            self._release()
            self._stable_since = self._hold_stable_since = None
            self.reason = "autopilot_disconnected"
            return True
        if not status.armed:
            self._reset_session(now)
            return True

        # Once GUIDED owns the hover, any pilot/failsafe mode takes priority.
        allowed = (DroneMode.GUIDED,)
        if self.state in (AlignmentState.WAIT_GUIDED, AlignmentState.WAIT_RETURN) or not self._hold_requested:
            allowed = (DroneMode.GUIDED, self._original_mode)
        if self._owns_session and status.mode not in allowed:
            self._release()
            self._owns_session = False
            self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
            return True
        return False

    def _reset_session(self, now: float) -> None:
        self._release()
        self._owns_session = False
        self._original_mode = self._hold_position = None
        self._hold_requested = False
        self._alignment_started_s = self._last_target_capture_s = None
        self._attempt_started_s = None
        self._attempts = 0
        self._total_attempts = 0
        self._retry_reference_altitude_m = None
        self._detection.reset(now)
        self._stable_since = self._hold_stable_since = None
        self._preflight_ready = False
        if self.state is not AlignmentState.MONITORING:
            self._transition(AlignmentState.MONITORING, now, "disarmed")

    def _step_exhausted(self, status: DroneStatus, tracking: TrackingResult | None, now: float) -> None:
        error = self._error(status, tracking, now)
        self.error_deg = None if error is None else math.degrees(error)
        if (
            self.config.retry_descent_m <= 0
            or self._retry_reference_altitude_m is None
            or status.mode is not DroneMode.QLAND
            or not math.isfinite(status.relative_altitude)
            or status.relative_altitude > self._retry_reference_altitude_m - self.config.retry_descent_m
            or now - self._state_since < self.config.cooldown_s
            or error is None
            or abs(self.error_deg) < self.config.trigger_deg
            or not self._detection_stable(now)
        ):
            return
        self._attempts = 0
        self._retry_reference_altitude_m = status.relative_altitude
        self._transition(AlignmentState.MONITORING, now, "retry_after_descent")

    def _step_failed(self, status: DroneStatus) -> None:
        if self._owns_session and status.mode is DroneMode.GUIDED:
            if not self._hold_requested:
                self._hold_requested = self.drone.request_position_hold(self._hold_position) is True
            self.drone.send_yaw_input(0.0)

    def _step_cooldown(self, now: float) -> None:
        if now - self._state_since >= self.config.cooldown_s:
            self._transition(AlignmentState.MONITORING, now)

    def _step_monitoring(self, status: DroneStatus, error: float | None, now: float) -> None:
        if status.mode is not DroneMode.QLAND:
            return
        if self._attempts >= self.config.max_attempts:
            self._exhaust(status, now)
            return
        if self._preflight_ready and (error is None or abs(self.error_deg) < self.config.trigger_deg):
            return
        if self._preflight_ready and not self._detection_stable(now):
            self.reason = "waiting_for_stable_detection"
            return
        self._transition(AlignmentState.PREPARING, now)

    def _step_preparing(self, status: DroneStatus, error: float | None, now: float) -> None:
        if status.mode is not DroneMode.QLAND:
            self._transition(AlignmentState.MONITORING, now, "alignment_conditions_lost")
            return
        if now - self._state_since >= self.config.preparation_timeout_s:
            self._fail(now, "quadplane_preparation_timeout")
            return
        try:
            ready = self.drone.prepare_yaw_control()
        except (ValueError, RuntimeError) as failure:
            self._fail(now, str(failure))
            return
        if not ready:
            return
        self._preflight_ready = True
        if error is None or abs(self.error_deg) < self.config.trigger_deg or not self._detection_stable(now):
            self._transition(AlignmentState.MONITORING, now, "waiting_for_stable_detection")
            return
        try:
            self._hold_position = HoldPosition(status.latitude, status.longitude, status.relative_altitude)
        except ValueError as failure:
            self._fail(now, str(failure))
            return
        self._original_mode = status.mode
        self._owns_session = True
        self._hold_requested = False
        self._alignment_started_s = self._hold_stable_since = None
        self._transition(AlignmentState.WAIT_GUIDED, now)
        if self.drone.request_mode(DroneMode.GUIDED, expected_modes=(self._original_mode,)) is False:
            self._owns_session = False
            self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
        else:
            self._attempts += 1
            self._total_attempts += 1
            self._attempt_started_s = now
            logger.info("Yaw alignment attempt {}/{}", self._attempts, self.config.max_attempts)

    def _check_attempt_limits(self, status: DroneStatus, tracking: TrackingResult | None, now: float) -> bool:
        """Return to landing on the first exceeded limit; true ends the tick."""
        offset = self._marker_offset(status, tracking, now)
        if offset is not None and offset[2] < self.config.min_height_m:
            self._return(now, "below_min_height")
            return True
        if now - self._attempt_started_s >= self.config.attempt_timeout_s:
            self._return(now, "attempt_timeout")
            return True
        if self._detection.loss_timed_out(now, timeout_s=self.config.detection_loss_timeout_s):
            self._return(now, "detection_unstable")
            return True
        return False

    def _step_wait_guided(self, status: DroneStatus, now: float) -> None:
        observed = status.last_heartbeat_monotonic_s
        if status.mode is DroneMode.GUIDED and observed is not None and observed > self._state_since:
            if self.drone.request_position_hold(self._hold_position) is not True:
                self._fail(now, "hold_request_refused")
                return
            self._hold_requested = True
            self.drone.send_yaw_input(0.0)
            self._transition(AlignmentState.HOLDING, now)
        elif now - self._state_since >= self.config.mode_timeout_s:
            self._fail(now, "guided_mode_timeout")

    def _step_hover_and_alignment(
        self, status: DroneStatus, tracking: TrackingResult | None, error: float | None, now: float
    ) -> None:
        accepted = self.drone.position_hold_accepted()
        if accepted is False:
            self.drone.send_yaw_input(0.0)
            self._fail(now, "hold_command_rejected")
            return
        if accepted is not True:
            self.drone.send_yaw_input(0.0)
            if now - self._state_since >= self.config.hold_timeout_s:
                self._fail(now, "hold_ack_timeout")
            return
        if self._alignment_started_s is not None and now - self._alignment_started_s >= self.config.alignment_timeout_s:
            self.drone.send_yaw_input(0.0)
            self._return(now, "alignment_timeout")
            return
        if not self._hold_settled(status, now):
            self._wait_for_hover(status, now)
            return
        if error is None or not self._detection_stable(now):
            self.drone.send_yaw_input(0.0)
            self._stable_since = None
            if not self._can_wait_for_target(status, tracking, now) and self.state is not AlignmentState.WAIT_TARGET:
                self._transition(AlignmentState.WAIT_TARGET, now, "target_or_attitude_lost")
            return
        self._step_alignment(tracking, error, now)

    def _wait_for_hover(self, status: DroneStatus, now: float) -> None:
        self.drone.send_yaw_input(0.0)
        self._stable_since = None
        if self.state is not AlignmentState.HOLDING:
            self._transition(AlignmentState.HOLDING, now, "stabilizing_position_and_altitude")
        elif now - self._state_since >= self.config.hold_timeout_s and self.reason != "waiting_for_stable_hover":
            # A slow brake/settling response is recoverable. The FC
            # already accepted the fixed hover target; keep waiting
            # instead of permanently disabling yaw just before it settles.
            self.reason = "waiting_for_stable_hover"
            logger.warning(
                "Hover still settling: horizontal error {:.2f} m, altitude error {:.2f} m, "
                "horizontal speed {:.2f} m/s, vertical speed {:.2f} m/s; keeping GUIDED",
                self._hold_position.horizontal_error_m(status),
                abs(status.relative_altitude - self._hold_position.relative_altitude_m),
                status.groundspeed_mps,
                abs(status.speed),
            )

    def _step_alignment(self, tracking: TrackingResult, error: float, now: float) -> None:
        if self.state is not AlignmentState.ALIGNING:
            self._transition(AlignmentState.ALIGNING, now)
            if self._alignment_started_s is None:
                self._alignment_started_s = now
        if abs(self.error_deg) <= self.config.tolerance_deg:
            self.drone.send_yaw_input(0.0)
            if self._stable_since is None:
                self._stable_since = tracking.captured_at_s
            elif tracking.captured_at_s - self._stable_since >= self.config.stable_time_s:
                self._return(now)
        else:
            self._stable_since = None
            value = max(-self.config.max_yaw_input, min(self.config.max_yaw_input, self.config.gain_per_rad * error))
            self.drone.send_yaw_input(value)

    def _step_wait_return(self, status: DroneStatus, now: float) -> None:
        observed = status.last_heartbeat_monotonic_s
        if status.mode is self._original_mode and observed is not None and observed > self._state_since:
            self._release()
            self._owns_session = False
            self._detection.reset(now)
            if self._attempts >= self.config.max_attempts:
                self._exhaust(status, now)
            else:
                self._transition(AlignmentState.COOLDOWN, now, self.reason)
        elif now - self._state_since >= self.config.mode_timeout_s:
            self._fail(now, "return_mode_timeout")
        elif status.mode is DroneMode.GUIDED:
            self.drone.send_yaw_input(0.0)

    def stop(self, status: DroneStatus | None) -> None:
        with self._lock:
            # Preserve the FC's fixed GUIDED target on shutdown. Stopping the
            # companion must not authorize an unaligned landing.
            self._release()
            self._owns_session = False
