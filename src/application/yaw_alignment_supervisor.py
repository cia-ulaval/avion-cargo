"""A single software owner for QLAND -> GUIDED VTOL -> QLAND transitions."""

import math
from enum import Enum
from threading import RLock

from loguru import logger

from domain.drone import Drone, DroneMode, DroneStatus
from domain.tracking import TrackingResult, TrackingStatus
from domain.yaw_alignment import YawAlignmentConfig, marker_heading_error_rad


class AlignmentState(str, Enum):
    MONITORING = "MONITORING"
    PREPARING = "PREPARING"
    WAIT_GUIDED = "WAIT_GUIDED"
    ALIGNING = "ALIGNING"
    WAIT_RETURN = "WAIT_RETURN"
    COOLDOWN = "COOLDOWN"
    SUSPENDED = "SUSPENDED"
    FAILED = "FAILED"


class YawAlignmentSupervisor:
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
        self._cancel_reason = None
        self._failed_return = False
        self._preflight_ready = False

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "enabled": self.config.enabled,
                "state": self.state.value,
                "reason": self.reason,
                "error_deg": self.error_deg,
            }

    def _transition(self, state: AlignmentState, now: float, reason: str | None = None) -> None:
        logger.info("Yaw alignment: {} -> {} ({})", self.state.value, state.value, reason or "normal")
        self.state, self._state_since, self.reason = state, now, reason
        self._stable_since = None

    def _release(self) -> None:
        self.drone.release_yaw_control()

    def _return(self, now: float, reason: str | None = None) -> None:
        self._release()
        self._failed_return = reason is not None
        requested = self.drone.request_mode(self._original_mode, expected_modes=(DroneMode.GUIDED, self._original_mode))
        if requested is False:
            self._owns_session = False
            self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
            return
        self._transition(AlignmentState.WAIT_RETURN, now, reason)

    def _error(self, status: DroneStatus, tracking: TrackingResult | None, now: float) -> float | None:
        timestamps = (
            status.attitude_timestamp_s,
            status.position_timestamp_s,
            tracking.captured_at_s if tracking else None,
        )
        if any(timestamp is None or not 0 <= now - timestamp < self.config.max_age_s for timestamp in timestamps):
            return None
        if tracking.status is not TrackingStatus.DETECTED or tracking.body_from_marker is None:
            return None
        position = tracking.body_from_marker.position
        if (
            position.z < self.config.min_height_m
            or math.hypot(position.x, position.y) > self.config.max_horizontal_distance_m
        ):
            return None
        if (
            not math.isfinite(status.groundspeed_mps)
            or not 0 <= status.groundspeed_mps <= self.config.max_groundspeed_mps
        ):
            return None
        try:
            return marker_heading_error_rad(tracking.body_from_marker, status, self.config.marker_forward_deg)
        except ValueError:
            return None

    def step(self, status: DroneStatus | None, tracking: TrackingResult | None, now: float) -> None:
        with self._lock:
            if not self.config.enabled or status is None:
                return
            if not status.connected:
                # Retain ownership: on reconnection, cancel our GUIDED request
                # instead of forgetting an interrupted flight-control session.
                self._release()
                self._cancel_reason = "autopilot_disconnected"
                return
            if not status.armed:
                self._release()
                self._owns_session = False
                self._original_mode = None
                self._cancel_reason = None
                if self.state is not AlignmentState.MONITORING:
                    self._transition(AlignmentState.MONITORING, now, "disarmed")
                return
            error = self._error(status, tracking, now)
            self.error_deg = None if error is None else math.degrees(error)
            if self.state in (AlignmentState.SUSPENDED, AlignmentState.FAILED):
                return
            if self.state is AlignmentState.COOLDOWN:
                if now - self._state_since >= self.config.cooldown_s:
                    self._transition(AlignmentState.MONITORING, now)
                return
            if self.state is AlignmentState.MONITORING:
                if status.mode is not DroneMode.QLAND:
                    return
                if self._preflight_ready:
                    if error is None or abs(self.error_deg) < self.config.trigger_deg:
                        return
                    self._preflight_ready = False
                self._cancel_reason = None
                self._transition(AlignmentState.PREPARING, now)
            if self.state is AlignmentState.PREPARING:
                if status.mode is not DroneMode.QLAND:
                    self._transition(AlignmentState.MONITORING, now, "alignment_conditions_lost")
                    return
                if now - self._state_since >= self.config.preparation_timeout_s:
                    self._transition(AlignmentState.FAILED, now, "quadplane_preparation_timeout")
                    return
                try:
                    ready = self.drone.prepare_yaw_control()
                except (ValueError, RuntimeError) as error:
                    self._transition(AlignmentState.FAILED, now, str(error))
                    return
                if not ready:
                    return
                self._preflight_ready = True
                if error is None or abs(self.error_deg) < self.config.trigger_deg:
                    self._transition(AlignmentState.MONITORING, now)
                    return
                self._original_mode = status.mode
                self._owns_session = True
                self._transition(AlignmentState.WAIT_GUIDED, now)
                requested = self.drone.request_mode(DroneMode.GUIDED, expected_modes=(self._original_mode,))
                if requested is False:
                    self._owns_session = False
                    self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
                return
            if self.state is AlignmentState.WAIT_GUIDED:
                if error is None:
                    self._cancel_reason = self._cancel_reason or "target_or_attitude_lost"
                if status.mode not in (self._original_mode, DroneMode.GUIDED):
                    self._release()
                    self._owns_session = False
                    self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
                elif status.mode is DroneMode.GUIDED:
                    if self._cancel_reason:
                        self._return(now, self._cancel_reason)
                    else:
                        self._transition(AlignmentState.ALIGNING, now)
                elif now - self._state_since >= self.config.mode_timeout_s:
                    # Also cancel a GUIDED command which may still be in flight.
                    self._return(now, "guided_mode_timeout")
                return
            if self.state is AlignmentState.ALIGNING:
                if status.mode is not DroneMode.GUIDED:
                    self._release()
                    self._owns_session = False
                    self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
                elif error is None or self._cancel_reason:
                    self._return(now, self._cancel_reason or "target_or_attitude_lost")
                elif now - self._state_since >= self.config.alignment_timeout_s:
                    self._return(now, "alignment_timeout")
                elif not math.isfinite(status.speed) or abs(status.speed) > 0.3:
                    # GUIDED must have arrested the descent before yaw correction.
                    self._stable_since = None
                    self.drone.send_yaw_input(0.0)
                elif abs(self.error_deg) <= self.config.tolerance_deg:
                    self.drone.send_yaw_input(0.0)
                    if self._stable_since is None:
                        self._stable_since = tracking.captured_at_s
                    elif tracking.captured_at_s - self._stable_since >= self.config.stable_time_s:
                        self._return(now)
                else:
                    self._stable_since = None
                    value = max(
                        -self.config.max_yaw_input, min(self.config.max_yaw_input, self.config.gain_per_rad * error)
                    )
                    self.drone.send_yaw_input(value)
                return
            if self.state is AlignmentState.WAIT_RETURN:
                observed = status.last_heartbeat_monotonic_s
                if status.mode is self._original_mode and observed is not None and observed > self._state_since:
                    self._owns_session = False
                    self._transition(
                        AlignmentState.FAILED if self._failed_return else AlignmentState.COOLDOWN, now, self.reason
                    )
                elif status.mode not in (DroneMode.GUIDED, self._original_mode):
                    self._owns_session = False
                    self._transition(AlignmentState.SUSPENDED, now, "external_mode_change")
                elif now - self._state_since >= self.config.mode_timeout_s:
                    self._transition(AlignmentState.FAILED, now, "return_mode_timeout")

    def stop(self, status: DroneStatus | None) -> None:
        with self._lock:
            self._release()
            if self._owns_session and status is not None and status.connected and status.armed:
                if status.mode in (DroneMode.GUIDED, self._original_mode):
                    self.drone.request_mode(self._original_mode, expected_modes=(DroneMode.GUIDED, self._original_mode))
            self._owns_session = False
