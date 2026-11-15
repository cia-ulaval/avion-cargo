"""Configuration and geometry for autonomous marker heading alignment."""

import math
from dataclasses import dataclass, fields

import numpy as np

from domain.drone import DroneStatus
from domain.rigid_transform import RigidTransform


@dataclass(frozen=True, slots=True)
class YawAlignmentConfig:
    enabled: bool = False
    rate_hz: float = 10.0
    trigger_deg: float = 12.0
    tolerance_deg: float = 3.0
    stable_time_s: float = 1.0
    alignment_timeout_s: float = 15.0
    mode_timeout_s: float = 5.0
    preparation_timeout_s: float = 8.0
    hold_timeout_s: float = 10.0
    hold_stable_time_s: float = 0.5
    hold_horizontal_tolerance_m: float = 0.5
    hold_altitude_tolerance_m: float = 0.3
    hold_speed_mps: float = 0.3
    cooldown_s: float = 1.0
    detection_stable_time_s: float = 1.0
    detection_loss_timeout_s: float = 1.5
    attempt_timeout_s: float = 30.0
    max_attempts: int = 3
    retry_descent_m: float = 0.0
    min_height_m: float = 3.0
    max_horizontal_distance_m: float = 2.0
    max_groundspeed_mps: float = 0.8
    max_age_s: float = 0.4
    target_loss_grace_s: float = 0.25
    max_yaw_input: float = 0.25
    gain_per_rad: float = 0.6
    marker_forward_deg: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError("enabled must be a boolean")
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int):
            raise ValueError("max_attempts must be a positive integer")
        for field in fields(self):
            if field.name == "enabled":
                continue
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{field.name} must be finite and numeric")
            if field.name == "retry_descent_m" and value == 0:
                continue
            if field.name != "marker_forward_deg" and value <= 0:
                raise ValueError(f"{field.name} must be positive")
        if not 0 < self.tolerance_deg < self.trigger_deg <= 180:
            raise ValueError("Require 0 < tolerance_deg < trigger_deg <= 180")
        if self.max_yaw_input > 1 or self.rate_hz > 100:
            raise ValueError("max_yaw_input must be <= 1 and rate_hz <= 100")
        if self.stable_time_s >= self.alignment_timeout_s:
            raise ValueError("stable_time_s must be shorter than alignment_timeout_s")
        if self.hold_stable_time_s >= self.hold_timeout_s:
            raise ValueError("hold_stable_time_s must be shorter than hold_timeout_s")


def body_to_ned_rotation(status: DroneStatus) -> np.ndarray:
    """Use the same attitude convention for target geometry and heading."""
    roll, pitch, yaw = status.roll_rad, status.pitch_rad, status.yaw_rad
    if any(value is None or not math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("A finite NED vehicle attitude is required for heading alignment")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def marker_offset_ned(transform: RigidTransform, status: DroneStatus) -> np.ndarray:
    """Relative target offset along north, east and vertical down, in metres."""
    return body_to_ned_rotation(status) @ transform.as_matrix()[:3, 3]


def marker_heading_error_rad(transform: RigidTransform, status: DroneStatus, forward_deg: float = 0) -> float:
    """Project the chosen marker axis into horizontal NED, compensating body tilt.

    The marker's +X axis is forward at zero offset. Positive error requests
    clockwise yaw viewed from above, matching BODY_FRD/NED conventions.
    """
    ned_from_body = body_to_ned_rotation(status)
    offset = math.radians(forward_deg)
    forward = ned_from_body @ transform.as_matrix()[:3, :3] @ [math.cos(offset), math.sin(offset), 0]
    if math.hypot(forward[0], forward[1]) < 0.2:
        raise ValueError("Marker forward axis has no reliable horizontal heading")
    error = math.atan2(forward[1], forward[0]) - status.yaw_rad
    return math.atan2(math.sin(error), math.cos(error))
