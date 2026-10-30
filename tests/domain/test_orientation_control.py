import math

import numpy as np
import pytest

from domain.orientation_control import OrientationControlConfig, marker_yaw_error_rad


@pytest.mark.parametrize(
    ("rotation", "axis", "axis_sign", "expected"),
    [
        (np.eye(3), "x", 1, 0.0),
        (np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), "x", 1, math.pi / 2),
        (np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), "x", -1, -math.pi / 2),
        (np.eye(3), "y", 1, math.pi / 2),
    ],
)
def test_marker_heading_error_uses_selected_marker_axis(rotation, axis, axis_sign, expected):
    assert marker_yaw_error_rad(rotation, marker_axis=axis, marker_axis_sign=axis_sign) == pytest.approx(expected)


def test_marker_heading_error_is_unavailable_when_axis_has_no_horizontal_projection():
    rotation = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    assert marker_yaw_error_rad(rotation, marker_axis="x") is None


def test_yaw_input_is_proportional_bounded_and_has_a_deadband():
    config = OrientationControlConfig(gain_pwm_per_rad=140, max_deflection_pwm=180, deadband_deg=3)
    assert config.yaw_input_pwm(0.0) == 1500
    assert config.yaw_input_pwm(math.radians(2)) == 1500
    assert config.yaw_input_pwm(0.5) == 1570
    assert config.yaw_input_pwm(3.0) == 1680
    assert config.yaw_input_pwm(-3.0) == 1320


def test_yaw_input_sign_can_be_reversed_for_airframe_mapping():
    config = OrientationControlConfig(yaw_input_sign=-1)
    assert config.yaw_input_pwm(0.2) < 1500


def test_invalid_orientation_control_settings_are_rejected():
    with pytest.raises(ValueError, match="yaw_channel"):
        OrientationControlConfig(yaw_channel=9)
    with pytest.raises(ValueError, match="marker_axis"):
        OrientationControlConfig(marker_axis="z")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="marker_axis_sign"):
        OrientationControlConfig(marker_axis_sign=True)