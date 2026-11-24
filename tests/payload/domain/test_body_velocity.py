import math

import pytest

from payload.domain.body_velocity import BodyVelocity


def test_body_velocity_accepts_finite_planar_setpoint():
    velocity = BodyVelocity(0.1, -0.2, 0.3)

    assert (velocity.vx_m_s, velocity.vy_m_s, velocity.yaw_rate_rad_s) == (0.1, -0.2, 0.3)


@pytest.mark.parametrize("value", [True, math.inf, -math.inf, math.nan, "0.1"])
def test_body_velocity_rejects_non_finite_or_non_numeric_fields(value):
    with pytest.raises(ValueError):
        BodyVelocity(value, 0.0, 0.0)
