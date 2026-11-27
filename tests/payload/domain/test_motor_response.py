import pytest

from payload.domain.motor_response import MotorResponse


def test_default_response_is_identity():
    response = MotorResponse()
    for fraction in (0.0, 0.1, 0.5, 1.0):
        assert response.duty_for(fraction) == pytest.approx(fraction)


def test_zero_speed_always_gives_zero_duty():
    assert MotorResponse(dead_zone=0.3).duty_for(0.0) == 0.0


def test_dead_zone_is_skipped_for_any_nonzero_speed():
    response = MotorResponse(dead_zone=0.3)
    assert response.duty_for(1e-6) > 0.3
    assert response.duty_for(0.5) == pytest.approx(0.65)
    assert response.duty_for(1.0) == 1.0


def test_calibration_points_are_inverted_piecewise_linearly():
    response = MotorResponse(dead_zone=0.2, calibration=((0.5, 0.1), (0.8, 0.6)))
    assert response.duty_for(0.1) == pytest.approx(0.5)
    assert response.duty_for(0.6) == pytest.approx(0.8)
    assert response.duty_for(0.35) == pytest.approx(0.65)
    assert response.duty_for(0.05) == pytest.approx(0.35)
    assert response.duty_for(0.8) == pytest.approx(0.9)
    assert response.duty_for(1.0) == 1.0


def test_duty_is_monotonic_and_bounded():
    response = MotorResponse(dead_zone=0.25, calibration=((0.4, 0.2), (0.7, 0.55)))
    duties = [response.duty_for(step / 200) for step in range(201)]
    assert all(0.0 <= duty <= 1.0 for duty in duties)
    assert duties == sorted(duties)


@pytest.mark.parametrize("fraction", [-0.1, 1.1, float("nan"), float("inf"), True, "0.5"])
def test_rejects_invalid_speed_fractions(fraction):
    with pytest.raises(ValueError, match="speed fraction"):
        MotorResponse().duty_for(fraction)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dead_zone": -0.1},
        {"dead_zone": 1.0},
        {"dead_zone": float("nan")},
        {"dead_zone": True},
        {"dead_zone": 0.3, "calibration": ((0.3, 0.2),)},
        {"calibration": ((0.5, 0.2), (0.4, 0.3))},
        {"calibration": ((0.4, 0.3), (0.5, 0.3))},
        {"calibration": ((0.5, 0.0),)},
        {"calibration": ((1.0, 0.9),)},
        {"calibration": ((1.2, 1.0),)},
        {"calibration": ((0.5,),)},
    ],
)
def test_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        MotorResponse(**kwargs)


def test_motor_and_compensation_are_inverse_of_each_other():
    response = MotorResponse(dead_zone=0.2, calibration=((0.5, 0.1), (0.8, 0.6)))
    for step in range(1, 101):
        fraction = step / 100
        assert response.speed_fraction_for(response.duty_for(fraction)) == pytest.approx(fraction)


def test_motor_does_not_turn_inside_the_dead_zone():
    response = MotorResponse(dead_zone=0.3)
    assert response.speed_fraction_for(0.0) == 0.0
    assert response.speed_fraction_for(0.3) == 0.0
    assert response.speed_fraction_for(0.65) == pytest.approx(0.5)
    assert response.speed_fraction_for(1.0) == 1.0


@pytest.mark.parametrize("duty", [-0.1, 1.1, float("nan"), True, "0.5"])
def test_rejects_invalid_duties(duty):
    with pytest.raises(ValueError, match="duty"):
        MotorResponse().speed_fraction_for(duty)
