import pytest

from payload.domain.motor_response import (
    LinearMotorResponse,
    MonotonousAffineSegmentsMotorResponse,
    SimpleDeadzoneMotorResponse,
)

ALL_FRACTIONS = [i / 20 for i in range(21)]


def test_linear_response_is_the_identity():
    response = LinearMotorResponse()
    assert [response.inverse_response(x) for x in ALL_FRACTIONS] == ALL_FRACTIONS


def test_simple_deadzone_skips_the_dead_zone():
    response = SimpleDeadzoneMotorResponse(0.2)
    assert response.inverse_response(0.0) == 0.0
    assert response.inverse_response(0.5) == pytest.approx(0.6)
    assert response.inverse_response(1.0) == pytest.approx(1.0)
    # The smallest wanted speed already starts the wheel.
    assert response.inverse_response(0.001) > 0.2


def test_simple_deadzone_of_zero_is_linear():
    response = SimpleDeadzoneMotorResponse(0.0)
    assert [response.inverse_response(x) for x in ALL_FRACTIONS] == pytest.approx(ALL_FRACTIONS)


@pytest.mark.parametrize("dead_zone", [-0.1, 1.0, 2.0, float("nan"), float("inf"), True, "0.2", None])
def test_simple_deadzone_rejects_invalid_dead_zone(dead_zone):
    with pytest.raises(ValueError, match="dead_zone must be a number in"):
        SimpleDeadzoneMotorResponse(dead_zone)


def test_affine_segments_follow_the_points_backwards():
    # The wheel starts at 20 % duty, reaches 40 % speed at 50 % duty, then full speed at full duty.
    response = MonotonousAffineSegmentsMotorResponse(((0.2, 0.0), (0.5, 0.4)))
    assert response.inverse_response(0.0) == 0.0
    assert response.inverse_response(0.2) == pytest.approx(0.35)
    assert response.inverse_response(0.4) == pytest.approx(0.5)
    assert response.inverse_response(0.7) == pytest.approx(0.75)
    assert response.inverse_response(1.0) == pytest.approx(1.0)


def test_affine_segments_without_points_are_linear():
    response = MonotonousAffineSegmentsMotorResponse(())
    assert [response.inverse_response(x) for x in ALL_FRACTIONS] == pytest.approx(ALL_FRACTIONS)


def test_affine_segments_can_describe_the_simple_dead_zone():
    simple = SimpleDeadzoneMotorResponse(0.25)
    points = MonotonousAffineSegmentsMotorResponse(((0.25, 0.0),))
    assert [points.inverse_response(x) for x in ALL_FRACTIONS] == pytest.approx(
        [simple.inverse_response(x) for x in ALL_FRACTIONS]
    )


def test_affine_segments_accept_explicit_end_points():
    response = MonotonousAffineSegmentsMotorResponse(((0.0, 0.0), (0.3, 0.0), (1.0, 1.0)))
    assert response.inverse_response(0.5) == pytest.approx(0.65)


def test_affine_segments_with_a_plateau_use_the_lowest_duty():
    response = MonotonousAffineSegmentsMotorResponse(((0.2, 0.0), (0.4, 0.5), (0.6, 0.5)))
    assert response.inverse_response(0.5) == pytest.approx(0.4)
    assert response.inverse_response(0.75) == pytest.approx(0.8)


@pytest.mark.parametrize(
    "response",
    [
        LinearMotorResponse(),
        SimpleDeadzoneMotorResponse(0.3),
        MonotonousAffineSegmentsMotorResponse(((0.1, 0.0), (0.4, 0.2), (0.8, 0.9))),
    ],
)
def test_every_response_is_monotonic_and_bounded(response):
    commands = [response.inverse_response(i / 200) for i in range(201)]
    assert all(0.0 <= command <= 1.0 for command in commands)
    assert commands == sorted(commands)
    assert commands[0] == 0.0
    assert commands[-1] == pytest.approx(1.0)


@pytest.mark.parametrize(
    "response",
    [LinearMotorResponse(), SimpleDeadzoneMotorResponse(0.3), MonotonousAffineSegmentsMotorResponse(((0.5, 0.2),))],
)
@pytest.mark.parametrize("fraction", [-0.1, 1.1, float("nan"), True, "0.5", None])
def test_every_response_rejects_speed_fractions_outside_the_unit_interval(response, fraction):
    with pytest.raises(ValueError, match="speed fraction must be a number in"):
        response.inverse_response(fraction)


@pytest.mark.parametrize(
    "points, message",
    [
        (((0.5, 0.2), (0.4, 0.3)), "duties must strictly increase"),
        (((0.5, 0.2), (0.5, 0.3)), "duties must strictly increase"),
        (((0.2, 0.5), (0.5, 0.4)), "must not decrease"),
        (((0.0, 0.1),), "duty 0 must have speed fraction 0"),
        (((1.0, 0.9),), "duty 1 must have speed fraction 1"),
        (((1.5, 0.5),), r"in \[0, 1\]"),
        (((0.5, -0.1),), r"in \[0, 1\]"),
        (((0.5, float("nan")),), "pairs of numbers"),
        (((0.5,),), "pairs of numbers"),
        (((0.5, 0.2, 0.1),), "pairs of numbers"),
        (((True, 0.2),), "pairs of numbers"),
        (([0.5, 0.2],), "pairs of numbers"),
        ([(0.5, 0.2)], "must be a tuple"),
    ],
)
def test_affine_segments_reject_invalid_points(points, message):
    with pytest.raises(ValueError, match=message):
        MonotonousAffineSegmentsMotorResponse(points)
