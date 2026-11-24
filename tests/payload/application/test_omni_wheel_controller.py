import math

import pytest

from payload.application.omni_wheel_controller import OmniWheelController
from payload.domain.body_velocity import BodyVelocity
from payload.domain.wheel_actuator import WheelActuatorCapabilities
from payload.domain.wheel_geometry import WheelTransform2D


class FakeActuator:
    def __init__(self, maximum=0.5, *, fail=False):
        self.capabilities = WheelActuatorCapabilities(maximum)
        self.commands = []
        self.stops = 0
        self.closed = False
        self.fail = fail

    async def set_surface_velocity(self, velocity):
        if self.fail:
            raise RuntimeError("simulated actuator failure")
        self.commands.append(velocity)

    async def stop(self):
        self.stops += 1

    async def close(self):
        self.closed = True


def make_wheels():
    transforms = (
        # yaw is the positive rolling direction, not the wheel axle angle.
        WheelTransform2D(0.0675499815, 0.039, 2.094395103, 0.04),
        WheelTransform2D(-0.0675499815, 0.039, 4.188790207, 0.04),
        WheelTransform2D(0.0, -0.078, 6.283185307, 0.04),
    )
    return transforms


def test_forward_translation_resolves_wheel_surface_velocities():
    actuators = [FakeActuator() for _ in range(3)]
    controller = OmniWheelController(make_wheels(), actuators)

    velocities = controller.calculate_surface_velocities(BodyVelocity(0.1, 0.0, 0.0))

    assert velocities == pytest.approx((-0.05, -0.05, 0.1))


def test_pure_yaw_rotation_produces_equal_wheel_speeds_for_symmetric_layout():
    actuators = [FakeActuator() for _ in range(3)]
    controller = OmniWheelController(make_wheels(), actuators)

    velocities = controller.calculate_surface_velocities(BodyVelocity(0.0, 0.0, 1.0))

    assert velocities == pytest.approx((0.078, 0.078, 0.078))


def test_scaled_wheel_velocities_are_clamped_after_float_rounding():
    actuators = [FakeActuator(0.12) for _ in range(3)]
    controller = OmniWheelController(make_wheels(), actuators)

    velocities = controller._limit_surface_velocities(
        (0.12000000000000001, -0.12000000000000001, 0.06)
    )

    assert velocities == pytest.approx((0.12, -0.12, 0.06))
    assert all(
        abs(value) <= actuator.capabilities.max_surface_velocity
        for value, actuator in zip(velocities, actuators)
    )


def test_wheel_limits_scale_all_wheels_proportionally():
    actuators = [FakeActuator(0.2) for _ in range(3)]
    controller = OmniWheelController(make_wheels(), actuators)

    velocities = controller._limit_surface_velocities((0.4, -0.2, 0.1))

    assert velocities == pytest.approx((0.2, -0.1, 0.05))


@pytest.mark.asyncio
async def test_controller_stops_every_actuator_if_one_command_fails():
    actuators = [FakeActuator(), FakeActuator(fail=True), FakeActuator()]
    controller = OmniWheelController(make_wheels(), actuators)

    with pytest.raises(RuntimeError, match="simulated actuator failure"):
        await controller.set_body_velocity(BodyVelocity(0.1, 0.0, 0.0))

    assert [actuator.stops for actuator in actuators] == [1, 1, 1]


@pytest.mark.asyncio
async def test_controller_does_not_republish_an_unchanged_setpoint():
    actuators = [FakeActuator() for _ in range(3)]
    controller = OmniWheelController(make_wheels(), actuators)
    velocity = BodyVelocity(0.1, 0.0, 0.0)

    await controller.set_body_velocity(velocity)
    await controller.set_body_velocity(velocity)

    assert [len(actuator.commands) for actuator in actuators] == [1, 1, 1]
    await controller.close()
    assert all(actuator.closed for actuator in actuators)


def test_reversing_positive_rolling_direction_reverses_motor_command():
    forward = WheelTransform2D(0.0, 0.0, 0.0, 0.04)
    reversed_wheel = WheelTransform2D(0.0, 0.0, math.pi, 0.04)
    forward_controller = OmniWheelController((forward, forward, forward), [FakeActuator() for _ in range(3)])
    reversed_controller = OmniWheelController((reversed_wheel, reversed_wheel, reversed_wheel), [FakeActuator() for _ in range(3)])

    forward_speed = forward_controller.calculate_surface_velocities(BodyVelocity(0.1, 0.0, 0.0))
    reversed_speed = reversed_controller.calculate_surface_velocities(BodyVelocity(0.1, 0.0, 0.0))

    assert forward_speed == pytest.approx((0.1, 0.1, 0.1))
    assert reversed_speed == pytest.approx((-0.1, -0.1, -0.1))
