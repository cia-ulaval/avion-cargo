import pytest

from payload.domain.wheel_actuator import WheelActuatorCapabilities
from payload.infrastructure.configuration import (
    GazeboMotorDriverConfiguration,
    MX1616MotorDriverConfiguration,
)
from payload.infrastructure.wheel_actuators import (
    GazeboWheelActuator,
    MX1616WheelActuator,
    create_wheel_actuator,
)


class FakePWMOutput:
    def __init__(self, pin, *, frequency, initial_value):
        self.pin = pin
        self.frequency = frequency
        self.value = initial_value
        self.closed = False

    def close(self):
        self.closed = True


def test_capabilities_expose_symmetric_speed_limits():
    capabilities = WheelActuatorCapabilities(0.8)
    assert capabilities.max_surface_velocity == 0.8
    assert capabilities.min_surface_velocity == -0.8


def test_capabilities_reject_invalid_maximum_speed():
    with pytest.raises(ValueError, match="finite and positive"):
        WheelActuatorCapabilities(float("inf"))


@pytest.mark.asyncio
async def test_gazebo_actuator_converts_surface_speed_to_joint_angular_speed():
    published = []
    actuator = GazeboWheelActuator(
        "/wheel/cmd_vel", 0.04, WheelActuatorCapabilities(0.5),
        publisher=lambda topic, value: published.append((topic, value)),
    )

    await actuator.set_surface_velocity(0.2)
    await actuator.set_surface_velocity(-0.1)
    await actuator.stop()

    assert published == [
        ("/wheel/cmd_vel", pytest.approx(5.0)),
        ("/wheel/cmd_vel", pytest.approx(-2.5)),
        ("/wheel/cmd_vel", 0.0),
    ]


@pytest.mark.asyncio
async def test_gazebo_actuator_rejects_speed_outside_capabilities():
    actuator = GazeboWheelActuator(
        "/wheel/cmd_vel", 0.04, WheelActuatorCapabilities(0.5),
        publisher=lambda *_: None,
    )
    with pytest.raises(ValueError, match="outside actuator limits"):
        await actuator.set_surface_velocity(0.51)


@pytest.mark.asyncio
async def test_mx1616_actuator_sets_direction_and_pwm_duty():
    outputs = []

    def factory(pin, **kwargs):
        output = FakePWMOutput(pin, **kwargs)
        outputs.append(output)
        return output

    actuator = MX1616WheelActuator(
        17, 18, WheelActuatorCapabilities(0.4), output_factory=factory,
    )
    await actuator.set_surface_velocity(0.2)
    assert (outputs[0].value, outputs[1].value) == (0.5, 0.0)

    await actuator.set_surface_velocity(-0.1)
    assert (outputs[0].value, outputs[1].value) == (0.0, 0.25)

    await actuator.stop()
    assert (outputs[0].value, outputs[1].value) == (0.0, 0.0)
    await actuator.close()
    assert all(output.closed for output in outputs)


def test_factory_builds_both_driver_types():
    capabilities = WheelActuatorCapabilities(0.5)
    gazebo = create_wheel_actuator(
        GazeboMotorDriverConfiguration("gazebo", "/wheel/cmd_vel"),
        0.04,
        capabilities,
    )
    assert isinstance(gazebo, GazeboWheelActuator)

    mx1616 = create_wheel_actuator(
        MX1616MotorDriverConfiguration("mx1616", 17, 18),
        0.04,
        capabilities,
        output_factory=FakePWMOutput,
    )
    assert isinstance(mx1616, MX1616WheelActuator)
