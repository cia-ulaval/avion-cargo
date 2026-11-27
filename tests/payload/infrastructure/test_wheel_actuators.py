import asyncio

import pytest

from payload.domain.motor_response import MotorResponse
from payload.domain.wheel_actuator import WheelActuatorCapabilities
from payload.infrastructure.configuration import (
    GazeboMotorDriverConfiguration,
    MX1616MotorDriverConfiguration,
)
from payload.infrastructure.wheel_actuators import (
    GazeboWheelActuator,
    MX1616WheelActuator,
    SimulatedMotorWheelActuator,
    WheelActuatorLinearizer,
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
        "/wheel/cmd_vel",
        0.04,
        WheelActuatorCapabilities(0.5),
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
        "/wheel/cmd_vel",
        0.04,
        WheelActuatorCapabilities(0.5),
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
        17,
        18,
        WheelActuatorCapabilities(0.4),
        output_factory=factory,
    )
    await actuator.set_surface_velocity(0.2)
    assert (outputs[0].value, outputs[1].value) == (0.5, 0.0)

    await actuator.set_surface_velocity(-0.1)
    assert (outputs[0].value, outputs[1].value) == (0.0, 0.25)

    await actuator.stop()
    assert (outputs[0].value, outputs[1].value) == (0.0, 0.0)
    await actuator.close()
    assert all(output.closed for output in outputs)


def test_factory_builds_both_driver_types(monkeypatch):
    from payload.infrastructure import wheel_actuators

    monkeypatch.setattr(
        wheel_actuators,
        "_gazebo_transport_publisher",
        lambda _topic: (lambda *_: None),
    )
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


@pytest.mark.asyncio
async def test_gazebo_actuators_reuse_persistent_transport_publishers(monkeypatch):
    import sys
    import types

    from payload.infrastructure import wheel_actuators

    class FakeDouble:
        def __init__(self):
            self.data = 0.0

    class FakePublisher:
        def __init__(self, topic):
            self.topic = topic
            self.values = []

        def publish(self, message):
            self.values.append(message.data)
            return True

    class FakeNode:
        instances = []

        def __init__(self):
            self.advertised = {}
            self.instances.append(self)

        def advertise(self, topic, message_type):
            assert message_type is FakeDouble
            publisher = FakePublisher(topic)
            self.advertised[topic] = publisher
            return publisher

    gz_module = types.ModuleType("gz")
    gz_module.__path__ = []
    msgs_module = types.ModuleType("gz.msgs10")
    msgs_module.__path__ = []
    double_module = types.ModuleType("gz.msgs10.double_pb2")
    double_module.Double = FakeDouble
    transport_module = types.ModuleType("gz.transport13")
    transport_module.Node = FakeNode
    monkeypatch.setitem(sys.modules, "gz", gz_module)
    monkeypatch.setitem(sys.modules, "gz.msgs10", msgs_module)
    monkeypatch.setitem(sys.modules, "gz.msgs10.double_pb2", double_module)
    monkeypatch.setitem(sys.modules, "gz.transport13", transport_module)
    monkeypatch.setattr(wheel_actuators, "_gazebo_transport", None)

    topics = ("/wheel/0/cmd_vel", "/wheel/1/cmd_vel", "/wheel/2/cmd_vel")
    actuators = [GazeboWheelActuator(topic, 0.04, WheelActuatorCapabilities(0.5)) for topic in topics]

    await actuators[0].set_surface_velocity(0.2)
    await actuators[1].set_surface_velocity(-0.1)
    await actuators[2].set_surface_velocity(0.05)
    await actuators[0].set_surface_velocity(0.1)

    assert len(FakeNode.instances) == 1
    node = FakeNode.instances[0]
    assert set(node.advertised) == set(topics)
    assert node.advertised[topics[0]].values == [pytest.approx(5.0), pytest.approx(2.5)]
    assert node.advertised[topics[1]].values == [pytest.approx(-2.5)]
    assert node.advertised[topics[2]].values == [pytest.approx(1.25)]


class RecordingActuator:
    """Stand-in for the actuator under a wrapper: it keeps the speeds it is given."""

    def __init__(self, max_speed=0.2):
        self.capabilities = WheelActuatorCapabilities(max_speed)
        self.speeds = []
        self.calls = []

    async def set_surface_velocity(self, velocity_m_s):
        self.speeds.append(velocity_m_s)

    async def stop(self):
        self.calls.append("stop")

    async def close(self):
        self.calls.append("close")


def test_linearizer_skips_the_dead_zone_and_keeps_the_direction():
    inner = RecordingActuator(0.2)
    linearizer = WheelActuatorLinearizer(inner, MotorResponse(dead_zone=0.2))

    async def scenario():
        for velocity in (0.0, 0.1, -0.1, 0.2, -0.2):
            await linearizer.set_surface_velocity(velocity)

    asyncio.run(scenario())
    # 50 % wanted -> 0.2 + 0.8 * 0.5 = 60 % of the maximum speed.
    assert inner.speeds == [0.0, pytest.approx(0.12), pytest.approx(-0.12), 0.2, -0.2]


def test_linearizer_rejects_speeds_outside_the_limits_and_forwards_the_rest():
    inner = RecordingActuator(0.2)
    linearizer = WheelActuatorLinearizer(inner, MotorResponse(dead_zone=0.2))

    async def scenario():
        with pytest.raises(ValueError, match="outside actuator limits"):
            await linearizer.set_surface_velocity(0.21)
        await linearizer.stop()
        await linearizer.close()

    asyncio.run(scenario())
    assert inner.speeds == []
    assert inner.calls == ["stop", "close"]
    assert linearizer.capabilities is inner.capabilities


def test_simulated_motor_does_not_move_inside_its_dead_zone():
    inner = RecordingActuator(0.2)
    motor = SimulatedMotorWheelActuator(inner, MotorResponse(dead_zone=0.3))

    async def scenario():
        await motor.set_surface_velocity(0.04)  # 20 % duty, under the 30 % dead zone
        await motor.set_surface_velocity(-0.1)  # 50 % duty -> (0.5 - 0.3) / 0.7 of the speed
        await motor.set_surface_velocity(0.2)

    asyncio.run(scenario())
    assert inner.speeds == [0.0, pytest.approx(-0.2 * 0.2 / 0.7), 0.2]


def test_linearizer_and_simulated_motor_give_back_the_wanted_speed():
    response = MotorResponse(dead_zone=0.25, calibration=((0.6, 0.4),))
    inner = RecordingActuator(0.2)
    chain = WheelActuatorLinearizer(SimulatedMotorWheelActuator(inner, response), response)

    async def scenario():
        for velocity in (0.002, 0.05, -0.1, 0.2):
            await chain.set_surface_velocity(velocity)

    asyncio.run(scenario())
    assert inner.speeds == [pytest.approx(v) for v in (0.002, 0.05, -0.1, 0.2)]


def test_uncompensated_simulated_motor_loses_small_speeds():
    inner = RecordingActuator(0.2)
    chain = SimulatedMotorWheelActuator(inner, MotorResponse(dead_zone=0.3))
    asyncio.run(chain.set_surface_velocity(0.04))
    assert inner.speeds == [0.0]


def test_factory_builds_the_chain_for_each_driver(monkeypatch):
    from payload.infrastructure import wheel_actuators

    monkeypatch.setattr(wheel_actuators, "_gazebo_transport_publisher", lambda topic: lambda t, v: None)
    capabilities = WheelActuatorCapabilities(0.2)
    response = MotorResponse(dead_zone=0.2)
    pins = lambda pin, **kwargs: FakePWMOutput(pin, **kwargs)  # noqa: E731

    plain = create_wheel_actuator(GazeboMotorDriverConfiguration("gazebo", "/t"), 0.019, capabilities)
    assert isinstance(plain, GazeboWheelActuator)

    simulated = create_wheel_actuator(
        GazeboMotorDriverConfiguration("gazebo", "/t", response=response), 0.019, capabilities
    )
    assert isinstance(simulated, WheelActuatorLinearizer)
    assert isinstance(simulated._actuator, SimulatedMotorWheelActuator)
    assert isinstance(simulated._actuator._actuator, GazeboWheelActuator)

    uncompensated = create_wheel_actuator(
        GazeboMotorDriverConfiguration("gazebo", "/t", motor=response), 0.019, capabilities
    )
    assert isinstance(uncompensated, SimulatedMotorWheelActuator)

    robot_plain = create_wheel_actuator(
        MX1616MotorDriverConfiguration("mx1616", 17, 18), 0.019, capabilities, output_factory=pins
    )
    assert isinstance(robot_plain, MX1616WheelActuator)

    robot = create_wheel_actuator(
        MX1616MotorDriverConfiguration("mx1616", 17, 18, response), 0.019, capabilities, output_factory=pins
    )
    assert isinstance(robot, WheelActuatorLinearizer)
    assert isinstance(robot._actuator, MX1616WheelActuator)


def test_linearized_mx1616_applies_the_dead_zone_to_the_pwm():
    outputs = []

    def factory(pin, **kwargs):
        outputs.append(FakePWMOutput(pin, **kwargs))
        return outputs[-1]

    actuator = create_wheel_actuator(
        MX1616MotorDriverConfiguration("mx1616", 17, 18, MotorResponse(dead_zone=0.2)),
        0.019,
        WheelActuatorCapabilities(0.4),
        output_factory=factory,
    )

    async def scenario():
        await actuator.set_surface_velocity(0.004)
        assert outputs[0].value > 0.2 and outputs[1].value == 0.0
        await actuator.set_surface_velocity(-0.2)
        assert outputs[0].value == 0.0 and outputs[1].value == pytest.approx(0.6)
        await actuator.set_surface_velocity(0.4)
        assert outputs[0].value == 1.0

    asyncio.run(scenario())
