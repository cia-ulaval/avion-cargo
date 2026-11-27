from __future__ import annotations

import asyncio
import math
import threading
from collections.abc import Callable
from typing import Any

from payload.domain.motor_response import MotorResponse
from payload.domain.wheel_actuator import WheelActuator, WheelActuatorCapabilities
from payload.infrastructure.configuration import (
    GazeboMotorDriverConfiguration,
    MotorDriverConfiguration,
    MX1616MotorDriverConfiguration,
)


def _validate_velocity(velocity_m_s: float, capabilities: WheelActuatorCapabilities) -> float:
    if isinstance(velocity_m_s, bool) or not isinstance(velocity_m_s, (int, float)):
        raise ValueError("surface velocity must be a finite number")
    velocity = float(velocity_m_s)
    if not math.isfinite(velocity):
        raise ValueError("surface velocity must be a finite number")
    if not capabilities.min_surface_velocity <= velocity <= capabilities.max_surface_velocity:
        raise ValueError(
            "surface velocity is outside actuator limits "
            f"[{capabilities.min_surface_velocity}, {capabilities.max_surface_velocity}] m/s"
        )
    return velocity


class _GazeboTransport:
    """Own one long-lived Gazebo Transport node and its wheel publishers."""

    def __init__(self) -> None:
        try:
            from gz.msgs10.double_pb2 import Double
            from gz.transport13 import Node
        except ImportError as exc:
            raise RuntimeError(
                "Gazebo wheel control requires the Gazebo Harmonic Python bindings; "
                "install python3-gz-transport13 in the simulation environment"
            ) from exc

        self._double_type = Double
        self._node = Node()
        self._publishers: dict[str, Callable[[str, float], None]] = {}
        self._lock = threading.Lock()

    def publisher_for(self, topic: str) -> Callable[[str, float], None]:
        with self._lock:
            existing = self._publishers.get(topic)
            if existing is not None:
                return existing

            publisher = self._node.advertise(topic, self._double_type)
            if publisher is None:
                raise RuntimeError(f"Could not advertise Gazebo wheel command topic: {topic}")

            message = self._double_type()
            publish_lock = threading.Lock()

            def publish(_topic: str, angular_velocity_rad_s: float) -> None:
                with publish_lock:
                    message.data = angular_velocity_rad_s
                    if not publisher.publish(message):
                        raise RuntimeError(f"Gazebo Transport rejected wheel command on {topic}")

            self._publishers[topic] = publish
            return publish


_gazebo_transport: _GazeboTransport | None = None
_gazebo_transport_lock = threading.Lock()


def _gazebo_transport_publisher(topic: str) -> Callable[[str, float], None]:
    """Get a publisher backed by the process-wide, persistent Gazebo node."""
    global _gazebo_transport
    with _gazebo_transport_lock:
        if _gazebo_transport is None:
            _gazebo_transport = _GazeboTransport()
        return _gazebo_transport.publisher_for(topic)


class GazeboWheelActuator:
    """
    Publish angular-velocity commands through a persistent Gazebo Transport publisher.
    """

    def __init__(
        self,
        topic: str,
        radius_m: float,
        capabilities: WheelActuatorCapabilities,
        *,
        publisher: Callable[[str, float], Any] | None = None,
    ) -> None:
        if not topic.strip():
            raise ValueError("Gazebo topic must not be empty")
        if not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError("wheel radius must be finite and positive")
        self._topic = topic
        self._radius_m = radius_m
        self._capabilities = capabilities
        self._publisher = publisher or _gazebo_transport_publisher(topic)

    @property
    def capabilities(self) -> WheelActuatorCapabilities:
        return self._capabilities

    async def set_surface_velocity(self, velocity_m_s: float) -> None:
        velocity = _validate_velocity(velocity_m_s, self._capabilities)
        await self._publish(self._topic, velocity / self._radius_m)

    async def stop(self) -> None:
        await self._publish(self._topic, 0.0)

    async def close(self) -> None:
        await self.stop()

    async def _publish(self, topic: str, angular_velocity_rad_s: float) -> None:
        result = self._publisher(topic, angular_velocity_rad_s)
        if asyncio.iscoroutine(result):
            await result


class MX1616WheelActuator:
    """
    Drive one MX1616 H-bridge using complementary PWM on its two inputs.
    """

    def __init__(
        self,
        in1_pin: int,
        in2_pin: int,
        capabilities: WheelActuatorCapabilities,
        *,
        pwm_frequency_hz: int = 1000,
        output_factory: Callable[..., Any] | None = None,
    ) -> None:
        if in1_pin == in2_pin:
            raise ValueError("MX1616 input pins must be distinct")
        if capabilities.max_surface_velocity <= 0 or not math.isfinite(capabilities.max_surface_velocity):
            raise ValueError("maximum surface velocity must be finite and positive")
        if output_factory is None:
            try:
                from gpiozero import PWMOutputDevice
            except ImportError as exc:
                raise RuntimeError("gpiozero is required for MX1616 motor control") from exc
            output_factory = PWMOutputDevice
        self._capabilities = capabilities
        self._in1 = output_factory(in1_pin, frequency=pwm_frequency_hz, initial_value=0)
        try:
            self._in2 = output_factory(in2_pin, frequency=pwm_frequency_hz, initial_value=0)
        except BaseException:
            self._in1.close()
            raise

    @property
    def capabilities(self) -> WheelActuatorCapabilities:
        return self._capabilities

    async def set_surface_velocity(self, velocity_m_s: float) -> None:
        velocity = _validate_velocity(velocity_m_s, self._capabilities)
        duty = abs(velocity) / self._capabilities.max_surface_velocity
        if velocity > 0:
            self._in1.value = duty
            self._in2.value = 0.0
        elif velocity < 0:
            self._in1.value = 0.0
            self._in2.value = duty
        else:
            self._in1.value = 0.0
            self._in2.value = 0.0

    async def stop(self) -> None:
        self._in1.value = 0.0
        self._in2.value = 0.0

    async def close(self) -> None:
        try:
            await self.stop()
        finally:
            self._in1.close()
            self._in2.close()


class WheelActuatorLinearizer:
    """
    Compensate a motor's dead zone and non-linear response around any wheel actuator.

    The wanted surface speed is turned into the speed the wrapped actuator must be
    asked for so that the wheel really reaches the wanted one. The wrapped actuator
    maps a speed to a PWM duty in a straight line (duty = speed / max speed), so the
    corrected speed is simply ``max speed * duty``.
    """

    def __init__(self, actuator: WheelActuator, response: MotorResponse) -> None:
        self._actuator = actuator
        self._response = response

    @property
    def capabilities(self) -> WheelActuatorCapabilities:
        return self._actuator.capabilities

    async def set_surface_velocity(self, velocity_m_s: float) -> None:
        capabilities = self._actuator.capabilities
        velocity = _validate_velocity(velocity_m_s, capabilities)
        duty = self._response.duty_for(abs(velocity) / capabilities.max_surface_velocity)
        await self._actuator.set_surface_velocity(math.copysign(duty * capabilities.max_surface_velocity, velocity))

    async def stop(self) -> None:
        await self._actuator.stop()

    async def close(self) -> None:
        await self._actuator.close()


class SimulatedMotorWheelActuator:
    """
    Make a simulated wheel behave like the real motor (dead zone, non-linear speed).

    It receives what a real motor would receive, a PWM duty expressed as a speed
    (``max speed * duty``), and passes on the speed the motor would really reach.
    Place it between ``WheelActuatorLinearizer`` and the simulated actuator.
    """

    def __init__(self, actuator: WheelActuator, motor: MotorResponse) -> None:
        self._actuator = actuator
        self._motor = motor

    @property
    def capabilities(self) -> WheelActuatorCapabilities:
        return self._actuator.capabilities

    async def set_surface_velocity(self, velocity_m_s: float) -> None:
        capabilities = self._actuator.capabilities
        velocity = _validate_velocity(velocity_m_s, capabilities)
        reached = self._motor.speed_fraction_for(abs(velocity) / capabilities.max_surface_velocity)
        await self._actuator.set_surface_velocity(math.copysign(reached * capabilities.max_surface_velocity, velocity))

    async def stop(self) -> None:
        await self._actuator.stop()

    async def close(self) -> None:
        await self._actuator.close()


def create_wheel_actuator(
    driver: MotorDriverConfiguration,
    radius_m: float,
    capabilities: WheelActuatorCapabilities,
    *,
    output_factory: Callable[..., Any] | None = None,
) -> WheelActuator:
    """
    Construct the actuator selected by a wheel's driver config.

    Chain on the robot:      linearizer -> MX1616 -> real motor.
    Chain in the simulation: linearizer -> simulated motor -> Gazebo.
    Layers that would do nothing (no dead zone, no calibration) are left out.
    """
    identity = MotorResponse()
    if isinstance(driver, GazeboMotorDriverConfiguration):
        actuator: WheelActuator = GazeboWheelActuator(driver.topic, radius_m, capabilities)
        motor = driver.motor if driver.motor is not None else driver.response
        if motor != identity:
            actuator = SimulatedMotorWheelActuator(actuator, motor)
    elif isinstance(driver, MX1616MotorDriverConfiguration):
        actuator = MX1616WheelActuator(driver.in1_pin, driver.in2_pin, capabilities, output_factory=output_factory)
    else:
        raise TypeError(f"Unsupported motor driver configuration: {type(driver).__name__}")

    if driver.response != identity:
        actuator = WheelActuatorLinearizer(actuator, driver.response)
    return actuator
