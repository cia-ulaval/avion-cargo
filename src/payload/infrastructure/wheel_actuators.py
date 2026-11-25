from __future__ import annotations

import asyncio
import math
from collections.abc import Callable
from typing import Any

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


class GazeboWheelActuator:
    """
    Publish angular-velocity commands to a Gazebo JointController topic.
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
        self._publisher = publisher or self._publish_with_gz

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

    @staticmethod
    async def _publish_with_gz(topic: str, angular_velocity_rad_s: float) -> None:
        process = await asyncio.create_subprocess_exec(
            "gz",
            "topic",
            "-t",
            topic,
            "-m",
            "gz.msgs.Double",
            "-p",
            f"data: {angular_velocity_rad_s:.12g}",
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
        )
        _, stderr = await process.communicate()
        if process.returncode:
            detail = stderr.decode(errors="replace").strip()
            raise RuntimeError(f"Gazebo command publish failed ({process.returncode}): {detail}")


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


def create_wheel_actuator(
    driver: MotorDriverConfiguration,
    radius_m: float,
    capabilities: WheelActuatorCapabilities,
    *,
    output_factory: Callable[..., Any] | None = None,
) -> WheelActuator:
    """Construct the actuator adapter selected by a wheel's driver config."""
    if isinstance(driver, GazeboMotorDriverConfiguration):
        return GazeboWheelActuator(driver.topic, radius_m, capabilities)
    if isinstance(driver, MX1616MotorDriverConfiguration):
        return MX1616WheelActuator(driver.in1_pin, driver.in2_pin, capabilities, output_factory=output_factory)
    raise TypeError(f"Unsupported motor driver configuration: {type(driver).__name__}")
