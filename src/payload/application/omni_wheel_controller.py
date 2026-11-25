from __future__ import annotations

import asyncio
import math
from collections.abc import Sequence

from payload.domain.body_velocity import BodyVelocity
from payload.domain.wheel_actuator import WheelActuator
from payload.domain.wheel_geometry import WheelTransform2D


class OmniWheelController:
    """
    Convert body-frame velocity setpoints into individual wheel commands.
    """

    def __init__(
        self,
        wheels: Sequence[WheelTransform2D],
        actuators: Sequence[WheelActuator],
    ) -> None:
        if len(wheels) < 3:
            raise ValueError("at least three wheels are required for planar omni-directional control")
        if len(wheels) != len(actuators):
            raise ValueError("wheel configuration and actuator counts must match")
        self._wheels = tuple(wheels)
        self._actuators = tuple(actuators)
        self._last_surface_velocities: tuple[float, ...] | None = None
        self._lock = asyncio.Lock()
        self._closed = False

    @property
    def wheel_count(self) -> int:
        return len(self._wheels)

    def _calculate_surface_velocity_components(
        self, velocity: BodyVelocity
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Return the translation and rotation contributions for each wheel."""
        linear_values: list[float] = []
        angular_values: list[float] = []
        for wheel in self._wheels:
            rolling_x = math.cos(wheel.yaw)
            rolling_y = math.sin(wheel.yaw)
            linear_values.append(rolling_x * velocity.vx_m_s + rolling_y * velocity.vy_m_s)
            angular_values.append(velocity.yaw_rate_rad_s * (-rolling_x * wheel.y + rolling_y * wheel.x))
        return tuple(linear_values), tuple(angular_values)

    def calculate_surface_velocities(self, velocity: BodyVelocity) -> tuple[float, ...]:
        """Return signed wheel surface speeds in m/s for a body velocity."""
        linear, angular = self._calculate_surface_velocity_components(velocity)
        return tuple(linear_value + angular_value for linear_value, angular_value in zip(linear, angular))

    def _translation_scale_interval(
        self,
        linear_values: tuple[float, ...],
        angular_values: tuple[float, ...],
        angular_scale: float,
    ) -> tuple[float, float] | None:
        """Return feasible translation-scale bounds for a fixed yaw scale."""
        lower, upper = 0.0, 1.0
        for linear, angular, actuator in zip(linear_values, angular_values, self._actuators):
            maximum = actuator.capabilities.max_surface_velocity
            angular = angular * angular_scale

            if linear == 0.0:
                if abs(angular) > maximum:
                    return None
                continue

            first = (-maximum - angular) / linear
            second = (maximum - angular) / linear
            lower = max(lower, min(first, second))
            upper = min(upper, max(first, second))
            if lower > upper:
                return None

        return lower, upper

    def _limit_surface_velocities(
        self,
        linear_values: tuple[float, ...],
        angular_values: tuple[float, ...],
    ) -> tuple[float, ...]:
        """Maximize yaw first, then maximize the translation scale."""
        if len(linear_values) != len(self._actuators) or len(angular_values) != len(self._actuators):
            raise ValueError("wheel velocity components and actuator counts must match")

        # First try to retain the full requested angular velocity.
        angular_scale = 1.0
        interval = self._translation_scale_interval(linear_values, angular_values, angular_scale)

        # If no translation scale can make full yaw feasible, find the largest
        # feasible yaw scale. Feasibility is monotonic because (alpha, beta)=(0,0)
        # is feasible and the actuator constraints are linear.
        if interval is None:
            low, high = 0.0, 1.0
            for _ in range(60):
                candidate = (low + high) / 2.0
                if self._translation_scale_interval(linear_values, angular_values, candidate) is None:
                    high = candidate
                else:
                    low = candidate
            angular_scale = low
            interval = self._translation_scale_interval(linear_values, angular_values, angular_scale)

        if interval is None:
            raise RuntimeError("unable to find feasible wheel velocity scales")

        # With the maximum feasible yaw scale fixed, use as much translation
        # as possible while preserving the requested translation direction.
        translation_scale = interval[1]
        maximums = tuple(actuator.capabilities.max_surface_velocity for actuator in self._actuators)
        return tuple(
            max(
                -maximum,
                min(
                    maximum,
                    translation * translation_scale + rotation * angular_scale,
                ),
            )
            for translation, rotation, maximum in zip(linear_values, angular_values, maximums)
        )

    async def set_body_velocity(self, velocity: BodyVelocity) -> tuple[float, ...]:
        if self._closed:
            raise RuntimeError("wheel controller is closed")
        linear, angular = self._calculate_surface_velocity_components(velocity)
        limited = self._limit_surface_velocities(linear, angular)
        async with self._lock:
            if self._closed:
                raise RuntimeError("wheel controller is closed")
            if limited == self._last_surface_velocities:
                return limited
            results = await asyncio.gather(
                *(actuator.set_surface_velocity(value) for actuator, value in zip(self._actuators, limited)),
                return_exceptions=True,
            )
            errors = [result for result in results if isinstance(result, BaseException)]
            if errors:
                await self._stop_all_safely()
                self._last_surface_velocities = tuple(0.0 for _ in self._actuators)
                if len(errors) == 1:
                    raise errors[0]
                raise BaseExceptionGroup("one or more wheel commands failed", errors)
            self._last_surface_velocities = limited
        return limited

    async def stop(self) -> None:
        async with self._lock:
            await self._stop_all_safely()
            self._last_surface_velocities = tuple(0.0 for _ in self._actuators)

    async def _stop_all_safely(self) -> None:
        results = await asyncio.gather(
            *(actuator.stop() for actuator in self._actuators),
            return_exceptions=True,
        )
        errors = [result for result in results if isinstance(result, BaseException)]
        if errors:
            raise BaseExceptionGroup("one or more wheel actuators failed to stop", errors)

    async def close(self) -> None:
        if self._closed:
            return
        async with self._lock:
            if self._closed:
                return
            errors: list[BaseException] = []
            try:
                await self._stop_all_safely()
            except BaseException as exc:
                errors.append(exc)
            results = await asyncio.gather(
                *(actuator.close() for actuator in self._actuators),
                return_exceptions=True,
            )
            errors.extend(result for result in results if isinstance(result, BaseException))
            self._last_surface_velocities = tuple(0.0 for _ in self._actuators)
            self._closed = True
            if errors:
                raise BaseExceptionGroup("failed to close one or more wheel actuators", errors)
