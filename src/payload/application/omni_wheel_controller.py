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

    def calculate_surface_velocities(self, velocity: BodyVelocity) -> tuple[float, ...]:
        """Return signed wheel surface speeds in m/s for a body velocity."""
        values: list[float] = []
        for wheel in self._wheels:
            transform = wheel
            local_vx = velocity.vx_m_s - velocity.yaw_rate_rad_s * transform.y
            local_vy = velocity.vy_m_s + velocity.yaw_rate_rad_s * transform.x
            rolling_x = math.cos(transform.yaw)
            rolling_y = math.sin(transform.yaw)
            values.append(rolling_x * local_vx + rolling_y * local_vy)
        return tuple(values)

    def _limit_surface_velocities(self, values: tuple[float, ...]) -> tuple[float, ...]:
        scale = 1.0
        for value, actuator in zip(values, self._actuators):
            maximum = actuator.capabilities.max_surface_velocity
            if abs(value) > maximum:
                scale = min(scale, maximum / abs(value))
        if scale == 1.0:
            return values
        return tuple(value * scale for value in values)

    async def set_body_velocity(self, velocity: BodyVelocity) -> tuple[float, ...]:
        if self._closed:
            raise RuntimeError("wheel controller is closed")
        requested = self.calculate_surface_velocities(velocity)
        limited = self._limit_surface_velocities(requested)
        async with self._lock:
            if self._closed:
                raise RuntimeError("wheel controller is closed")
            if limited == self._last_surface_velocities:
                return limited
            results = await asyncio.gather(
                *(
                    actuator.set_surface_velocity(value)
                    for actuator, value in zip(self._actuators, limited)
                ),
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
