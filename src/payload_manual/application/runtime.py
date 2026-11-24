from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass

from loguru import logger

from payload.application.omni_wheel_controller import OmniWheelController
from payload.domain.body_velocity import BodyVelocity
from payload.domain.positioning.events import DistanceFailure, DistanceSample
from payload_manual.infrastructure.d3kble.manual_control_peer import D3kbleManualControlPeer
from payload.domain.positioning.distance_sensor import DistanceSensor


@dataclass(frozen=True, slots=True)
class ManualControlLimits:
    max_linear_speed_m_s: float = 0.15
    max_angular_speed_rad_s: float = 0.8
    command_timeout_ms: int = 300
    control_rate_hz: int = 50

    def __post_init__(self) -> None:
        for name in ("max_linear_speed_m_s", "max_angular_speed_rad_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            isinstance(self.command_timeout_ms, bool)
            or not isinstance(self.command_timeout_ms, int)
            or not 100 <= self.command_timeout_ms <= 5000
        ):
            raise ValueError("command_timeout_ms must be an integer in [100, 5000]")
        if (
            isinstance(self.control_rate_hz, bool)
            or not isinstance(self.control_rate_hz, int)
            or not 1 <= self.control_rate_hz <= 100
        ):
            raise ValueError("control_rate_hz must be an integer in [1, 100]")


class ManualControlRuntime:
    """Run the independent, open-loop manual payload-control application."""

    def __init__(
        self,
        peer: D3kbleManualControlPeer,
        wheel_controller: OmniWheelController,
        distance_sensor: DistanceSensor,
        limits: ManualControlLimits,
    ) -> None:
        self._peer = peer
        self._wheel_controller = wheel_controller
        self._distance_sensor = distance_sensor
        self._limits = limits
        self._armed = False
        self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
        self._last_command_at: float | None = None
        self._session_id: str | None = None
        self._last_sequence = -1
        self._outputs_may_be_active = False

    async def run(self, stop_event: asyncio.Event) -> None:
        await self._peer.start()
        await self._wheel_controller.stop()
        try:
            async with asyncio.TaskGroup() as group:
                commands = group.create_task(self._consume_commands(), name="manual-control-commands")
                control = group.create_task(self._control_loop(), name="manual-control-loop")
                ranging = group.create_task(self._consume_distance_events(), name="manual-control-uwb")
                await stop_event.wait()
                commands.cancel()
                control.cancel()
                ranging.cancel()
        finally:
            self._armed = False
            self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
            try:
                await self._wheel_controller.stop()
            finally:
                await self._distance_sensor.stop_ranging()

    async def _consume_commands(self) -> None:
        async for command in self._peer.events():
            if command.session_id != self._session_id:
                if command.command_type == "status":
                    # A newly connected UI may inspect state without taking over the
                    # current session. It can still issue DISARM as a safety action.
                    await self._peer.send_status(command.session_id, command.sequence, self._armed, False)
                    continue
                if command.command_type not in {"arm", "disarm"}:
                    logger.warning("ignoring command from an inactive manual-control session")
                    continue
                self._session_id = command.session_id
                self._last_sequence = -1
                logger.info("manual controller session changed")

            if command.sequence <= self._last_sequence:
                logger.warning("ignoring stale manual-control sequence {}", command.sequence)
                continue
            self._last_sequence = command.sequence
            now = asyncio.get_running_loop().time()

            if command.command_type == "status":
                await self._peer.send_status(command.session_id, command.sequence, self._armed, True)
            elif command.command_type == "arm":
                self._armed = True
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = now
                self._outputs_may_be_active = True
                await self._peer.send_status(command.session_id, command.sequence, self._armed, True)
                logger.info("manual payload control armed")
            elif command.command_type == "disarm":
                self._armed = False
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = None
                await self._wheel_controller.stop()
                self._outputs_may_be_active = False
                await self._peer.send_status(command.session_id, command.sequence, self._armed, True)
                logger.info("manual payload control disarmed")
            elif command.command_type == "stop":
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = now if self._armed else None
                if self._armed:
                    self._outputs_may_be_active = True
                else:
                    await self._wheel_controller.stop()
                logger.debug("manual payload stop received")
            elif command.command_type == "drive":
                if not self._armed:
                    logger.warning("ignoring drive command while manual control is disarmed")
                    continue
                assert command.velocity is not None
                if not self._within_limits(command.velocity):
                    logger.warning("drive command exceeded configured manual-control limits; disarming")
                    self._armed = False
                    self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                    self._last_command_at = None
                    await self._wheel_controller.stop()
                    self._outputs_may_be_active = False
                    await self._peer.send_status(command.session_id, command.sequence, self._armed, True)
                    continue
                self._desired_velocity = command.velocity
                self._last_command_at = now
                self._outputs_may_be_active = True

    def _within_limits(self, velocity: BodyVelocity) -> bool:
        linear_speed = math.hypot(velocity.vx_m_s, velocity.vy_m_s)
        return (
            linear_speed <= self._limits.max_linear_speed_m_s
            and abs(velocity.yaw_rate_rad_s) <= self._limits.max_angular_speed_rad_s
        )

    async def _control_loop(self) -> None:
        interval = 1.0 / self._limits.control_rate_hz
        timeout_s = self._limits.command_timeout_ms / 1000.0
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(interval)
            now = loop.time()
            if self._armed and (
                self._last_command_at is None or now - self._last_command_at > timeout_s
            ):
                logger.warning("manual-control command watchdog expired; stopping and disarming")
                self._armed = False
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = None
                await self._wheel_controller.stop()
                self._outputs_may_be_active = False
                if self._session_id is not None:
                    await self._peer.send_status(self._session_id, self._last_sequence, self._armed, True)
                continue

            if self._armed:
                await self._wheel_controller.set_body_velocity(self._desired_velocity)
            elif self._outputs_may_be_active:
                await self._wheel_controller.stop()
                self._outputs_may_be_active = False

    async def _consume_distance_events(self) -> None:
        await self._distance_sensor.start_ranging()
        async for event in self._distance_sensor.events():
            if isinstance(event, DistanceSample):
                logger.debug(
                    "UWB distance to %s: %.3f m (sequence %s)",
                    event.anchor_id,
                    event.distance_m,
                    event.sequence,
                )
            elif isinstance(event, DistanceFailure):
                logger.warning("UWB range to %s failed: %s", event.anchor_id, event.reason.message)
            else:
                logger.warning("ignoring unsupported UWB event: {}", event)
