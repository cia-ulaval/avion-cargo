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
    max_angular_speed_rad_s: float = 0.8
    command_timeout_ms: int = 300
    control_rate_hz: int = 50

    def __post_init__(self) -> None:
        for name in ("max_angular_speed_rad_s",):
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
        self._arm_started_at: float | None = None
        self._session_id: str | None = None
        self._last_sequence = -1
        self._outputs_may_be_active = False
        self._next_stop_retry_at = 0.0

    async def run(self, stop_event: asyncio.Event) -> None:
        await self._peer.start()
        await self._stop_wheels_safely("runtime startup")
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
            self._last_command_at = None
            self._arm_started_at = None
            try:
                await self._wheel_controller.stop()
            except Exception:
                logger.exception("could not stop manual payload wheel actuators during shutdown")
            try:
                await self._distance_sensor.stop_ranging()
            except Exception:
                logger.exception("could not stop manual payload UWB ranging during shutdown")

    async def _stop_wheels_safely(self, reason: str) -> bool:
        """Attempt a stop without allowing an actuator error to kill the runtime."""
        try:
            await self._wheel_controller.stop()
        except Exception as exc:
            logger.error("wheel stop failed after {}: {}", reason, exc)
            self._outputs_may_be_active = True
            self._next_stop_retry_at = asyncio.get_running_loop().time() + 0.2
            return False

        self._outputs_may_be_active = False
        self._next_stop_retry_at = 0.0
        return True

    async def _report_status_safely(
        self, session_id: str | None = None, sequence: int | None = None
    ) -> None:
        session_id = self._session_id if session_id is None else session_id
        sequence = self._last_sequence if sequence is None else sequence
        if session_id is None:
            return
        try:
            await self._peer.send_status(session_id, sequence, self._armed, True)
        except Exception:
            logger.exception("could not report manual-control state")

    async def _disarm_after_wheel_failure(self, error: Exception) -> None:
        """Fail safe on an actuator error while keeping the application alive."""
        logger.error("wheel command failed; disarming manual control: {}", error)
        self._armed = False
        self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
        self._last_command_at = None
        self._arm_started_at = None
        await self._stop_wheels_safely("wheel command failure")
        await self._report_status_safely()

    async def _consume_commands(self) -> None:
        async for command in self._peer.events():
            if command.session_id != self._session_id:
                if command.command_type == "status":
                    # A newly connected UI may inspect state without taking over the
                    # current session. It can still issue DISARM as a safety action.
                    try:
                        await self._peer.send_status(command.session_id, command.sequence, self._armed, False)
                    except Exception:
                        logger.exception("could not report inactive-session status")
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
                await self._report_status_safely(command.session_id, command.sequence)
            elif command.command_type == "arm":
                self._armed = True
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                # The controller only starts its drive heartbeat after it has
                # received this state acknowledgement. Give that round trip a
                # bounded startup window before enforcing the drive watchdog.
                self._last_command_at = None
                self._arm_started_at = now
                self._outputs_may_be_active = True
                await self._report_status_safely(command.session_id, command.sequence)
                logger.info("manual payload control armed")
            elif command.command_type == "disarm":
                self._armed = False
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = None
                self._arm_started_at = None
                await self._stop_wheels_safely("disarm command")
                await self._report_status_safely(command.session_id, command.sequence)
                logger.info("manual payload control disarmed")
            elif command.command_type == "stop":
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = now if self._armed else None
                if self._armed:
                    self._arm_started_at = None
                    self._outputs_may_be_active = True
                else:
                    await self._stop_wheels_safely("stop command while disarmed")
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
                    self._arm_started_at = None
                    await self._stop_wheels_safely("out-of-limits command")
                    await self._report_status_safely(command.session_id, command.sequence)
                    continue
                self._desired_velocity = command.velocity
                self._last_command_at = now
                self._arm_started_at = None
                self._outputs_may_be_active = True

    def _within_limits(self, velocity: BodyVelocity) -> bool:
        # Wheel surface-speed capabilities are the sole translational limit.
        return abs(velocity.yaw_rate_rad_s) <= self._limits.max_angular_speed_rad_s

    async def _control_loop(self) -> None:
        interval = 1.0 / self._limits.control_rate_hz
        timeout_s = self._limits.command_timeout_ms / 1000.0
        loop = asyncio.get_running_loop()
        # Arm confirmation requires a BLE status round trip before the UI
        # starts its drive heartbeat. Do not apply the short motion watchdog
        # during that handshake, but still disarm if no first drive arrives.
        arm_startup_timeout_s = max(1.5, timeout_s * 5.0)
        while True:
            await asyncio.sleep(interval)
            now = loop.time()
            startup_expired = (
                self._armed
                and self._last_command_at is None
                and self._arm_started_at is not None
                and now - self._arm_started_at > arm_startup_timeout_s
            )
            drive_watchdog_expired = (
                self._armed
                and self._last_command_at is not None
                and now - self._last_command_at > timeout_s
            )
            if startup_expired or drive_watchdog_expired:
                reason = "initial drive command did not arrive after arm" if startup_expired else "drive heartbeat expired"
                logger.warning("manual-control watchdog expired ({}); stopping and disarming", reason)
                self._armed = False
                self._desired_velocity = BodyVelocity(0.0, 0.0, 0.0)
                self._last_command_at = None
                self._arm_started_at = None
                await self._stop_wheels_safely("command watchdog")
                await self._report_status_safely()
                continue

            if self._armed:
                try:
                    await self._wheel_controller.set_body_velocity(self._desired_velocity)
                except Exception as exc:
                    # A single Gazebo/GPIO write failure must not escape the
                    # TaskGroup and terminate the payload process. Disarm and
                    # attempt a best-effort stop instead.
                    await self._disarm_after_wheel_failure(exc)
            elif self._outputs_may_be_active and now >= self._next_stop_retry_at:
                await self._stop_wheels_safely("disarmed stop retry")

    async def _consume_distance_events(self) -> None:
        await self._distance_sensor.start_ranging()
        async for event in self._distance_sensor.events():
            if isinstance(event, DistanceSample):
                logger.debug(
                    "UWB distance to {}: {:.3f} m (sequence {})",
                    event.anchor_id,
                    event.distance_m,
                    event.sequence,
                )
            elif isinstance(event, DistanceFailure):
                logger.warning("UWB range to {} failed: {}", event.anchor_id, event.reason.message)
            else:
                logger.warning("ignoring unsupported UWB event: {}", event)
