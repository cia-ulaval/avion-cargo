from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass

from loguru import logger

from payload.application.omni_wheel_controller import OmniWheelController
from payload.application.positioning_runtime import PayloadPositioningRuntime
from payload.domain.body_velocity import BodyVelocity
from payload.domain.positioning.distance_sensor import DistanceSensor
from payload.domain.positioning.events import DistanceFailure, DistanceSample
from payload_manual.domain.manual_control import ReferenceVelocity
from payload_manual.infrastructure.d3kble.manual_control_peer import D3kbleManualControlPeer


@dataclass(frozen=True, slots=True)
class ManualControlLimits:
    # One drive heartbeat may wait behind an application ACK on the
    # controller's single-flight BLE channel. Leave margin for observed
    # delivery jitter while still disarming promptly if commands stop.
    command_timeout_ms: int = 1000

    def __post_init__(self) -> None:
        if (
            isinstance(self.command_timeout_ms, bool)
            or not isinstance(self.command_timeout_ms, int)
            or not 100 <= self.command_timeout_ms <= 5000
        ):
            raise ValueError("command_timeout_ms must be an integer in [100, 5000]")


class ManualControlRuntime:
    """Run the independent, event-driven manual payload-control application."""

    def __init__(
        self,
        peer: D3kbleManualControlPeer,
        wheel_controller: OmniWheelController,
        distance_sensor: DistanceSensor,
        limits: ManualControlLimits,
        positioning_runtime: PayloadPositioningRuntime | None = None,
    ) -> None:
        self._peer = peer
        self._wheel_controller = wheel_controller
        self._distance_sensor = distance_sensor
        self._limits = limits
        self._positioning = positioning_runtime
        self._armed = False
        self._current_setpoint: BodyVelocity | None = None
        self._session_id: str | None = None
        self._last_sequence = -1

        # A command lease expires only when the UI stops refreshing its setpoint.
        # This timer does not refresh or rewrite wheel outputs.
        self._watchdog_deadline: float | None = None
        self._watchdog_reason: str | None = None
        self._watchdog_changed = asyncio.Event()
        self._state_lock = asyncio.Lock()

        self._outputs_may_be_active = False
        self._stop_retry_task: asyncio.Task[None] | None = None

    async def run(self, stop_event: asyncio.Event) -> None:
        await self._peer.start()
        await self._stop_wheels_safely("runtime startup")
        try:
            async with asyncio.TaskGroup() as group:
                commands = group.create_task(self._consume_commands(), name="manual-control-commands")
                watchdog = group.create_task(self._watch_command_lease(), name="manual-control-watchdog")
                ranging_ready = asyncio.Event()
                ranging = group.create_task(
                    self._consume_distance_events(ranging_ready), name="manual-control-uwb"
                )
                await ranging_ready.wait()
                # events() has been advanced once and has installed its D3K
                # subscription before the firmware schedule is started.
                await self._distance_sensor.start_ranging()
                await stop_event.wait()
                commands.cancel()
                watchdog.cancel()
                ranging.cancel()
        finally:
            self._armed = False
            self._clear_watchdog()
            await self._cancel_stop_retry()
            try:
                await self._wheel_controller.stop()
            except Exception:
                logger.exception("could not stop manual payload wheel actuators during shutdown")
            try:
                await self._distance_sensor.stop_ranging()
            except Exception:
                logger.exception("could not stop manual payload UWB ranging during shutdown")

    def _set_watchdog(self, deadline: float, reason: str) -> None:
        self._watchdog_deadline = deadline
        self._watchdog_reason = reason
        self._watchdog_changed.set()

    def _clear_watchdog(self) -> None:
        self._watchdog_deadline = None
        self._watchdog_reason = None
        self._watchdog_changed.set()

    async def _watch_command_lease(self) -> None:
        """Wait for the current command lease to expire; do not poll wheel outputs."""
        loop = asyncio.get_running_loop()
        while True:
            self._watchdog_changed.clear()
            deadline = self._watchdog_deadline
            if deadline is None:
                await self._watchdog_changed.wait()
                continue

            remaining = deadline - loop.time()
            if remaining > 0:
                try:
                    await asyncio.wait_for(self._watchdog_changed.wait(), timeout=remaining)
                except asyncio.TimeoutError:
                    pass
                else:
                    continue

            async with self._state_lock:
                # A command may have renewed or cleared the lease while this
                # task was waking up or waiting for the state lock.
                if not self._armed or self._watchdog_deadline != deadline:
                    continue
                if loop.time() < deadline:
                    continue

                reason = self._watchdog_reason or "command watchdog"
                self._clear_watchdog()
                self._armed = False
                logger.warning("manual-control watchdog expired ({}); stopping and disarming", reason)
                await self._stop_wheels_safely("command watchdog")
                await self._report_status_safely()

    async def _cancel_stop_retry(self) -> None:
        task = self._stop_retry_task
        self._stop_retry_task = None
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    def _schedule_stop_retry(self, reason: str) -> None:
        if self._stop_retry_task is None:
            self._stop_retry_task = asyncio.create_task(
                self._retry_stop_until_safe(reason),
                name="manual-control-stop-retry",
            )

    async def _retry_stop_until_safe(self, reason: str) -> None:
        try:
            while self._outputs_may_be_active:
                await asyncio.sleep(0.2)
                try:
                    await self._wheel_controller.stop()
                except Exception as exc:
                    logger.error("wheel stop retry failed after {}: {}", reason, exc)
                else:
                    self._outputs_may_be_active = False
                    self._current_setpoint = BodyVelocity(0.0, 0.0, 0.0)
                    return
        finally:
            if self._stop_retry_task is asyncio.current_task():
                self._stop_retry_task = None

    async def _stop_wheels_safely(self, reason: str) -> bool:
        """Attempt a stop and retry asynchronously if an actuator write fails."""
        if self._positioning is not None:
            # The last drive setpoint is not a valid motion observation after a
            # STOP, DISARM, watchdog expiry, or actuator fault.
            self._positioning.set_body_velocity(BodyVelocity(0.0, 0.0, 0.0))
        try:
            await self._wheel_controller.stop()
        except Exception as exc:
            logger.error("wheel stop failed after {}: {}", reason, exc)
            self._outputs_may_be_active = True
            self._current_setpoint = None
            self._schedule_stop_retry(reason)
            return False

        self._outputs_may_be_active = False
        self._current_setpoint = BodyVelocity(0.0, 0.0, 0.0)
        await self._cancel_stop_retry()
        return True

    async def _report_status_safely(self, session_id: str | None = None, sequence: int | None = None) -> None:
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
        self._clear_watchdog()
        await self._stop_wheels_safely("wheel command failure")
        await self._report_status_safely()

    async def _consume_commands(self) -> None:
        timeout_s = self._limits.command_timeout_ms / 1000.0
        # Keep the initial handshake grace bounded even when the motion lease is longer.
        arm_startup_timeout_s = max(1.5, timeout_s * 2.0)
        loop = asyncio.get_running_loop()

        async for command in self._peer.events():
            async with self._state_lock:
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

                if command.sequence <= self._last_sequence:
                    logger.warning("ignoring stale manual-control sequence {}", command.sequence)
                    continue
                self._last_sequence = command.sequence

                if command.command_type == "status":
                    await self._report_status_safely(command.session_id, command.sequence)

                elif command.command_type == "arm":
                    # Arming always starts from a confirmed zero output. Only
                    # acknowledge the arm after the stop request succeeds.
                    self._armed = False
                    self._clear_watchdog()
                    stopped = await self._stop_wheels_safely("arm command")
                    if stopped:
                        self._armed = True
                        self._set_watchdog(
                            loop.time() + arm_startup_timeout_s,
                            "initial drive command did not arrive after arm",
                        )
                    await self._report_status_safely(command.session_id, command.sequence)
                    if self._armed:
                        logger.info("manual payload control armed")

                elif command.command_type == "disarm":
                    self._armed = False
                    self._clear_watchdog()
                    await self._stop_wheels_safely("disarm command")
                    await self._report_status_safely(command.session_id, command.sequence)
                    logger.info("manual payload control disarmed")

                elif command.command_type == "stop":
                    if self._armed:
                        self._set_watchdog(
                            loop.time() + timeout_s,
                            "drive heartbeat expired",
                        )
                    await self._stop_wheels_safely("stop command")
                    logger.debug("manual payload stop received")

                elif command.command_type == "drive":
                    if not self._armed:
                        logger.warning("ignoring drive command while manual control is disarmed")
                        continue
                    assert command.velocity is not None
                    self._set_watchdog(
                        loop.time() + timeout_s,
                        "drive heartbeat expired",
                    )
                    body_velocity = self._to_body_velocity(command.velocity)
                    if body_velocity != self._current_setpoint:
                        try:
                            # Apply changed setpoints immediately. Repeated drive messages
                            # renew the command lease without reissuing an unchanged target.
                            await self._wheel_controller.set_body_velocity(body_velocity)
                            self._current_setpoint = body_velocity
                            if self._positioning is not None:
                                self._positioning.set_body_velocity(body_velocity)
                            self._outputs_may_be_active = body_velocity != BodyVelocity(0.0, 0.0, 0.0)
                        except Exception as exc:
                            logger.exception(
                                "[manual-actuator] drive failed: sequence={} body_velocity={} error={}",
                                command.sequence,
                                body_velocity,
                                exc,
                            )
                            self._current_setpoint = None
                            await self._disarm_after_wheel_failure(exc)

    def _to_body_velocity(self, velocity: ReferenceVelocity) -> BodyVelocity:
        """Rotate anchor-reference translation into the payload body frame."""
        angle = 0.0
        if self._positioning is not None:
            angle = self._positioning.snapshot.heading.angle_rad
        c, s = math.cos(angle), math.sin(angle)
        return BodyVelocity(
            c * velocity.vx_m_s + s * velocity.vy_m_s,
            -s * velocity.vx_m_s + c * velocity.vy_m_s,
            velocity.yaw_rate_rad_s,
        )

    async def _consume_distance_events(self, ready: asyncio.Event) -> None:
        iterator = self._distance_sensor.events()
        pending = asyncio.create_task(iterator.__anext__())
        # The async generator subscribes synchronously before its first await.
        await asyncio.sleep(0)
        ready.set()
        window_started = time.monotonic()
        event_counts: dict[str, int] = {"sample": 0, "failure": 0}
        process_total_ms = 0.0
        process_max_ms = 0.0
        estimate_updates = 0
        last_device_us: int | None = None
        last_event_at = time.monotonic()
        last_reported_estimate_sequence: int | None = None
        try:
            while True:
                try:
                    event = await pending
                except StopAsyncIteration:
                    logger.warning("[manual-uwb] UWB event stream ended")
                    return
                if not isinstance(event, (DistanceSample, DistanceFailure)):
                    logger.warning("[manual-uwb] ignoring unsupported distance event: {}", event)
                    pending = asyncio.create_task(iterator.__anext__())
                    continue

                event_type = "sample" if isinstance(event, DistanceSample) else "failure"
                event_counts[event_type] += 1
                event_started = time.monotonic()
                device_delta_us = (
                    event.device_monotonic_us - last_device_us
                    if event.device_monotonic_us is not None and last_device_us is not None
                    else None
                )
                if event.device_monotonic_us is not None:
                    last_device_us = event.device_monotonic_us

                previous_rejected = 0
                previous_sequence = None
                if self._positioning is not None:
                    previous_snapshot = self._positioning.snapshot
                    previous_rejected = previous_snapshot.rejected_range_count
                    previous = previous_snapshot.tag_estimate
                    previous_sequence = previous.sequence if previous is not None else None
                    snapshot = self._positioning.process(event)
                    current = snapshot.tag_estimate
                    if current is not None and current.sequence != previous_sequence:
                        estimate_updates += 1
                        last_reported_estimate_sequence = current.sequence
                    rejected_now = snapshot.rejected_range_count
                    if rejected_now > previous_rejected:
                        logger.warning(
                            "[positioning] range rejected as outlier: anchor={} distance_m={} sequence={} "
                            "rejected_total={} range_count={}",
                            event.anchor_id,
                            getattr(event, "distance_m", None),
                            getattr(event, "sequence", None),
                            rejected_now,
                            snapshot.range_count,
                        )
                    if isinstance(event, DistanceFailure) or current is None:
                        logger.warning(
                            "[positioning] no current position: event={} anchor={} status={} "
                            "range_count={} rejected_ranges={} estimator_sequence={} device_us={}",
                            event_type,
                            event.anchor_id,
                            snapshot.status,
                            snapshot.range_count,
                            snapshot.rejected_range_count,
                            previous_sequence,
                            event.device_monotonic_us,
                        )
                    elif current.sequence != previous_sequence:
                        pass
                    elif current is not None:
                        pass
                    await self._peer.send_positioning(snapshot)

                process_ms = (time.monotonic() - event_started) * 1000.0
                process_total_ms += process_ms
                process_max_ms = max(process_max_ms, process_ms)
                last_event_at = time.monotonic()
                if isinstance(event, DistanceSample):
                    pass
                else:
                    logger.warning(
                        "[manual-uwb] consumed failed range: anchor={} reason={} device_us={} "
                        "epoch_us={} received_at={} status={} quality_flags={} processing_ms={:.3f}",
                        event.anchor_id,
                        event.reason.message,
                        event.device_monotonic_us,
                        event.epoch_us,
                        event.received_at,
                        event.status,
                        event.quality_flags,
                        process_ms,
                    )

                now = time.monotonic()
                if now - window_started >= 1.0:
                    event_counts = {"sample": 0, "failure": 0}
                    process_total_ms = 0.0
                    process_max_ms = 0.0
                    estimate_updates = 0
                    window_started = now

                pending = asyncio.create_task(iterator.__anext__())
        finally:
            if not pending.done():
                pending.cancel()
                await asyncio.gather(pending, return_exceptions=True)
            await iterator.aclose()
