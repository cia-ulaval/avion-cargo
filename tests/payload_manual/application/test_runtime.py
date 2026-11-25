import asyncio

import pytest

from payload.domain.body_velocity import BodyVelocity
from payload_manual.application.runtime import ManualControlLimits, ManualControlRuntime
from payload_manual.domain.manual_control import ManualControlCommand


class FakePeer:
    def __init__(self):
        self.queue = asyncio.Queue()

    async def start(self):
        pass

    async def send_status(self, session_id, sequence, armed, session_active):
        self.status_messages = getattr(self, "status_messages", [])
        self.status_messages.append((session_id, sequence, armed, session_active))

    async def events(self):
        while True:
            yield await self.queue.get()


class FakeWheelController:
    def __init__(self):
        self.commands = []
        self.stops = 0
        self.fail_next_command = False

    async def stop(self):
        self.stops += 1

    async def set_body_velocity(self, velocity):
        if self.fail_next_command:
            self.fail_next_command = False
            raise RuntimeError("simulated wheel command failure")
        self.commands.append(velocity)


class FakeDistanceSensor:
    def __init__(self):
        self.started = False
        self.stopped = False

    async def start_ranging(self):
        self.started = True

    async def stop_ranging(self):
        self.stopped = True

    async def events(self):
        while True:
            await asyncio.sleep(1)
            yield None


@pytest.mark.asyncio
async def test_manual_runtime_requires_arm_and_applies_drive_command():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=500, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "drive", BodyVelocity(0.1, 0.0, 0.0)))
        await asyncio.sleep(0.04)
        assert wheels.commands == []

        await peer.queue.put(ManualControlCommand("session-a", 2, "arm"))
        await peer.queue.put(ManualControlCommand("session-a", 3, "drive", BodyVelocity(0.3, 0.0, 2.0)))
        await asyncio.sleep(0.06)
        assert wheels.commands
        assert wheels.commands[-1] == BodyVelocity(0.3, 0.0, 2.0)
        assert sensor.started
    finally:
        stop_event.set()
        await task
    assert sensor.stopped
    assert wheels.stops >= 2


@pytest.mark.asyncio
async def test_manual_runtime_watchdog_stops_and_disarms():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=100, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        await peer.queue.put(ManualControlCommand("session-a", 2, "drive", BodyVelocity(0.05, 0.0, 0.0)))
        await asyncio.sleep(0.2)
        stops_after_watchdog = wheels.stops
        commands_after_watchdog = len(wheels.commands)
        await asyncio.sleep(0.08)
        assert wheels.stops >= stops_after_watchdog
        assert len(wheels.commands) == commands_after_watchdog
    finally:
        stop_event.set()
        await task


@pytest.mark.asyncio
async def test_manual_runtime_disarm_stops_output_immediately():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=500, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        await peer.queue.put(ManualControlCommand("session-a", 2, "drive", BodyVelocity(0.05, 0.0, 0.0)))
        await asyncio.sleep(0.04)
        previous_stops = wheels.stops
        await peer.queue.put(ManualControlCommand("session-a", 3, "disarm"))
        await asyncio.sleep(0.02)
        assert wheels.stops > previous_stops
    finally:
        stop_event.set()
        await task


@pytest.mark.asyncio
async def test_actuator_command_failure_disarms_without_terminating_runtime():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=500, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        await asyncio.sleep(0.03)
        wheels.fail_next_command = True
        await peer.queue.put(ManualControlCommand("session-a", 2, "drive", BodyVelocity(0.05, 0.0, 0.0)))
        await asyncio.sleep(0.08)

        assert not task.done()
        assert wheels.stops >= 2
        assert peer.status_messages[-1] == ("session-a", 2, False, True)
    finally:
        stop_event.set()
        await task


def test_manual_control_limits_reject_invalid_command_timeout():
    with pytest.raises(ValueError):
        ManualControlLimits(command_timeout_ms=20)


@pytest.mark.asyncio
async def test_new_controller_session_can_rearm_with_a_fresh_sequence():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=500, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 10, "arm"))
        await peer.queue.put(ManualControlCommand("session-a", 11, "drive", BodyVelocity(0.05, 0.0, 0.0)))
        await asyncio.sleep(0.04)
        await peer.queue.put(ManualControlCommand("session-b", 0, "arm"))
        await peer.queue.put(ManualControlCommand("session-b", 1, "drive", BodyVelocity(0.0, 0.05, 0.0)))
        await asyncio.sleep(0.04)
        assert wheels.commands[-1] == BodyVelocity(0.0, 0.05, 0.0)
    finally:
        stop_event.set()
        await task


@pytest.mark.asyncio
async def test_status_command_reports_actual_arm_state_without_changing_session():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=500, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        await asyncio.sleep(0.02)
        await peer.queue.put(ManualControlCommand("session-b", 0, "status"))
        await asyncio.sleep(0.02)
        assert peer.status_messages == [
            ("session-a", 1, True, True),
            ("session-b", 0, True, False),
        ]
        await peer.queue.put(ManualControlCommand("session-b", 1, "disarm"))
        await asyncio.sleep(0.02)
        assert peer.status_messages[-1] == ("session-b", 1, False, True)
    finally:
        stop_event.set()
        await task


@pytest.mark.asyncio
async def test_manual_runtime_allows_arm_ack_round_trip_before_starting_drive_watchdog():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=100, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        # Longer than the 100 ms motion watchdog: the UI is still waiting for
        # the ARM status acknowledgement and has not started its drive heartbeat.
        await asyncio.sleep(0.3)
        assert runtime._armed is True
        assert peer.status_messages == [("session-a", 1, True, True)]

        await peer.queue.put(ManualControlCommand("session-a", 2, "drive", BodyVelocity(0.05, 0.0, 0.0)))
        await asyncio.sleep(0.04)
        assert runtime._armed is True
        assert wheels.commands[-1] == BodyVelocity(0.05, 0.0, 0.0)
    finally:
        stop_event.set()
        await task


@pytest.mark.asyncio
async def test_manual_runtime_disarms_if_no_drive_follows_arm_handshake_grace():
    peer = FakePeer()
    wheels = FakeWheelController()
    sensor = FakeDistanceSensor()
    runtime = ManualControlRuntime(
        peer,
        wheels,
        sensor,
        ManualControlLimits(command_timeout_ms=100, control_rate_hz=50),
    )
    stop_event = asyncio.Event()
    task = asyncio.create_task(runtime.run(stop_event))
    try:
        await peer.queue.put(ManualControlCommand("session-a", 1, "arm"))
        await asyncio.sleep(1.7)
        assert runtime._armed is False
        assert peer.status_messages[-1] == ("session-a", 1, False, True)
    finally:
        stop_event.set()
        await task
