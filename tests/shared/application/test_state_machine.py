import asyncio

import pytest

from shared.application.state_machine import StateMachine


class RecordingState:
    def __init__(self, name, events, next_state=None, result=None):
        self.name = name
        self.events = events
        self.next_state = next_state
        self.result = result

    async def start(self, input):
        self.events.append(("start", self.name, input))

    async def exec(self, input):
        self.events.append(("exec", self.name, input))
        return self.result

    def next(self, input):
        self.events.append(("next", self.name, input))
        return self.next_state or self

    async def stop(self, input):
        self.events.append(("stop", self.name, input))

    def debug(self, input):
        self.events.append(("debug", self.name, input))


def test_state_lifecycle_matches_start_exec_next_stop_then_start_next() -> None:
    async def scenario():
        events = []
        second = RecordingState("second", events, result="done")
        first = RecordingState("first", events, next_state=second, result="step-1")
        machine = StateMachine(first)

        assert await machine.exec("input-1") == "step-1"
        assert machine.current_state_name == "second"
        assert events == [
            ("start", "first", "input-1"),
            ("exec", "first", "input-1"),
            ("next", "first", "input-1"),
            ("stop", "first", "input-1"),
        ]

        assert await machine.exec("input-2") == "done"
        assert events[-3:] == [
            ("start", "second", "input-2"),
            ("exec", "second", "input-2"),
            ("next", "second", "input-2"),
        ]
        await machine.close("shutdown")
        assert events[-1] == ("stop", "second", "shutdown")

    asyncio.run(scenario())


def test_close_is_idempotent_and_does_not_stop_a_pending_unstarted_state() -> None:
    async def scenario():
        events = []
        second = RecordingState("second", events)
        first = RecordingState("first", events, next_state=second)
        machine = StateMachine(first)
        await machine.exec("transition")
        await machine.close("shutdown")
        await machine.close("again")
        assert [event[:2] for event in events if event[0] in {"start", "stop"}] == [
            ("start", "first"),
            ("stop", "first"),
        ]
        assert machine.is_closed

    asyncio.run(scenario())


def test_step_after_close_is_rejected() -> None:
    async def scenario():
        machine = StateMachine(RecordingState("initial", []))
        await machine.close("shutdown")
        with pytest.raises(RuntimeError, match="closed"):
            await machine.exec("input")

    asyncio.run(scenario())
