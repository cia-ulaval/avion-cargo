from __future__ import annotations

from typing import Generic, Protocol, TypeVar

InputT = TypeVar("InputT")
OutputT = TypeVar("OutputT")


class State(Protocol[InputT, OutputT]):
    """
    One executable state object.
    """

    @property
    def name(self) -> str:
        """
        Human-readable name used for diagnostics.
        """
        ...

    async def start(self, input: InputT) -> None:
        """
        Run once when this state becomes active and before running exec(). 
        """
        ...

    async def exec(self, input: InputT) -> OutputT:
        """
        Execute one step and return its output.
        """
        ...

    def next(self, input: InputT) -> State[InputT, OutputT]:
        """
        Select the state that should execute on the next step.
        """
        ...

    async def stop(self, input: InputT) -> None:
        """
        Run once when this state is exited or the machine is closed.
        """
        ...

    def debug(self, input: InputT) -> None:
        """
        Emit state-specific diagnostics without advancing the machine.
        """
        ...


class StateMachine(Generic[InputT, OutputT]):
    """
    Execute state objects while managing their lifecycle.
    """

    def __init__(self, initial_state: State[InputT, OutputT]):
        self._active_state: State[InputT, OutputT] | None = None
        self._pending_state = initial_state
        self._closed = False
        self._stepping = False

    @property
    def current_state_name(self) -> str:
        """
        Name of the active state, or the pending initial/next state.
        """
        state = self._active_state or self._pending_state
        return state.name

    @property
    def is_active(self) -> bool:
        return self._active_state is not None

    @property
    def is_closed(self) -> bool:
        return self._closed

    async def exec(self, input: InputT) -> OutputT:
        """
        Execute one state step and perform any resulting transition.
        """
        if self._closed:
            raise RuntimeError("state machine is closed")
        
        if self._stepping:
            raise RuntimeError("state machine execution is already in progress")

        self._stepping = True

        try:
            if self._active_state is None:
                state = self._pending_state
                await state.start(input)
                self._active_state = state
            else:
                state = self._active_state

            output = await state.exec(input)
            next_state = state.next(input)

            if next_state is None:
                raise TypeError(f"state {state.name!r} returned None from next()")

            if next_state is not state:
                self._active_state = None
                self._pending_state = next_state
                await state.stop(input)

            return output
        finally:
            self._stepping = False

    def debug(self, input: InputT) -> None:
        """
        Delegate diagnostics to the currently active or pending state.
        """
        if self._closed:
            raise RuntimeError("state machine is closed")
        
        state = self._active_state or self._pending_state
        state.debug(input)

    async def close(self, input: InputT) -> None:
        """
        Stop the active state and permanently close the machine.
        """
        if self._closed:
            return
        
        if self._stepping:
            raise RuntimeError("cannot close a state machine while it is executing")

        self._closed = True
        state, self._active_state = self._active_state, None
        
        if state is not None:
            await state.stop(input)
