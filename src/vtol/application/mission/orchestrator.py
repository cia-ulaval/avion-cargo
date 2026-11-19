from __future__ import annotations

import asyncio
from collections.abc import Callable

from loguru import logger

from shared.application.state_machine import StateMachine
from vtol.application.mission.context import MissionContext
from vtol.application.mission.decisions import MissionProgress
from vtol.application.mission.states import IdleState


class MissionOrchestrator:
    """Drive the VTOL mission state machine from current observation snapshots."""

    def __init__(self, context_provider: Callable[[], MissionContext]):
        self._context_provider = context_provider
        self._machine: StateMachine[MissionContext, MissionProgress] = StateMachine(IdleState())
        self._last_progress: MissionProgress | None = None

    @property
    def current_state_name(self) -> str:
        return self._machine.current_state_name

    async def step(self, context: MissionContext | None = None) -> MissionProgress:
        snapshot = self._context_provider() if context is None else context
        progress = await self._machine.exec(snapshot)
        if progress != self._last_progress:
            logger.info(
                "Mission progress: {} (section={}, description={})",
                progress.status.name,
                progress.section.name if progress.section is not None else None,
                progress.description,
            )
            self._last_progress = progress
        else:
            logger.debug("Mission state {} executed", self.current_state_name)
        return progress

    async def run(self, stop_event: asyncio.Event, *, tick_interval: float = 0.1) -> None:
        """Refresh observations and execute one state-machine step per tick."""
        if tick_interval <= 0:
            raise ValueError("tick_interval must be greater than zero")

        try:
            while not stop_event.is_set():
                await self.step()
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=tick_interval)
                except asyncio.TimeoutError:
                    pass
        finally:
            await self._machine.close(self._context_provider())
