from loguru import logger

from vtol.application.mission.context import MissionContext
from vtol.application.mission.decisions import MissionProgress, MissionProgressStatus
from vtol.application.mission.states.interfaces import MissionState
from vtol.domain.mission.flight_section import FlightSection


class PassiveMissionState(MissionState):
    """
    Lifecycle-complete scaffold state; transition conditions are deferred.
    """

    section: FlightSection | None = None
    status = MissionProgressStatus.ACTIVE
    description = "Mission phase observation is not implemented yet."

    @property
    def name(self) -> str:
        return "mission_state"

    async def start(self, input: MissionContext) -> None:
        logger.info("Entered mission state {}", self.name)

    async def exec(self, input: MissionContext) -> MissionProgress:
        return MissionProgress(self.section, self.status, self.description)

    def next(self, input: MissionContext) -> MissionState:
        return self

    async def stop(self, input: MissionContext) -> None:
        logger.debug("Leaving mission state {}", self.name)

    def debug(self, input: MissionContext) -> None:
        logger.debug("Mission state {} is active", self.name)
