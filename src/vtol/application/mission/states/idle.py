from vtol.application.mission.context import MissionContext
from vtol.application.mission.decisions import MissionProgressStatus
from vtol.application.mission.states.base import PassiveMissionState
from vtol.application.mission.states.interfaces import MissionState


class IdleState(PassiveMissionState):
    section = None
    status = MissionProgressStatus.IDLE
    description = "Waiting for the next mission cycle."

    @property
    def name(self) -> str:
        return "idle"
