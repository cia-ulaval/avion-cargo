from shared.application.state_machine import State
from vtol.application.mission.context import MissionContext
from vtol.application.mission.decisions import MissionProgress

MissionState = State[MissionContext, MissionProgress]

__all__ = ["MissionState"]
