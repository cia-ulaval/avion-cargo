from vtol.application.mission.states.base import PassiveMissionState
from vtol.domain.mission.flight_section import FlightSection


class PlaneLandingState(PassiveMissionState):
    section = FlightSection.PLANE_LANDING
    description = "Observing the plane landing phase."

    @property
    def name(self) -> str:
        return "plane_landing"
