from vtol.application.mission.states.base import PassiveMissionState
from vtol.domain.mission.flight_section import FlightSection


class PlaneTakeoffState(PassiveMissionState):
    section = FlightSection.PLANE_TAKEOFF
    description = "Observing the plane takeoff phase."

    @property
    def name(self) -> str:
        return "plane_takeoff"
