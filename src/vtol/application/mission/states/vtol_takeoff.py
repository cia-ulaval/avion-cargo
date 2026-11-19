from vtol.application.mission.states.base import PassiveMissionState
from vtol.domain.mission.flight_section import FlightSection


class VtolTakeoffState(PassiveMissionState):
    section = FlightSection.VTOL_TAKEOFF
    description = "Observing the VTOL takeoff phase."

    @property
    def name(self) -> str:
        return "vtol_takeoff"
