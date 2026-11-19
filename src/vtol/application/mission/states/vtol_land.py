from vtol.application.mission.states.base import PassiveMissionState
from vtol.domain.mission.flight_section import FlightSection


class VtolLandState(PassiveMissionState):
    section = FlightSection.VTOL_LAND
    description = "Observing the VTOL landing phase."

    @property
    def name(self) -> str:
        return "vtol_land"
