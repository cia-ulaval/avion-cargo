from vtol.application.mission.states.base import PassiveMissionState
from vtol.domain.mission.flight_section import FlightSection


class PayloadTransferState(PassiveMissionState):
    section = FlightSection.PAYLOAD_TRANSFER
    description = "Observing the payload transfer phase."

    @property
    def name(self) -> str:
        return "payload_transfer"
