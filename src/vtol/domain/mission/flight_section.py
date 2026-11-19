from enum import IntEnum


class FlightSection(IntEnum):
    """Application-level phases of an Avion Cargo mission.

    These values identify VTOL application phases. They are not ArduPilot
    mission-item sequence numbers or ground-station directives.
    """

    PLANE_TAKEOFF = 1
    VTOL_LAND = 2
    PAYLOAD_TRANSFER = 3
    VTOL_TAKEOFF = 4
    PLANE_LANDING = 5
