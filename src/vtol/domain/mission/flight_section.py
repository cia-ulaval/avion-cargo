from enum import IntEnum


class FlightSection(IntEnum):
    """
    Application-level phases of an Avion Cargo mission.
    """

    PLANE_TAKEOFF = 1
    VTOL_LAND = 2
    PAYLOAD_TRANSFER = 3
    VTOL_TAKEOFF = 4
    PLANE_LANDING = 5
