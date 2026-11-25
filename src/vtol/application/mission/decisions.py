from dataclasses import dataclass
from enum import Enum, auto

from vtol.domain.mission.flight_section import FlightSection


class MissionProgressStatus(Enum):
    IDLE = auto()
    ACTIVE = auto()
    FAILED = auto()


@dataclass(frozen=True, slots=True)
class MissionProgress:
    """Informational progress from the Pi's application-level mission state."""

    section: FlightSection | None
    status: MissionProgressStatus
    description: str
