from dataclasses import dataclass
from enum import Enum, auto


class PayloadStatus(Enum):
    UNKNOWN = auto()
    DOCKED = auto()
    PARKING = auto()
    PARKED = auto()
    DOCKING = auto()
    FAULT = auto()


class DockingLatchState(Enum):
    UNKNOWN = auto()
    OPEN = auto()
    CLOSED = auto()


@dataclass(frozen=True, slots=True)
class PayloadState:
    status: PayloadStatus
    latch: DockingLatchState
