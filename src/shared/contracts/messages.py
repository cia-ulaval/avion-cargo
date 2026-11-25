from dataclasses import dataclass
from typing import TypeAlias

from shared.contracts.geometry import Heading, Position, PositionEstimate
from shared.contracts.payload_state import PayloadState


@dataclass(frozen=True, slots=True)
class FailureReason:
    code: str
    message: str


@dataclass(frozen=True, slots=True)
class CommandReceipt:
    command_id: str
    accepted: bool
    rejection_reason: FailureReason | None = None


@dataclass(frozen=True, slots=True)
class PayloadStatusReport:
    state: PayloadState
    position: PositionEstimate | None
    sequence: int


@dataclass(frozen=True, slots=True)
class PayloadSnapshot:
    payload_id: str
    state: PayloadState
    position: PositionEstimate | None
    sequence: int


@dataclass(frozen=True, slots=True)
class DLZState:
    revision: int
    payloads: tuple[PayloadSnapshot, ...]


@dataclass(frozen=True, slots=True)
class PlaneHeadingUpdated:
    heading: Heading
    sequence: int


@dataclass(frozen=True, slots=True)
class DLZStateUpdated:
    state: DLZState


@dataclass(frozen=True, slots=True)
class DockRequest:
    command_id: str


@dataclass(frozen=True, slots=True)
class UndockRequest:
    command_id: str
    target_position: Position


@dataclass(frozen=True, slots=True)
class PayloadStatusRequested:
    request_id: str


PlaneEvent: TypeAlias = PlaneHeadingUpdated | DLZStateUpdated | DockRequest | UndockRequest | PayloadStatusRequested


@dataclass(frozen=True, slots=True)
class PayloadPositionUpdated:
    estimate: PositionEstimate


@dataclass(frozen=True, slots=True)
class PayloadStateUpdated:
    state: PayloadState
    sequence: int


@dataclass(frozen=True, slots=True)
class ParkingCompleted:
    command_id: str
    final_position: Position


@dataclass(frozen=True, slots=True)
class ParkingAborted:
    command_id: str
    reason: FailureReason


@dataclass(frozen=True, slots=True)
class DockingSucceeded:
    command_id: str


@dataclass(frozen=True, slots=True)
class DockingAborted:
    command_id: str
    reason: FailureReason


PayloadOperationEvent: TypeAlias = ParkingCompleted | ParkingAborted | DockingSucceeded | DockingAborted
PayloadEvent: TypeAlias = PayloadPositionUpdated | PayloadStateUpdated | PayloadOperationEvent
