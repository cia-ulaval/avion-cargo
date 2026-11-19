from dataclasses import dataclass, field

from shared.contracts.messages import PayloadEvent
from vtol.domain.drone import DroneStatus
from vtol.domain.mission.execution_status import MissionExecutionStatus


@dataclass(frozen=True, slots=True)
class PayloadObservation:
    payload_id: str
    event: PayloadEvent
    received_monotonic_s: float


@dataclass(frozen=True, slots=True)
class MissionContext:
    """
    Immutable snapshot of observations available to the VTOL mission logic.
    """

    drone_status: DroneStatus | None = None
    mission_execution_status: MissionExecutionStatus = field(default_factory=MissionExecutionStatus)
    payload_observations: tuple[PayloadObservation, ...] = ()
