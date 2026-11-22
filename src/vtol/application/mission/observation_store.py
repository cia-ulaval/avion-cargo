import time
from collections import deque
from dataclasses import replace
from threading import Lock

from shared.contracts.messages import PayloadEvent
from vtol.application.mission.context import MissionContext, PayloadObservation
from vtol.domain.drone import DroneStatus
from vtol.domain.mission.execution_status import MissionExecutionStatus


class MissionObservationStore:
    """
    Thread-safe latest-observation store shared by telemetry/event readers.
    """

    def __init__(self, *, payload_event_history: int = 32):
        if payload_event_history <= 0:
            raise ValueError("payload_event_history must be greater than zero")
        
        self._lock = Lock()
        self._drone_status: DroneStatus | None = None
        self._mission_execution_status = MissionExecutionStatus()
        self._payload_observations: deque[PayloadObservation] = deque(maxlen=payload_event_history)

    def set_flight_observation(
        self,
        drone_status: DroneStatus | None,
        mission_execution_status: MissionExecutionStatus,
    ) -> None:
        with self._lock:
            self._drone_status = None if drone_status is None else replace(drone_status)
            self._mission_execution_status = mission_execution_status

    def record_payload_event(self, payload_id: str, event: PayloadEvent) -> None:
        if not payload_id:
            raise ValueError("payload_id must be a non-empty string")
        
        with self._lock:
            self._payload_observations.append(PayloadObservation(payload_id, event, time.monotonic()))

    def capture_mission_snapshot(self) -> MissionContext:
        with self._lock:
            return MissionContext(
                drone_status=None if self._drone_status is None else replace(self._drone_status),
                mission_execution_status=self._mission_execution_status,
                payload_observations=tuple(self._payload_observations),
            )
