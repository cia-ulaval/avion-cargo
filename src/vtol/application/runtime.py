from __future__ import annotations

import asyncio

from vtol.application.flight_controller_runtime import FlightControllerObservationRuntime
from vtol.application.mission.orchestrator import MissionOrchestrator
from vtol.application.payload_coordination.runtime import VtolPayloadCommunicationRuntime


class VtolApplicationRuntime:
    """Coordinate the independent VTOL application subsystems."""

    def __init__(
        self,
        flight_controller: FlightControllerObservationRuntime,
        payload_communication: VtolPayloadCommunicationRuntime,
        mission_orchestrator: MissionOrchestrator,
    ):
        self.flight_controller = flight_controller
        self.payload_communication = payload_communication
        self.mission_orchestrator = mission_orchestrator

    async def run(self, stop_event: asyncio.Event) -> None:
        async with asyncio.TaskGroup() as group:
            group.create_task(self.flight_controller.run(stop_event), name="flight-controller-observation")
            group.create_task(self.payload_communication.run(stop_event), name="payload-communication")
            group.create_task(self.mission_orchestrator.run(stop_event), name="mission-orchestrator")
            await stop_event.wait()
