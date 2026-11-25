from __future__ import annotations

import asyncio

from loguru import logger

from vtol.application.mission.observation_store import MissionObservationStore
from vtol.domain.drone import Drone


class FlightControllerObservationRuntime:
    """Own the MAVLink polling loop and publish immutable status observations."""

    def __init__(self, drone: Drone, observations: MissionObservationStore, *, poll_interval_s: float = 0.1):
        if poll_interval_s <= 0:
            raise ValueError("poll_interval_s must be greater than zero")
        self.drone = drone
        self.observations = observations
        self.poll_interval_s = poll_interval_s

    async def run(self, stop_event: asyncio.Event) -> None:
        try:
            await asyncio.to_thread(self.drone.connect)
            logger.info("Flight-controller telemetry observation started")
            while not stop_event.is_set():
                status = await asyncio.to_thread(self.drone.get_status)
                mission_status = await asyncio.to_thread(self.drone.get_mission_execution_status)
                self.observations.set_flight_observation(status, mission_status)
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=self.poll_interval_s)
                except asyncio.TimeoutError:
                    pass
        finally:
            await asyncio.to_thread(self.drone.close)
            logger.info("Flight-controller telemetry observation stopped")
