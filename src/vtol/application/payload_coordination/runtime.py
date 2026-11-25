from __future__ import annotations

import asyncio
from collections.abc import Sequence

from loguru import logger

from vtol.application.mission.observation_store import MissionObservationStore
from vtol.domain.payload_peer import PayloadPeer


class VtolPayloadCommunicationRuntime:
    """Consume payload event streams and publish events to the observation store."""

    def __init__(self, payload_peers: Sequence[PayloadPeer], observations: MissionObservationStore):
        self.payload_peers = tuple(payload_peers)
        self.observations = observations

    async def run(self, stop_event: asyncio.Event) -> None:
        async with asyncio.TaskGroup() as group:
            tasks = [
                group.create_task(self._consume_peer(peer), name=f"payload-events-{peer.payload_id}")
                for peer in self.payload_peers
            ]
            await stop_event.wait()
            for task in tasks:
                task.cancel()

    async def _consume_peer(self, peer: PayloadPeer) -> None:
        async for event in peer.events():
            self.observations.record_payload_event(peer.payload_id, event)
            logger.info("Payload {} event: {}", peer.payload_id, event)
