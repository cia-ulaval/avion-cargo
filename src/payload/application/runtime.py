from __future__ import annotations

import asyncio

from loguru import logger

from payload.domain.docking.plane_peer import PlanePeer
from payload.domain.positioning.distance_sensor import DistanceSensor
from payload.domain.positioning.events import DistanceFailure, DistanceSample
from shared.contracts.messages import (
    CommandReceipt,
    DLZStateUpdated,
    DockRequest,
    FailureReason,
    PayloadStatusReport,
    PayloadStatusRequested,
    PlaneHeadingUpdated,
    UndockRequest,
)
from shared.contracts.payload_state import DockingLatchState, PayloadState, PayloadStatus


class PayloadApplicationRuntime:
    """
    Consume D3Kble events.
    """

    def __init__(self, plane_peer: PlanePeer, distance_sensor: DistanceSensor):
        self.plane_peer = plane_peer
        self.distance_sensor = distance_sensor

    async def run(self, stop_event: asyncio.Event) -> None:
        async with asyncio.TaskGroup() as group:
            plane_task = group.create_task(self._consume_plane_events(), name="payload-plane-events")
            distance_task = group.create_task(self._consume_distance_events(), name="payload-distance-events")
            await stop_event.wait()
            plane_task.cancel()
            distance_task.cancel()

    async def _consume_plane_events(self) -> None:
        async for event in self.plane_peer.events():
            if isinstance(event, (DockRequest, UndockRequest)):
                await self._reject_unsupported_command(event)
            elif isinstance(event, PayloadStatusRequested):
                await self._report_unknown_status(event)
            elif isinstance(event, PlaneHeadingUpdated):
                logger.debug("Received plane heading update: {}", event.heading)
            elif isinstance(event, DLZStateUpdated):
                logger.debug("Received DLZ snapshot revision {}", event.state.revision)
            else:
                logger.warning("Ignoring unsupported plane event: {}", event)

    async def _consume_distance_events(self) -> None:
        async for event in self.distance_sensor.events():
            if isinstance(event, DistanceSample):
                logger.debug(
                    "UWB distance to %s: %.3f m (sequence %s)",
                    event.anchor_id,
                    event.distance_m,
                    event.sequence,
                )
            elif isinstance(event, DistanceFailure):
                logger.warning("UWB range to %s failed: %s", event.anchor_id, event.reason.message)
            else:
                logger.warning("Ignoring unsupported distance event: {}", event)

    async def _reject_unsupported_command(self, command: DockRequest | UndockRequest) -> None:
        receipt = CommandReceipt(
            command_id=command.command_id,
            accepted=False,
            rejection_reason=FailureReason(
                code="not_implemented",
                message="This operation is not implemented in this application version.",
            ),
        )
        await self.plane_peer.acknowledge_command(receipt)
        logger.info("Rejected unsupported payload command {}", command.command_id)

    async def _report_unknown_status(self, request: PayloadStatusRequested) -> None:
        report = PayloadStatusReport(
            state=PayloadState(PayloadStatus.UNKNOWN, DockingLatchState.UNKNOWN),
            position=None,
            sequence=0,
        )
        await self.plane_peer.respond_status_request(request.request_id, report)
