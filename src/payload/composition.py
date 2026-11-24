from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from payload.application.runtime import PayloadApplicationRuntime
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload.infrastructure.d3kble import D3kblePlanePeer
from payload.infrastructure.resources import PayloadInfrastructure, build_payload_infrastructure


@dataclass
class PayloadApplication:
    """
    Mission-oriented payload application.
    """

    infrastructure: PayloadInfrastructure
    plane_peer: D3kblePlanePeer
    runtime: PayloadApplicationRuntime

    @property
    def node(self) -> Any:
        return self.infrastructure.node

    @property
    def router(self):
        return self.infrastructure.router

    @property
    def distance_sensor(self):
        return self.infrastructure.distance_sensor

    async def run(self, stop_event) -> None:
        await self.runtime.run(stop_event)

    def close(self) -> None:
        """
        Close the mission peer, then the shared payload infrastructure.
        """
        try:
            self.plane_peer.close()
        finally:
            self.infrastructure.close()


def build_payload_application(
    config: PayloadRuntimeConfiguration,
    *,
    node_factory: Callable[..., Any] | None = None,
) -> PayloadApplication:
    """
    Compose the mission-oriented application.
    """
    infrastructure = build_payload_infrastructure(config, node_factory=node_factory)
    try:
        plane_peer = D3kblePlanePeer(
            infrastructure.node,
            infrastructure.router,
            plane_peer_id=config.plane_peer_id,
        )
        runtime = PayloadApplicationRuntime(plane_peer, infrastructure.distance_sensor)
        return PayloadApplication(infrastructure, plane_peer, runtime)
    except BaseException:
        infrastructure.close()
        raise
