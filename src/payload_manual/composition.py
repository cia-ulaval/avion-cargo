from __future__ import annotations

import asyncio
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Callable

from loguru import logger

from payload.application.omni_wheel_controller import OmniWheelController
from payload.domain.wheel_actuator import WheelActuator, WheelActuatorCapabilities
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload.infrastructure.resources import PayloadInfrastructure, build_payload_infrastructure
from payload.infrastructure.wheel_actuators import create_wheel_actuator
from payload_manual.application.runtime import ManualControlLimits, ManualControlRuntime
from payload_manual.infrastructure.d3kble.manual_control_peer import D3kbleManualControlPeer


@dataclass
class ManualControlApplication:
    """Manual-control application layered on the shared payload infrastructure."""

    infrastructure: PayloadInfrastructure
    control_peer: D3kbleManualControlPeer
    wheel_controller: OmniWheelController
    runtime: ManualControlRuntime

    @property
    def node(self) -> Any:
        return self.infrastructure.node

    @property
    def router(self):
        return self.infrastructure.router

    async def run(self, stop_event: asyncio.Event) -> None:
        await self.runtime.run(stop_event)

    async def close(self) -> None:
        try:
            await self.wheel_controller.close()
        finally:
            try:
                self.control_peer.close()
            finally:
                await self.infrastructure.aclose()


def build_manual_control_application(
    config: PayloadRuntimeConfiguration,
    *,
    controller_peer_id: int,
    max_wheel_surface_velocity_m_s: float | None = None,
    limits: ManualControlLimits | None = None,
    wheel_capabilities: Sequence[WheelActuatorCapabilities] | None = None,
    node_factory: Callable[..., Any] | None = None,
) -> ManualControlApplication:
    """Build manual control without creating the mission-oriented plane peer."""
    if (
        isinstance(controller_peer_id, bool)
        or not isinstance(controller_peer_id, int)
        or not 1 <= controller_peer_id <= 65534
    ):
        raise ValueError("controller_peer_id must be an integer in 1..65534")
    if controller_peer_id == config.plane_peer_id:
        raise ValueError("manual controller peer ID must differ from the plane peer ID")
    if len(config.wheels) < 3:
        raise ValueError("at least three wheels must be configured for manual control")

    if wheel_capabilities is None:
        if max_wheel_surface_velocity_m_s is None:
            raise ValueError(
                "max_wheel_surface_velocity_m_s is required when wheel_capabilities are omitted"
            )
        default_capabilities = WheelActuatorCapabilities(max_wheel_surface_velocity_m_s)
        capabilities_by_wheel = (default_capabilities,) * len(config.wheels)
    else:
        capabilities_by_wheel = tuple(wheel_capabilities)
        if len(capabilities_by_wheel) != len(config.wheels):
            raise ValueError("wheel_capabilities must contain one item per configured wheel")
        if not all(isinstance(item, WheelActuatorCapabilities) for item in capabilities_by_wheel):
            raise TypeError("wheel_capabilities entries must be WheelActuatorCapabilities")

    infrastructure = build_payload_infrastructure(config, node_factory=node_factory)
    actuators: list[WheelActuator] = []
    control_peer: D3kbleManualControlPeer | None = None
    try:
        for wheel, capabilities in zip(config.wheels, capabilities_by_wheel):
            actuators.append(
                create_wheel_actuator(
                    wheel.driver,
                    wheel.transform.radius,
                    capabilities,
                )
            )

        control_peer = D3kbleManualControlPeer(
            infrastructure.node,
            infrastructure.router,
            controller_peer_id=controller_peer_id,
        )
        wheel_controller = OmniWheelController(
            tuple(wheel.transform for wheel in config.wheels),
            actuators,
        )
        runtime = ManualControlRuntime(
            control_peer,
            wheel_controller,
            infrastructure.distance_sensor,
            limits or ManualControlLimits(),
        )
        return ManualControlApplication(infrastructure, control_peer, wheel_controller, runtime)
    except BaseException:
        if actuators:
            async def close_actuators() -> None:
                await asyncio.gather(
                    *(actuator.close() for actuator in actuators),
                    return_exceptions=True,
                )

            try:
                asyncio.run(close_actuators())
            except RuntimeError:
                logger.exception(
                    "Could not close partially initialized wheel actuators; "
                    "build_manual_control_application must run outside an event loop"
                )
        if control_peer is not None:
            control_peer.close()
        infrastructure.close()
        raise
