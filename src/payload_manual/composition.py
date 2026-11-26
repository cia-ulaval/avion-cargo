from __future__ import annotations

import asyncio
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any, Callable

from loguru import logger

from payload.application.omni_wheel_controller import OmniWheelController
from payload.domain.wheel_actuator import WheelActuator, WheelActuatorCapabilities
from payload.infrastructure.configuration import ManualPayloadRuntimeConfiguration
from payload.infrastructure.d3kble import D3kbleDistanceSensor
from payload.infrastructure.wheel_actuators import create_wheel_actuator
from payload_manual.application.runtime import ManualControlLimits, ManualControlRuntime
from payload_manual.infrastructure.d3kble.manual_control_peer import D3kbleManualControlPeer
from shared.infrastructure.d3kble import D3kbleMessageRouter


@dataclass(slots=True)
class ManualControlApplication:
    """Own the manual payload controller's resources and runtime."""

    node: Any
    router: D3kbleMessageRouter
    control_peer: D3kbleManualControlPeer
    distance_sensor: D3kbleDistanceSensor
    wheel_controller: OmniWheelController
    runtime: ManualControlRuntime
    _closed: bool = field(default=False, init=False, repr=False)

    async def run(self, stop_event: asyncio.Event) -> None:
        await self.runtime.run(stop_event)

    async def close(self) -> None:
        """Close manual-control adapters before the D3K and motor resources."""
        if self._closed:
            return
        self._closed = True

        try:
            await self.wheel_controller.close()
        except Exception:
            logger.exception("Could not close manual payload wheel actuators cleanly")

        for close_resource in (
            self.control_peer.close,
            self.router.close,
            self.node.ble.stop_ble,
            self.node.close,
        ):
            try:
                close_resource()
            except Exception:
                logger.exception("Could not close a manual payload application resource cleanly")


def build_manual_control_application(
    config: ManualPayloadRuntimeConfiguration,
    *,
    limits: ManualControlLimits | None = None,
    wheel_capabilities: Sequence[WheelActuatorCapabilities] | None = None,
    node_factory: Callable[..., Any] | None = None,
) -> ManualControlApplication:
    """Build manual control with a D3K session owned by this application."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        pass
    else:
        raise RuntimeError("build_manual_control_application must be called outside a running event loop")

    controller_peer_id = config.controller_peer_id
    if (
        isinstance(controller_peer_id, bool)
        or not isinstance(controller_peer_id, int)
        or not 1 <= controller_peer_id <= 65534
    ):
        raise ValueError("controller_peer_id must be an integer in 1..65534")
    if len(config.wheels) < 3:
        raise ValueError("at least three wheels must be configured for manual control")

    if wheel_capabilities is None:
        capabilities_by_wheel = tuple(WheelActuatorCapabilities(wheel.max_surface_velocity) for wheel in config.wheels)
    else:
        capabilities_by_wheel = tuple(wheel_capabilities)
        if len(capabilities_by_wheel) != len(config.wheels):
            raise ValueError("wheel_capabilities must contain one item per configured wheel")
        if not all(isinstance(item, WheelActuatorCapabilities) for item in capabilities_by_wheel):
            raise TypeError("wheel_capabilities entries must be WheelActuatorCapabilities")

    if node_factory is None:
        try:
            from d3kble import D3KNode
        except ImportError as exc:
            raise RuntimeError("The d3kble package is required. Install the vehicle runtime dependencies.") from exc
        node_factory = D3KNode

    node = node_factory(config.uart_device, baudrate=config.baudrate)
    router: D3kbleMessageRouter | None = None
    control_peer: D3kbleManualControlPeer | None = None
    actuators: list[WheelActuator] = []
    ble_start_attempted = False

    try:
        node.start()
        node.wait_until_ready(timeout=config.ready_timeout_s)

        ble_start_attempted = True
        node.ble.start_ble()

        uwb_overrides = config.uwb_overrides
        if uwb_overrides:
            node.config.set_desired(uwb_overrides)
            node.config.set_mode("manual")
            result = node.config.reconcile(approve=True)
            if not result.get("success", False):
                raise RuntimeError(f"Payload D3K did not apply its UWB overrides: {result}")

        router = D3kbleMessageRouter(node)
        distance_sensor = D3kbleDistanceSensor(
            node,
            anchor_id_by_target_id=config.anchor_id_by_target_id,
        )

        for wheel, capabilities in zip(config.wheels, capabilities_by_wheel):
            actuators.append(
                create_wheel_actuator(
                    wheel.driver,
                    wheel.transform.radius,
                    capabilities,
                )
            )

        control_peer = D3kbleManualControlPeer(
            node,
            router,
            controller_peer_id=controller_peer_id,
        )
        wheel_controller = OmniWheelController(
            tuple(wheel.transform for wheel in config.wheels),
            actuators,
        )
        runtime = ManualControlRuntime(
            control_peer,
            wheel_controller,
            distance_sensor,
            limits or ManualControlLimits(),
        )
        return ManualControlApplication(
            node,
            router,
            control_peer,
            distance_sensor,
            wheel_controller,
            runtime,
        )
    except BaseException:
        if control_peer is not None:
            try:
                control_peer.close()
            except Exception:
                logger.exception("Could not close manual-control BLE peer after startup failure")
        if router is not None:
            try:
                router.close()
            except Exception:
                logger.exception("Could not close manual-control D3K router after startup failure")
        if actuators:

            async def close_actuators() -> None:
                await asyncio.gather(
                    *(actuator.close() for actuator in actuators),
                    return_exceptions=True,
                )

            asyncio.run(close_actuators())
        if ble_start_attempted:
            try:
                node.ble.stop_ble()
            except Exception:
                logger.exception("Could not stop manual payload BLE after startup failure")
        try:
            node.close()
        except Exception:
            logger.exception("Could not close manual payload D3K node after startup failure")
        raise
