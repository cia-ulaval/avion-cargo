from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from loguru import logger

from payload.application.runtime import PayloadApplicationRuntime
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload.infrastructure.d3kble import D3kbleDistanceSensor, D3kblePlanePeer
from shared.infrastructure.d3kble import D3kbleMessageRouter


@dataclass(slots=True)
class PayloadApplication:
    """Own the mission payload's D3K resources and application runtime."""

    node: Any
    router: D3kbleMessageRouter
    plane_peer: D3kblePlanePeer
    distance_sensor: D3kbleDistanceSensor
    runtime: PayloadApplicationRuntime
    _closed: bool = field(default=False, init=False, repr=False)

    async def run(self, stop_event) -> None:
        await self.runtime.run(stop_event)

    def close(self) -> None:
        """Close mission adapters before the D3K resources they depend on."""
        if self._closed:
            return
        self._closed = True

        for close_resource in (
            self.plane_peer.close,
            self.router.close,
            self.node.ble.stop_ble,
            self.node.close,
        ):
            try:
                close_resource()
            except Exception:
                logger.exception("Could not close a payload application resource cleanly")


def build_payload_application(
    config: PayloadRuntimeConfiguration,
    *,
    node_factory: Callable[..., Any] | None = None,
) -> PayloadApplication:
    """Build the mission-oriented payload application and its owned resources."""
    if node_factory is None:
        try:
            from d3kble import D3KNode
        except ImportError as exc:
            raise RuntimeError("The d3kble package is required. Install the vehicle runtime dependencies.") from exc
        node_factory = D3KNode

    node = node_factory(config.uart_device, baudrate=config.baudrate)
    router: D3kbleMessageRouter | None = None
    plane_peer: D3kblePlanePeer | None = None
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
        plane_peer = D3kblePlanePeer(
            node,
            router,
            plane_peer_id=config.plane_peer_id,
        )
        runtime = PayloadApplicationRuntime(plane_peer, distance_sensor)
        return PayloadApplication(node, router, plane_peer, distance_sensor, runtime)
    except BaseException:
        if plane_peer is not None:
            try:
                plane_peer.close()
            except Exception:
                logger.exception("Could not close the payload plane peer after startup failure")
        if router is not None:
            try:
                router.close()
            except Exception:
                logger.exception("Could not close the payload D3K router after startup failure")
        if ble_start_attempted:
            try:
                node.ble.stop_ble()
            except Exception:
                logger.exception("Could not stop payload BLE after startup failure")
        try:
            node.close()
        except Exception:
            logger.exception("Could not close payload D3K node after startup failure")
        raise
