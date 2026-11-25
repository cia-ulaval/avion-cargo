from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from d3kble import D3KNode
from loguru import logger

from payload.application.runtime import PayloadApplicationRuntime
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload.infrastructure.d3kble import D3kbleDistanceSensor, D3kblePlanePeer
from shared.infrastructure.d3kble import D3kbleMessageRouter


@dataclass
class PayloadApplication:
    """
    Own the payload's local D3KNode and all adapters built on top of it.
    """

    node: D3KNode
    router: D3kbleMessageRouter
    plane_peer: D3kblePlanePeer
    distance_sensor: D3kbleDistanceSensor
    runtime: PayloadApplicationRuntime

    async def run(self, stop_event) -> None:
        await self.runtime.run(stop_event)

    def close(self) -> None:
        """Close adapters before closing their shared node."""
        self.plane_peer.close()
        self.router.close()
        try:
            self.node.ble.stop_ble()
        except Exception:
            logger.exception("Could not stop the payload BLE service cleanly")
        finally:
            self.node.close()


def build_payload_application(
    config: PayloadRuntimeConfiguration,
    *,
    node_factory: Callable[..., Any] | None = None,
) -> PayloadApplication:
    """
    Create and configure the payload's local hardware infrastructure.
    """
    if node_factory is None:
        try:
            from d3kble import D3KNode
        except ImportError as exc:
            raise RuntimeError("The d3kble package is required. Install the vehicle runtime dependencies.") from exc
        node_factory = D3KNode

    node = node_factory(config.uart_device, baudrate=config.baudrate)

    try:
        node.start()
        node.wait_until_ready(timeout=config.ready_timeout_s)
        node.ble.start_ble()

        uwb_overrides = config.uwb_overrides
        if uwb_overrides:
            node.config.set_desired(uwb_overrides)
            node.config.set_mode("manual")
            result = node.config.reconcile(approve=True)
            if not result.get("success", False):
                raise RuntimeError(f"Payload D3K did not apply its UWB overrides: {result}")

        router = D3kbleMessageRouter(node)
        plane_peer = D3kblePlanePeer(
            node,
            router,
            plane_peer_id=config.plane_peer_id,
        )
        distance_sensor = D3kbleDistanceSensor(
            node,
            anchor_id_by_target_id=config.anchor_id_by_target_id,
        )
        runtime = PayloadApplicationRuntime(plane_peer, distance_sensor)
        return PayloadApplication(node, router, plane_peer, distance_sensor, runtime)
    except BaseException:
        try:
            node.ble.stop_ble()
        except Exception:
            pass

        node.close()
        raise
