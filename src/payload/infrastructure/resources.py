from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any, Callable

from loguru import logger

from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload.infrastructure.d3kble import D3kbleDistanceSensor
from shared.infrastructure.d3kble import D3kbleMessageRouter


@dataclass(slots=True)
class PayloadInfrastructure:
    """
    Resources shared by every payload application entrypoint.
    """

    node: Any
    router: D3kbleMessageRouter
    distance_sensor: D3kbleDistanceSensor
    _closed: bool = field(default=False, init=False, repr=False)

    def close(self) -> None:
        """Stop shared services and close the local D3K node exactly once."""
        if self._closed:
            return
        self._closed = True
        try:
            self.router.close()
        except Exception:
            logger.exception("Could not close the payload D3K message router cleanly")
        try:
            self.node.ble.stop_ble()
        except Exception:
            logger.exception("Could not stop the payload BLE service cleanly")
        finally:
            self.node.close()

    async def aclose(self) -> None:
        """Asynchronous counterpart for applications with async shutdown."""
        await asyncio.to_thread(self.close)


def build_payload_infrastructure(
    config: PayloadRuntimeConfiguration,
    *,
    node_factory: Callable[..., Any] | None = None,
) -> PayloadInfrastructure:
    """Start the shared D3K/BLE/UWB infrastructure for a payload entrypoint."""
    if node_factory is None:
        try:
            from d3kble import D3KNode
        except ImportError as exc:
            raise RuntimeError(
                "The d3kble package is required. Install the vehicle runtime dependencies."
            ) from exc
        node_factory = D3KNode

    node = node_factory(config.uart_device, baudrate=config.baudrate)
    router: D3kbleMessageRouter | None = None
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
        distance_sensor = D3kbleDistanceSensor(
            node,
            anchor_id_by_target_id=config.anchor_id_by_target_id,
        )
        return PayloadInfrastructure(node, router, distance_sensor)
    except BaseException:
        if router is not None:
            try:
                router.close()
            except Exception:
                logger.exception("Could not close the payload D3K message router after startup failure")
        try:
            node.ble.stop_ble()
        except Exception:
            pass
        finally:
            node.close()
        raise
