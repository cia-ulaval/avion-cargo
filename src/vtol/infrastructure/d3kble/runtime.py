from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from loguru import logger

from shared.infrastructure.d3kble import D3kbleMessageRouter
from vtol.infrastructure.d3kble.configuration import D3KDeviceConfiguration, VtolRuntimeConfiguration
from vtol.infrastructure.d3kble.payload_peer import D3kblePayloadPeer


@dataclass
class _D3KNodeSession:
    configuration: D3KDeviceConfiguration
    node: Any
    ranging_started: bool = False
    ble_start_attempted: bool = False


@dataclass
class VtolD3kbleApplication:
    """
    Own every local D3K node and the single BLE payload-communication node.
    """

    device_sessions: tuple[_D3KNodeSession, ...]
    ble_node: Any | None
    router: D3kbleMessageRouter | None
    payload_peers: tuple[D3kblePayloadPeer, ...]

    @property
    def nodes(self) -> tuple[Any, ...]:
        return tuple(session.node for session in self.device_sessions)

    def close(self) -> None:
        for peer in self.payload_peers:
            peer.close()

        if self.router is not None:
            self.router.close()
        _close_device_nodes(self.device_sessions)


def build_vtol_d3kble_application(
    config: VtolRuntimeConfiguration,
    *,
    node_factory: Callable[..., Any] | None = None,
) -> VtolD3kbleApplication:
    """
    Connect and configure every local D3K board installed on the VTOL.
    """
    config.validate()
    if node_factory is None:
        try:
            from d3kble import D3KNode
        except ImportError as exc:
            raise RuntimeError("The d3kble package is required. Install the vehicle runtime dependencies.") from exc

        node_factory = D3KNode

    device_sessions: list[_D3KNodeSession] = []
    ble_session = None

    try:
        for device_config in config.devices:
            logger.info(
                "Connecting D3K device '{}' on {}",
                device_config.device_id,
                device_config.uart_device,
            )
            node = node_factory(device_config.uart_device, baudrate=device_config.baudrate)
            # Record immediately so a failure during startup still closes this node.
            session = _D3KNodeSession(configuration=device_config, node=node)
            device_sessions.append(session)

            node.start()
            node.wait_until_ready(timeout=device_config.ready_timeout_s)

            uwb_overrides = device_config.uwb.to_d3kble_desired()
            if uwb_overrides:
                node.config.set_desired(uwb_overrides)
                node.config.set_mode("manual")
                result = node.config.reconcile(approve=True)
                if not result.get("success", False):
                    raise RuntimeError(
                        f"D3K device '{device_config.device_id}' did not apply its UWB configuration: {result}"
                    )

                state = node.get_state()
                session.ranging_started = bool(state.status.get("ranging_enabled", False))
                logger.info(
                    "D3K device '{}' applied UWB overrides {} (ranging_enabled={})",
                    device_config.device_id,
                    uwb_overrides,
                    session.ranging_started,
                )

        for session in device_sessions:
            if session.configuration.ble_enabled is True:
                session.ble_start_attempted = True
                session.node.ble.start_ble()
            elif session.configuration.ble_enabled is False:
                session.node.ble.stop_ble()

        ble_device = config.ble_device
        if ble_device is not None:
            ble_session = next(
                session for session in device_sessions if session.configuration.device_id == ble_device.device_id
            )
            ble_node = ble_session.node
            router = D3kbleMessageRouter(ble_node)
            payload_peers = tuple(
                D3kblePayloadPeer(
                    ble_node,
                    router,
                    payload_id=peer.payload_id,
                    peer_id=peer.peer_id,
                )
                for peer in config.payloads
            )
            logger.info(
                "D3K device '{}' owns BLE connections for {} payload(s) (BLE override={})",
                ble_session.configuration.device_id,
                len(ble_session.configuration.payloads),
                ble_session.configuration.ble_enabled,
            )
        else:
            ble_node = None
            router = None
            payload_peers = ()
        return VtolD3kbleApplication(
            device_sessions=tuple(device_sessions),
            ble_node=ble_node,
            router=router,
            payload_peers=payload_peers,
        )
    except BaseException:
        _close_device_nodes(tuple(device_sessions))
        raise


def _close_device_nodes(device_sessions: tuple[_D3KNodeSession, ...]) -> None:
    """Best-effort reverse-order shutdown of started D3K resources."""
    for session in reversed(device_sessions):
        device_config = session.configuration
        node = session.node

        if session.ble_start_attempted:
            try:
                node.ble.stop_ble()
            except Exception:
                logger.exception("Could not stop BLE on D3K device '{}'", device_config.device_id)

        if session.ranging_started:
            try:
                node.uwb.stop_ranging()
            except Exception:
                logger.exception("Could not stop UWB ranging on D3K device '{}'", device_config.device_id)

        try:
            node.close()
        except Exception:
            logger.exception("Could not close D3K device '{}'", device_config.device_id)
