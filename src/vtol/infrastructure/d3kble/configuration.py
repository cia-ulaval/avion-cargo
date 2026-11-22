from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from shared.infrastructure.configuration import integer, non_empty_string, positive_number


@dataclass(frozen=True, slots=True)
class PayloadPeerConfiguration:
    payload_id: str
    peer_id: int


@dataclass(frozen=True, slots=True)
class UwbDeviceConfiguration:
    """
    Optional runtime overrides for the UWB-related D3K configuration.
    """

    enabled: bool | None = None
    role: str | None = None
    target_ids: tuple[int, ...] | None = None
    device_id: int | None = None
    pan_id: int | None = None
    channel: int | None = None
    tx_antenna_delay: int | None = None
    rx_antenna_delay: int | None = None
    period_ms: int | None = None
    timeout_ms: int | None = None
    retries: int | None = None

    def to_d3kble_desired(self) -> dict[str, Any]:
        """Return only explicitly configured fields in d3kble ConfigManager form.

        The mapping intentionally remains partial. In particular, this method
        never supplies default schedule values, target lists, roles, or service
        state, so settings prepared with d3km remain untouched unless explicitly
        overridden here.
        """
        desired: dict[str, Any] = {}
        if self.enabled is not None:
            desired["ranging_enabled"] = self.enabled
        if self.role is not None:
            desired["role"] = self.role
        if self.target_ids is not None:
            desired["targets"] = list(self.target_ids)

        for name in (
            "device_id",
            "pan_id",
            "channel",
            "tx_antenna_delay",
            "rx_antenna_delay",
            "period_ms",
            "timeout_ms",
            "retries",
        ):
            value = getattr(self, name)
            if value is not None:
                desired[name] = value

        return desired


@dataclass(frozen=True, slots=True)
class D3KDeviceConfiguration:
    """Configuration for one physical ESP32/DWM3000 board on the VTOL."""

    device_id: str
    uart_device: str
    baudrate: int
    ready_timeout_s: float
    uwb: UwbDeviceConfiguration
    ble_enabled: bool | None = None
    payloads: tuple[PayloadPeerConfiguration, ...] = ()


@dataclass(frozen=True, slots=True)
class VtolRuntimeConfiguration:
    """
    D3Kble configuration for every local board installed on the VTOL.
    """

    devices: tuple[D3KDeviceConfiguration, ...]

    @property
    def ble_device(self) -> D3KDeviceConfiguration | None:
        """Return the sole node assigned to BLE payload communication, if any.

        A peer mapping selects the communication node independently of the
        BLE service override. This lets the application keep peer definitions
        while leaving the service in its firmware-configured state (or
        explicitly disabling it).
        """
        candidates = [
            device for device in self.devices
            if device.payloads or device.ble_enabled is True
        ]
        return candidates[0] if candidates else None

    @property
    def payloads(self) -> tuple[PayloadPeerConfiguration, ...]:
        """Payload peers attached to the configured BLE communication node."""
        device = self.ble_device
        return device.payloads if device is not None else ()

    @classmethod
    def from_file(cls, path: str | Path) -> VtolRuntimeConfiguration:
        with Path(path).open("r", encoding="utf-8") as stream:
            root = json.load(stream)

        if not isinstance(root, dict):
            raise ValueError("VTOL configuration must be a JSON object")

        raw = root.get("d3kble", root)

        if not isinstance(raw, dict):
            raise ValueError("d3kble configuration must be an object")
        
        _no_extra_keys(raw, {"devices"}, "d3kble")

        raw_devices = raw.get("devices")

        if not isinstance(raw_devices, list) or not raw_devices:
            raise ValueError("d3kble.devices must be a non-empty list")

        devices = tuple(_parse_device(item, index) for index, item in enumerate(raw_devices))
        config = cls(devices=devices)
        config.validate()
        return config

    def validate(self) -> None:
        if not self.devices:
            raise ValueError("at least one local D3K device must be configured")

        for device in self.devices:
            if not isinstance(device, D3KDeviceConfiguration):
                raise ValueError("devices must contain D3KDeviceConfiguration values")
            non_empty_string(device.device_id, "device_id")
            non_empty_string(device.uart_device, f"{device.device_id}.uart_device")
            integer(device.baudrate, f"{device.device_id}.baudrate", minimum=1)
            positive_number(device.ready_timeout_s, f"{device.device_id}.ready_timeout_s")
            _validate_uwb(device.uwb, device.device_id)
            if device.ble_enabled is not None and not isinstance(device.ble_enabled, bool):
                raise ValueError(f"{device.device_id}.ble.enabled must be a boolean or null")

        device_ids = [device.device_id for device in self.devices]
        uart_devices = [device.uart_device for device in self.devices]
        if len(device_ids) != len(set(device_ids)):
            raise ValueError("D3K device_id values must be unique")
        if len(uart_devices) != len(set(uart_devices)):
            raise ValueError("D3K UART device paths must be unique")

        ble_devices = [
            device for device in self.devices
            if device.payloads or device.ble_enabled is True
        ]
        if len(ble_devices) > 1:
            raise ValueError(
                "only one local D3K device can own BLE payload communication or explicitly enable BLE"
            )

        payloads = ble_devices[0].payloads if ble_devices else ()
        if len(payloads) > 8:
            raise ValueError("the BLE communication node supports at most eight payload peers")

        for peer in payloads:
            if not isinstance(peer, PayloadPeerConfiguration):
                raise ValueError("ble.payloads must contain payload peer configurations")
            non_empty_string(peer.payload_id, "payload_id")
            integer(peer.peer_id, "BLE peer_id", minimum=1, maximum=65534)

        payload_ids = [peer.payload_id for peer in payloads]
        peer_ids = [peer.peer_id for peer in payloads]
        if len(payload_ids) != len(set(payload_ids)):
            raise ValueError("payload_id values must be unique")
        if len(peer_ids) != len(set(peer_ids)):
            raise ValueError("BLE peer IDs must be unique")


def _parse_device(raw: Any, index: int) -> D3KDeviceConfiguration:
    name = f"d3kble.devices[{index}]"
    
    if not isinstance(raw, dict):
        raise ValueError(f"{name} must be an object")
    
    _no_extra_keys(
        raw,
        {"device_id", "uart_device", "baudrate", "ready_timeout_s", "uwb", "ble"},
        name,
    )

    device_id = raw.get("device_id")
    uart_device = raw.get("uart_device")
    baudrate = raw.get("baudrate", 460800)
    ready_timeout_s = raw.get("ready_timeout_s", 10.0)
    raw_uwb = raw.get("uwb", {})

    if raw_uwb is None:
        raw_uwb = {}

    if not isinstance(raw_uwb, dict):
        raise ValueError(f"{name}.uwb must be an object")
    
    _no_extra_keys(
        raw_uwb,
        {
            "enabled",
            "role",
            "target_ids",
            "device_id",
            "pan_id",
            "channel",
            "tx_antenna_delay",
            "rx_antenna_delay",
            "period_ms",
            "timeout_ms",
            "retries",
        },
        f"{name}.uwb",
    )

    raw_target_ids = raw_uwb.get("target_ids")

    if raw_target_ids is not None and not isinstance(raw_target_ids, list):
        raise ValueError(f"{name}.uwb.target_ids must be a list or null")

    uwb = UwbDeviceConfiguration(
        enabled=raw_uwb.get("enabled"),
        role=raw_uwb.get("role"),
        target_ids=tuple(raw_target_ids) if raw_target_ids is not None else None,
        device_id=raw_uwb.get("device_id"),
        pan_id=raw_uwb.get("pan_id"),
        channel=raw_uwb.get("channel"),
        tx_antenna_delay=raw_uwb.get("tx_antenna_delay"),
        rx_antenna_delay=raw_uwb.get("rx_antenna_delay"),
        period_ms=raw_uwb.get("period_ms"),
        timeout_ms=raw_uwb.get("timeout_ms"),
        retries=raw_uwb.get("retries"),
    )
    raw_ble = raw.get("ble", {})

    if raw_ble is None:
        raw_ble = {}

    if not isinstance(raw_ble, dict):
        raise ValueError(f"{name}.ble must be an object or null")
    
    _no_extra_keys(raw_ble, {"enabled", "payloads"}, f"{name}.ble")
    ble_enabled = raw_ble.get("enabled")
    raw_payloads = raw_ble.get("payloads", [])

    if not isinstance(raw_payloads, list):
        raise ValueError(f"{name}.ble.payloads must be a list")

    payloads: list[PayloadPeerConfiguration] = []

    for peer_index, item in enumerate(raw_payloads):
        if not isinstance(item, dict):
            raise ValueError(f"{name}.ble.payloads[{peer_index}] must be an object")
        _no_extra_keys(item, {"payload_id", "peer_id"}, f"{name}.ble.payloads[{peer_index}]")
        payloads.append(
            PayloadPeerConfiguration(
                payload_id=item.get("payload_id"),
                peer_id=item.get("peer_id"),
            )
        )

    return D3KDeviceConfiguration(
        device_id=device_id,
        uart_device=uart_device,
        baudrate=baudrate,
        ready_timeout_s=ready_timeout_s,
        uwb=uwb,
        ble_enabled=ble_enabled,
        payloads=tuple(payloads),
    )


def _validate_uwb(config: UwbDeviceConfiguration, device_id: str) -> None:
    prefix = f"{device_id}.uwb"
    if not isinstance(config, UwbDeviceConfiguration):
        raise ValueError(f"{prefix} must be a UwbDeviceConfiguration")

    if config.enabled is not None and not isinstance(config.enabled, bool):
        raise ValueError(f"{prefix}.enabled must be a boolean or null")
    if config.role is not None and (
        not isinstance(config.role, str) or config.role not in {"idle", "initiator", "responder"}
    ):
        raise ValueError(f"{prefix}.role must be 'idle', 'initiator', 'responder', or null")
    if config.enabled is True and config.role == "idle":
        raise ValueError(f"{prefix}.enabled cannot be true when role is 'idle'")

    if config.target_ids is not None:
        if not isinstance(config.target_ids, tuple):
            raise ValueError(f"{prefix}.target_ids must be a tuple of integers")
        if len(config.target_ids) > 16:
            raise ValueError(f"{prefix}.target_ids supports at most 16 target IDs")
        for target_id in config.target_ids:
            integer(target_id, f"{prefix}.target_ids entry", minimum=1, maximum=65534)
        if len(config.target_ids) != len(set(config.target_ids)):
            raise ValueError(f"{prefix}.target_ids must be unique")
        if config.enabled is True and config.role == "initiator" and not config.target_ids:
            raise ValueError(f"{prefix}.target_ids must not be empty for an enabled initiator")

    if config.device_id is not None:
        integer(config.device_id, f"{prefix}.device_id", minimum=1, maximum=65534)
    if config.pan_id is not None:
        integer(config.pan_id, f"{prefix}.pan_id", minimum=0, maximum=65535)
    if config.channel is not None:
        integer(config.channel, f"{prefix}.channel", minimum=5, maximum=9)
        if config.channel not in (5, 9):
            raise ValueError(f"{prefix}.channel must be 5 or 9")
    if config.tx_antenna_delay is not None:
        integer(config.tx_antenna_delay, f"{prefix}.tx_antenna_delay", minimum=0, maximum=65535)
    if config.rx_antenna_delay is not None:
        integer(config.rx_antenna_delay, f"{prefix}.rx_antenna_delay", minimum=0, maximum=65535)
    if config.period_ms is not None:
        integer(config.period_ms, f"{prefix}.period_ms", minimum=20, maximum=60000)
    if config.timeout_ms is not None:
        integer(config.timeout_ms, f"{prefix}.timeout_ms", minimum=50, maximum=5000)
    if config.retries is not None:
        integer(config.retries, f"{prefix}.retries", minimum=0, maximum=5)


def _no_extra_keys(value: dict[str, Any], allowed: set[str], path: str) -> None:
    extra = set(value) - allowed
    if extra:
        raise ValueError(f"Unknown keys at {path}: {sorted(extra)}")
