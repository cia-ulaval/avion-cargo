from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from shared.infrastructure.configuration import integer, non_empty_string, positive_number


@dataclass(frozen=True, slots=True)
class PayloadRuntimeConfiguration:
    payload_id: str
    uart_device: str
    plane_peer_id: int
    anchor_id_by_target_id: dict[int, str]
    uwb_role: str | None = None
    uwb_target_ids_override: tuple[int, ...] | None = None
    uwb_period_ms: int | None = None
    uwb_timeout_ms: int | None = None
    uwb_retries: int | None = None
    baudrate: int = 460800
    ready_timeout_s: float = 10.0

    @property
    def uwb_target_ids(self) -> tuple[int, ...]:
        """Target IDs known to the application mapping (not a device override)."""
        return tuple(self.anchor_id_by_target_id)

    @property
    def uwb_overrides(self) -> dict[str, Any]:
        """
        Return only explicit runtime overrides in d3kble ConfigManager form.
        """
        desired: dict[str, Any] = {}
        if self.uwb_role is not None:
            desired["role"] = self.uwb_role
        if self.uwb_target_ids_override is not None:
            desired["targets"] = list(self.uwb_target_ids_override)
        for name in ("period_ms", "timeout_ms", "retries"):
            value = getattr(self, f"uwb_{name}")
            if value is not None:
                desired[name] = value
        return desired

    @classmethod
    def from_file(cls, path: str | Path) -> PayloadRuntimeConfiguration:
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
            
        if not isinstance(raw, dict):
            raise ValueError("payload configuration must be a JSON object")

        uwb = raw.get("uwb")

        if not isinstance(uwb, dict):
            raise ValueError("payload configuration requires an 'uwb' object")
        
        mapping = uwb.get("anchor_id_by_target_id")

        if not isinstance(mapping, dict) or not mapping:
            raise ValueError("uwb.anchor_id_by_target_id must be a non-empty object")
        
        try:
            converted = [(int(target_id), anchor_id) for target_id, anchor_id in mapping.items()]
        except (TypeError, ValueError) as exc:
            raise ValueError("UWB target IDs in anchor_id_by_target_id must be integers") from exc
        
        anchor_mapping = dict(converted)

        if len(anchor_mapping) != len(converted):
            raise ValueError("UWB target IDs must be unique after integer conversion")

        raw_target_ids = uwb.get("target_ids")
        if raw_target_ids is not None and not isinstance(raw_target_ids, list):
            raise ValueError("uwb.target_ids must be a list")

        config = cls(
            payload_id=raw.get("payload_id"),
            uart_device=raw.get("uart_device"),
            plane_peer_id=raw.get("plane_peer_id"),
            anchor_id_by_target_id=anchor_mapping,
            uwb_role=uwb.get("role"),
            uwb_target_ids_override=(tuple(raw_target_ids) if raw_target_ids is not None else None),
            uwb_period_ms=uwb.get("period_ms"),
            uwb_timeout_ms=uwb.get("timeout_ms"),
            uwb_retries=uwb.get("retries"),
            baudrate=raw.get("baudrate", 460800),
            ready_timeout_s=raw.get("ready_timeout_s", 10.0),
        )
        config.validate()
        return config

    def validate(self) -> None:
        non_empty_string(self.payload_id, "payload_id")
        non_empty_string(self.uart_device, "uart_device")
        integer(self.plane_peer_id, "plane_peer_id", minimum=1, maximum=65534)
        integer(self.baudrate, "baudrate", minimum=1)
        positive_number(self.ready_timeout_s, "ready_timeout_s")

        if self.uwb_role is not None and (
            not isinstance(self.uwb_role, str)
            or self.uwb_role not in {"idle", "initiator", "responder"}
        ):
            raise ValueError("uwb.role must be 'idle', 'initiator', 'responder', or null")

        if self.uwb_target_ids_override is not None:
            if not isinstance(self.uwb_target_ids_override, tuple):
                raise ValueError("uwb.target_ids must be a list of integers")
            if len(self.uwb_target_ids_override) > 16:
                raise ValueError("uwb.target_ids supports at most 16 target IDs")
            for target_id in self.uwb_target_ids_override:
                integer(target_id, "UWB target ID", minimum=1, maximum=65534)
            if len(set(self.uwb_target_ids_override)) != len(self.uwb_target_ids_override):
                raise ValueError("uwb.target_ids must be unique")

        if self.uwb_period_ms is not None:
            integer(self.uwb_period_ms, "uwb.period_ms", minimum=20, maximum=60000)
        if self.uwb_timeout_ms is not None:
            integer(self.uwb_timeout_ms, "uwb.timeout_ms", minimum=50, maximum=5000)
        if self.uwb_retries is not None:
            integer(self.uwb_retries, "uwb.retries", minimum=0, maximum=5)

        if not self.anchor_id_by_target_id:
            raise ValueError("at least one UWB target-to-anchor mapping is required")
        
        if len(self.anchor_id_by_target_id) > 16:
            raise ValueError("at most 16 UWB targets can be configured")
        
        for target_id, anchor_id in self.anchor_id_by_target_id.items():
            integer(target_id, "UWB target ID", minimum=1, maximum=65534)
            non_empty_string(anchor_id, "UWB anchor ID")

        anchor_ids = tuple(self.anchor_id_by_target_id.values())
        if len(set(anchor_ids)) != len(anchor_ids):
            raise ValueError("application anchor IDs must be unique")
