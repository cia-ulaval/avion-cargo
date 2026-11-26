from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from payload.infrastructure.configuration import (
    PayloadRuntimeConfigurationValidation,
    WheelConfiguration,
    parse_payload_runtime_fields,
)
from shared.infrastructure.configuration import integer


@dataclass(frozen=True, slots=True)
class ManualPayloadRuntimeConfiguration(PayloadRuntimeConfigurationValidation):
    """Configuration for a manually controlled payload, which talks to a controller."""

    payload_id: str
    uart_device: str
    controller_peer_id: int
    anchor_id_by_target_id: dict[int, str]
    wheels: tuple[WheelConfiguration, ...] = ()
    uwb_role: str | None = None
    uwb_target_ids_override: tuple[int, ...] | None = None
    uwb_period_ms: int | None = None
    uwb_timeout_ms: int | None = None
    uwb_retries: int | None = None
    baudrate: int = 460800
    ready_timeout_s: float = 10.0

    @classmethod
    def from_file(cls, path: str | Path) -> ManualPayloadRuntimeConfiguration:
        raw, common_fields = parse_payload_runtime_fields(path)
        config = cls(**common_fields, controller_peer_id=raw.get("controller_peer_id"))
        config.validate()
        return config

    def validate(self) -> None:
        self.validate_common()
        integer(self.controller_peer_id, "controller_peer_id", minimum=1, maximum=65534)
