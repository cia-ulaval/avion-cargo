from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from payload.infrastructure.configuration import (
    PayloadRuntimeConfigurationValidation,
    WheelConfiguration,
    parse_payload_runtime_fields,
)
from shared.contracts.geometry import Position
from shared.infrastructure.configuration import integer


@dataclass(frozen=True, slots=True)
class ManualPayloadRuntimeConfiguration(PayloadRuntimeConfigurationValidation):
    """Configuration for a manually controlled payload, which talks to a controller."""

    payload_id: str
    uart_device: str
    controller_peer_id: int
    anchor_id_by_target_id: dict[int, str]
    anchor_positions_by_id: dict[str, Position] = field(default_factory=dict)
    tag_height_m: float | None = None
    range_std_m: float = 0.02
    payload_to_tag_xy_m: tuple[float, float] = (0.0, 0.0)
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
        positioning = raw.get("positioning", {})
        if not isinstance(positioning, dict):
            raise ValueError("positioning must be an object")
        anchors_raw = positioning.get("anchors", {})
        if not isinstance(anchors_raw, dict):
            raise ValueError("positioning.anchors must be an object")
        anchors: dict[str, Position] = {}
        for anchor_id, coordinates in anchors_raw.items():
            if not isinstance(anchor_id, str) or not anchor_id or not isinstance(coordinates, dict):
                raise ValueError("positioning anchor entries must map non-empty IDs to coordinate objects")
            values = [coordinates.get(axis) for axis in ("x", "y", "z")]
            if any(
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in values
            ):
                raise ValueError(f"positioning.anchors.{anchor_id} requires finite numeric x, y, z")
            anchors[anchor_id] = Position(*(float(value) for value in values))
        offset = positioning.get("payload_to_tag_xy_m", [0.0, 0.0])
        if (
            not isinstance(offset, list)
            or len(offset) != 2
            or any(
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in offset
            )
        ):
            raise ValueError("positioning.payload_to_tag_xy_m must be a two-number array")
        config = cls(
            **common_fields,
            controller_peer_id=raw.get("controller_peer_id"),
            anchor_positions_by_id=anchors,
            tag_height_m=positioning.get("tag_height_m"),
            range_std_m=positioning.get("range_std_m", 0.02),
            payload_to_tag_xy_m=(float(offset[0]), float(offset[1])),
        )
        config.validate()
        return config

    def validate(self) -> None:
        self.validate_common()
        integer(self.controller_peer_id, "controller_peer_id", minimum=1, maximum=65534)
        if (
            isinstance(self.range_std_m, bool)
            or not isinstance(self.range_std_m, (int, float))
            or not math.isfinite(self.range_std_m)
            or self.range_std_m <= 0
        ):
            raise ValueError("positioning.range_std_m must be finite and positive")
        if self.tag_height_m is not None and (
            isinstance(self.tag_height_m, bool)
            or not isinstance(self.tag_height_m, (int, float))
            or not math.isfinite(self.tag_height_m)
        ):
            raise ValueError("positioning.tag_height_m must be numeric")
        if self.tag_height_m is not None and not self.anchor_positions_by_id:
            raise ValueError("positioning.anchors are required when tag_height_m is configured")
        if self.anchor_positions_by_id and self.tag_height_m is None:
            raise ValueError("positioning.tag_height_m is required when anchors are configured")
