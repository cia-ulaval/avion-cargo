from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from payload.domain.motor_response import (
    LinearMotorResponse,
    MonotonousAffineSegmentsMotorResponse,
    MotorResponse,
    SimpleDeadzoneMotorResponse,
)
from payload.domain.wheel_geometry import WheelTransform2D
from shared.infrastructure.configuration import integer, non_empty_string, positive_number


@dataclass(frozen=True, slots=True)
class GazeboMotorDriverConfiguration:
    type: Literal["gazebo"]
    topic: str


@dataclass(frozen=True, slots=True)
class MX1616MotorDriverConfiguration:
    type: Literal["mx1616"]
    in1_pin: int
    in2_pin: int


MotorDriverConfiguration = GazeboMotorDriverConfiguration | MX1616MotorDriverConfiguration


@dataclass(frozen=True, slots=True)
class WheelConfiguration:
    transform: WheelTransform2D
    driver: MotorDriverConfiguration
    max_surface_velocity: float
    linearization: MotorResponse = LinearMotorResponse()


def _parse_linearization(wheel_path: str, raw: Any) -> MotorResponse:
    """Read the optional ``linearization`` object of a wheel; a missing one means a linear motor."""
    field_name = f"{wheel_path}.linearization"
    if raw is None:
        return LinearMotorResponse()
    if not isinstance(raw, dict):
        raise ValueError(f"{field_name} must be an object")

    fields_by_type = {
        "linear": set(),
        "simple_deadzone": {"dead_zone"},
        "monotonous_affine_segments": {"points"},
    }
    linearization_type = raw.get("type")
    if not isinstance(linearization_type, str) or linearization_type not in fields_by_type:
        raise ValueError(f"{field_name}.type must be one of: {', '.join(sorted(fields_by_type))}")

    expected_fields = fields_by_type[linearization_type]
    missing = expected_fields - raw.keys()
    if missing:
        raise ValueError(f"{field_name} is missing required field(s): {', '.join(sorted(missing))}")
    unexpected = raw.keys() - expected_fields - {"type"}
    if unexpected:
        raise ValueError(f"{field_name} has unsupported field(s): {', '.join(sorted(unexpected))}")

    if linearization_type == "monotonous_affine_segments" and not isinstance(raw["points"], list):
        raise ValueError(f"{field_name}.points must be a list of [duty, speed_fraction] pairs")

    try:
        if linearization_type == "linear":
            return LinearMotorResponse()
        if linearization_type == "simple_deadzone":
            return SimpleDeadzoneMotorResponse(raw["dead_zone"])
        return MonotonousAffineSegmentsMotorResponse(
            tuple(tuple(point) if isinstance(point, list) else point for point in raw["points"])
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name}: {exc}") from exc


def _parse_motor_driver_configuration(
    wheel_path: str,
    raw: Any,
) -> MotorDriverConfiguration:
    field_name = f"{wheel_path}.driver"
    if not isinstance(raw, dict):
        raise ValueError(f"{field_name} must be an object")

    driver_type = raw.get("type")
    fields_by_type = {
        "gazebo": {"topic"},
        "mx1616": {"in1_pin", "in2_pin"},
    }
    if not isinstance(driver_type, str) or driver_type not in fields_by_type:
        raise ValueError(f"{field_name}.type must be one of: gazebo, mx1616")

    expected_fields = fields_by_type[driver_type]
    missing = expected_fields - raw.keys()
    if missing:
        missing_fields = ", ".join(sorted(missing))
        raise ValueError(f"{field_name} is missing required field(s): {missing_fields}")

    unexpected = raw.keys() - expected_fields - {"type"}
    if unexpected:
        unexpected_fields = ", ".join(sorted(unexpected))
        raise ValueError(f"{field_name} has unsupported field(s): {unexpected_fields}")

    if driver_type == "gazebo":
        topic = raw["topic"]
        non_empty_string(topic, f"{field_name}.topic")
        return GazeboMotorDriverConfiguration(type="gazebo", topic=topic)

    pins = {name: raw[name] for name in sorted(expected_fields)}
    for name, pin in pins.items():
        integer(pin, f"{field_name}.{name}", minimum=0)
    if len(set(pins.values())) != len(pins):
        raise ValueError(f"{field_name} GPIO pins must be distinct")

    return MX1616MotorDriverConfiguration(
        type="mx1616",
        in1_pin=raw["in1_pin"],
        in2_pin=raw["in2_pin"],
    )


def parse_payload_runtime_fields(path: str | Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Parse common hardware and UWB settings from a payload configuration."""
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = json.load(stream)

    if not isinstance(raw, dict):
        raise ValueError("payload configuration must be a JSON object")

    raw_wheels = raw.get("wheels", [])
    if not isinstance(raw_wheels, list):
        raise ValueError("wheels must be a list")

    wheels: list[WheelConfiguration] = []
    for wheel_index, raw_wheel in enumerate(raw_wheels):
        wheel_path = f"wheels[{wheel_index}]"
        if not isinstance(raw_wheel, dict):
            raise ValueError(f"{wheel_path} must be an object")

        missing = {"x", "y", "yaw", "radius", "max_surface_velocity"} - raw_wheel.keys()
        if missing:
            missing_fields = ", ".join(sorted(missing))
            raise ValueError(f"{wheel_path} is missing required field(s): {missing_fields}")

        values: dict[str, float] = {}
        for axis in ("x", "y", "yaw", "radius"):
            value = raw_wheel[axis]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{wheel_path}.{axis} must be a finite number")
            values[axis] = float(value)

        positive_number(values["radius"], f"{wheel_path}.radius")
        max_surface_velocity = raw_wheel["max_surface_velocity"]
        positive_number(
            max_surface_velocity,
            f"{wheel_path}.max_surface_velocity",
        )
        max_surface_velocity = float(max_surface_velocity)
        transform = WheelTransform2D(**values)

        driver = _parse_motor_driver_configuration(wheel_path, raw_wheel.get("driver"))
        linearization = _parse_linearization(wheel_path, raw_wheel.get("linearization"))
        wheels.append(
            WheelConfiguration(
                transform=transform,
                driver=driver,
                max_surface_velocity=max_surface_velocity,
                linearization=linearization,
            )
        )

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

    common_fields = {
        "payload_id": raw.get("payload_id"),
        "uart_device": raw.get("uart_device"),
        "anchor_id_by_target_id": anchor_mapping,
        "wheels": tuple(wheels),
        "uwb_role": uwb.get("role"),
        "uwb_target_ids_override": tuple(raw_target_ids) if raw_target_ids is not None else None,
        "uwb_period_ms": uwb.get("period_ms"),
        "uwb_timeout_ms": uwb.get("timeout_ms"),
        "uwb_retries": uwb.get("retries"),
        "baudrate": raw.get("baudrate", 460800),
        "ready_timeout_s": raw.get("ready_timeout_s", 10.0),
    }
    return raw, common_fields


class PayloadRuntimeConfigurationValidation:
    """Provide common validation and UWB helpers for payload runtime configurations."""

    __slots__ = ()

    payload_id: str
    uart_device: str
    anchor_id_by_target_id: dict[int, str]
    wheels: tuple[WheelConfiguration, ...]
    uwb_role: str | None
    uwb_target_ids_override: tuple[int, ...] | None
    uwb_period_ms: int | None
    uwb_timeout_ms: int | None
    uwb_retries: int | None
    baudrate: int
    ready_timeout_s: float

    @property
    def uwb_target_ids(self) -> tuple[int, ...]:
        """Target IDs known to the application mapping (not a device override)."""
        return tuple(self.anchor_id_by_target_id)

    @property
    def uwb_overrides(self) -> dict[str, Any]:
        """Return only explicit runtime overrides in d3kble ConfigManager form."""
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

    def validate_common(self) -> None:
        non_empty_string(self.payload_id, "payload_id")
        non_empty_string(self.uart_device, "uart_device")
        integer(self.baudrate, "baudrate", minimum=1)
        positive_number(self.ready_timeout_s, "ready_timeout_s")

        if not isinstance(self.wheels, tuple):
            raise ValueError("wheels must be a tuple")

        configured_gpio_pins: set[int] = set()
        for wheel_index, wheel in enumerate(self.wheels):
            wheel_path = f"wheels[{wheel_index}]"
            if not isinstance(wheel, WheelConfiguration):
                raise ValueError(f"{wheel_path} must be a WheelConfiguration")

            transform = wheel.transform
            if not isinstance(transform, WheelTransform2D):
                raise ValueError(f"{wheel_path}.transform must be a WheelTransform2D")
            for axis in ("x", "y", "yaw", "radius"):
                value = getattr(transform, axis)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"{wheel_path}.{axis} must be a finite number")
            positive_number(transform.radius, f"{wheel_path}.radius")
            positive_number(
                wheel.max_surface_velocity,
                f"{wheel_path}.max_surface_velocity",
            )

            if not callable(getattr(wheel.linearization, "inverse_response", None)):
                raise ValueError(f"{wheel_path}.linearization must implement inverse_response")

            driver = wheel.driver
            if isinstance(driver, GazeboMotorDriverConfiguration):
                non_empty_string(driver.topic, f"{wheel_path}.driver.topic")
                continue

            if isinstance(driver, MX1616MotorDriverConfiguration):
                pins = (driver.in1_pin, driver.in2_pin)
            else:
                raise ValueError(f"{wheel_path}.driver has an unsupported driver configuration")

            for pin_name, pin in zip(("in1_pin", "in2_pin"), pins):
                integer(pin, f"{wheel_path}.driver.{pin_name}", minimum=0)
                if pin in configured_gpio_pins:
                    raise ValueError(f"GPIO pin {pin} is assigned to more than one wheel driver")
                configured_gpio_pins.add(pin)
            if len(set(pins)) != len(pins):
                raise ValueError(f"{wheel_path}.driver GPIO pins must be distinct")

        if self.uwb_role is not None and (
            not isinstance(self.uwb_role, str) or self.uwb_role not in {"idle", "initiator", "responder"}
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


@dataclass(frozen=True, slots=True)
class PayloadRuntimeConfiguration(PayloadRuntimeConfigurationValidation):
    """Configuration for the mission-oriented payload, which talks to a plane."""

    payload_id: str
    uart_device: str
    plane_peer_id: int
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
    def from_file(cls, path: str | Path) -> PayloadRuntimeConfiguration:
        raw, common_fields = parse_payload_runtime_fields(path)
        config = cls(**common_fields, plane_peer_id=raw.get("plane_peer_id"))
        config.validate()
        return config

    def validate(self) -> None:
        self.validate_common()
        integer(self.plane_peer_id, "plane_peer_id", minimum=1, maximum=65534)
