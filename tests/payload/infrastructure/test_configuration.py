import json

import pytest

from payload.infrastructure.configuration import (
    GazeboMotorDriverConfiguration,
    MX1616MotorDriverConfiguration,
    PayloadRuntimeConfiguration,
    WheelConfiguration,
)

_MISSING = object()


def test_payload_runtime_configuration_parses_target_to_anchor_mapping(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {
                    "role": "initiator",
                    "anchor_id_by_target_id": {"200": "front-left", "201": "front-right"},
                    "period_ms": 100,
                    "timeout_ms": 120,
                    "retries": 1,
                },
            }
        ),
        encoding="utf-8",
    )
    config = PayloadRuntimeConfiguration.from_file(path)
    assert config.uwb_target_ids == (200, 201)
    assert config.anchor_id_by_target_id[201] == "front-right"


def test_payload_runtime_configuration_rejects_invalid_schedule(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {
                    "role": "initiator",
                    "anchor_id_by_target_id": {"200": "front-left"},
                    "period_ms": 1,
                    "timeout_ms": 120,
                    "retries": 1,
                },
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="period_ms"):
        PayloadRuntimeConfiguration.from_file(path)


def test_payload_runtime_configuration_keeps_nvs_uwb_settings_when_omitted(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-1",
                "uart_device": "/dev/serial/by-id/REPLACE_WITH_PAYLOAD_D3K",
                "plane_peer_id": 2,
                "uwb": {
                    "anchor_id_by_target_id": {
                        "2": "plane-anchor-front-left",
                        "3": "plane-anchor-front-right",
                        "4": "plane-anchor-back",
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    config = PayloadRuntimeConfiguration.from_file(path)

    assert config.uwb_target_ids == (2, 3, 4)
    assert config.uwb_role is None
    assert config.uwb_target_ids_override is None
    assert config.uwb_period_ms is None
    assert config.uwb_timeout_ms is None
    assert config.uwb_retries is None
    assert config.uwb_overrides == {}
    assert config.wheels == ()


def test_payload_runtime_configuration_parses_independent_wheels(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": [
                    {
                        "x": 0.1,
                        "y": 0.2,
                        "yaw": 0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {"type": "gazebo", "topic": "/wheel/front_left"},
                    },
                    {
                        "x": -0.15,
                        "y": -0.04,
                        "yaw": -1.2,
                        "radius": 0.05,
                        "max_surface_velocity": 0.5,
                        "driver": {"type": "gazebo", "topic": "/wheel/rear"},
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    config = PayloadRuntimeConfiguration.from_file(path)

    assert len(config.wheels) == 2
    assert isinstance(config.wheels[0], WheelConfiguration)
    assert config.wheels[0].transform.x == pytest.approx(0.1)
    assert config.wheels[0].transform.y == pytest.approx(0.2)
    assert config.wheels[0].transform.yaw == pytest.approx(0.3)
    assert config.wheels[0].transform.radius == pytest.approx(0.04)
    assert config.wheels[0].max_surface_velocity == pytest.approx(0.5)
    assert config.wheels[1].transform.yaw == pytest.approx(-1.2)
    assert config.wheels[1].transform.radius == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("wheels", "message"),
    [
        ({}, "wheels must be a list"),
        (
            [{"x": 0.1, "y": 0.2, "radius": 0.04}],
            "missing required field.*yaw",
        ),
        (
            [{"x": 0.1, "y": 0.2, "yaw": 0.0}],
            "missing required field.*radius",
        ),
        (
            [{"x": 0.1, "y": 0.2, "yaw": 0.0, "radius": 0.0}],
            r"wheels\[0\]\.radius must be a finite positive number",
        ),
        (
            [{"x": 0.1, "y": 0.2, "yaw": 0.0, "radius": -0.01}],
            r"wheels\[0\]\.radius must be a finite positive number",
        ),
        (
            [{"x": 0.1, "y": 0.2, "yaw": 0.0, "radius": float("nan")}],
            r"wheels\[0\]\.radius must be a finite number",
        ),
        (
            [{"x": 0.1, "y": float("nan"), "yaw": 0.0, "radius": 0.04}],
            r"wheels\[0\]\.y must be a finite number",
        ),
        (
            [{"x": True, "y": 0.2, "yaw": 0.0, "radius": 0.04}],
            r"wheels\[0\]\.x must be a finite number",
        ),
    ],
)
def test_payload_runtime_configuration_rejects_invalid_wheel_transforms(tmp_path, wheels, message):
    path = tmp_path / "payload.json"
    if isinstance(wheels, list):
        wheels = [
            {
                **wheel,
                **(
                    {"max_surface_velocity": 0.5, "driver": {"type": "gazebo", "topic": f"/wheel/{index}"}}
                    if {"x", "y", "yaw", "radius"}.issubset(wheel)
                    else {}
                ),
            }
            for index, wheel in enumerate(wheels)
        ]
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": wheels,
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=message):
        PayloadRuntimeConfiguration.from_file(path)


def test_payload_runtime_configuration_parses_partial_uwb_overrides(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {
                    "anchor_id_by_target_id": {"200": "front-left", "201": "front-right"},
                    "target_ids": [200],
                    "period_ms": 150,
                },
            }
        ),
        encoding="utf-8",
    )

    config = PayloadRuntimeConfiguration.from_file(path)

    assert config.uwb_overrides == {"targets": [200], "period_ms": 150}
    assert config.uwb_role is None
    assert config.uwb_timeout_ms is None
    assert config.uwb_retries is None


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"role": "invalid"}, "uwb.role"),
        ({"target_ids": [1, 1]}, "uwb.target_ids must be unique"),
        ({"target_ids": [0]}, "UWB target ID must be in 1..65534"),
        ({"period_ms": 1}, "uwb.period_ms"),
        ({"timeout_ms": 6000}, "uwb.timeout_ms"),
        ({"retries": 6}, "uwb.retries"),
    ],
)
def test_payload_runtime_configuration_validates_optional_uwb_overrides(tmp_path, overrides, message):
    path = tmp_path / "payload.json"
    document = {
        "payload_id": "payload-a",
        "uart_device": "/dev/ttyAMA2",
        "plane_peer_id": 10,
        "uwb": {"anchor_id_by_target_id": {"200": "front-left"}, **overrides},
    }
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        PayloadRuntimeConfiguration.from_file(path)


def test_payload_runtime_configuration_parses_motor_driver_types(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": [
                    {
                        "x": 0.1,
                        "y": 0.2,
                        "yaw": 0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {
                            "type": "gazebo",
                            "topic": "/model/payload/joint/wheel_0_joint/cmd_vel",
                        },
                    },
                    {
                        "x": -0.1,
                        "y": 0.2,
                        "yaw": -0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {
                            "type": "mx1616",
                            "in1_pin": 17,
                            "in2_pin": 18,
                        },
                    },
                    {
                        "x": 0.0,
                        "y": -0.2,
                        "yaw": 1.5,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {
                            "type": "mx1616",
                            "in1_pin": 19,
                            "in2_pin": 20,
                        },
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    config = PayloadRuntimeConfiguration.from_file(path)

    assert isinstance(config.wheels[0].driver, GazeboMotorDriverConfiguration)
    assert config.wheels[0].driver.topic == ("/model/payload/joint/wheel_0_joint/cmd_vel")

    assert isinstance(config.wheels[1].driver, MX1616MotorDriverConfiguration)
    assert (config.wheels[1].driver.in1_pin, config.wheels[1].driver.in2_pin) == (17, 18)

    assert isinstance(config.wheels[2].driver, MX1616MotorDriverConfiguration)
    assert (config.wheels[2].driver.in1_pin, config.wheels[2].driver.in2_pin) == (19, 20)


@pytest.mark.parametrize(
    ("driver", "message"),
    [
        (_MISSING, r"wheels\[0\]\.driver must be an object"),
        (None, r"wheels\[0\]\.driver must be an object"),
        ({"type": "unknown"}, "driver.type must be one of"),
        ({"type": "gazebo"}, "missing required field.*topic"),
        (
            {"type": "gazebo", "topic": "  "},
            "driver.topic must be a non-empty string",
        ),
        (
            {"type": "mx1616", "in1_pin": 17},
            "missing required field.*in2_pin",
        ),
        (
            {"type": "mx1616", "in1_pin": True, "in2_pin": 18},
            "driver.in1_pin must be an integer",
        ),
    ],
)
def test_payload_runtime_configuration_rejects_invalid_motor_drivers(tmp_path, driver, message):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": [
                    {
                        "x": 0.1,
                        "y": 0.2,
                        "yaw": 0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        **({"driver": driver} if driver is not _MISSING else {}),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=message):
        PayloadRuntimeConfiguration.from_file(path)


def test_payload_runtime_configuration_rejects_reused_gpio_pins(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": [
                    {
                        "x": 0.1,
                        "y": 0.2,
                        "yaw": 0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {
                            "type": "mx1616",
                            "in1_pin": 17,
                            "in2_pin": 18,
                        },
                    },
                    {
                        "x": -0.1,
                        "y": 0.2,
                        "yaw": -0.3,
                        "radius": 0.04,
                        "max_surface_velocity": 0.5,
                        "driver": {
                            "type": "mx1616",
                            "in1_pin": 18,
                            "in2_pin": 19,
                        },
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(
        ValueError,
        match="GPIO pin 18 is assigned to more than one",
    ):
        PayloadRuntimeConfiguration.from_file(path)
