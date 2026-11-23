import json

import pytest

from payload.infrastructure.configuration import PayloadRuntimeConfiguration


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
    assert config.wheel_transforms == {}


def test_payload_runtime_configuration_parses_independent_wheel_transforms(tmp_path):
    path = tmp_path / "payload.json"
    path.write_text(
        json.dumps(
            {
                "payload_id": "payload-a",
                "uart_device": "/dev/ttyAMA2",
                "plane_peer_id": 10,
                "uwb": {"anchor_id_by_target_id": {"200": "front-left"}},
                "wheels": {
                    "front_left": {"x": 0.1, "y": 0.2, "yaw": 0.3},
                    "rear": {"x": -0.15, "y": -0.04, "yaw": -1.2},
                },
            }
        ),
        encoding="utf-8",
    )

    config = PayloadRuntimeConfiguration.from_file(path)

    assert config.wheel_transforms["front_left"].x == pytest.approx(0.1)
    assert config.wheel_transforms["front_left"].y == pytest.approx(0.2)
    assert config.wheel_transforms["front_left"].yaw == pytest.approx(0.3)
    assert config.wheel_transforms["rear"].yaw == pytest.approx(-1.2)


@pytest.mark.parametrize(
    ("wheels", "message"),
    [
        ([], "wheels must be an object"),
        ({"wheel_0": {"x": 0.1, "y": 0.2}}, "missing required field.*yaw"),
        (
            {"wheel_0": {"x": 0.1, "y": float("nan"), "yaw": 0.0}},
            "wheels.wheel_0.y must be a finite number",
        ),
        (
            {"wheel_0": {"x": True, "y": 0.2, "yaw": 0.0}},
            "wheels.wheel_0.x must be a finite number",
        ),
    ],
)
def test_payload_runtime_configuration_rejects_invalid_wheel_transforms(tmp_path, wheels, message):
    path = tmp_path / "payload.json"
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
def test_payload_runtime_configuration_validates_optional_uwb_overrides(
    tmp_path, overrides, message
):
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
