import json

import pytest

from payload_manual_trilateration.infrastructure.configuration import ManualPayloadRuntimeConfiguration


def _manual_configuration(controller_peer_id=3):
    return {
        "payload_id": "payload-manual-simulator",
        "uart_device": "/tmp/payload-anchor",
        "controller_peer_id": controller_peer_id,
        "uwb": {
            "anchor_id_by_target_id": {
                "2": "manual-controller-d3k-node",
            }
        },
        "wheels": [
            {
                "x": 0.0675499815,
                "y": 0.039,
                "yaw": 5.23598775638,
                "radius": 0.027,
                "max_surface_velocity": 0.2,
                "driver": {
                    "type": "gazebo",
                    "topic": "/model/payload/joint/wheel_0_joint/cmd_vel",
                },
            },
            {
                "x": -0.0675499815,
                "y": 0.039,
                "yaw": 1.04719755321,
                "radius": 0.027,
                "max_surface_velocity": 0.2,
                "driver": {
                    "type": "gazebo",
                    "topic": "/model/payload/joint/wheel_1_joint/cmd_vel",
                },
            },
            {
                "x": 0.0,
                "y": -0.078,
                "yaw": 3.14159265359,
                "radius": 0.027,
                "max_surface_velocity": 0.2,
                "driver": {
                    "type": "gazebo",
                    "topic": "/model/payload/joint/wheel_2_joint/cmd_vel",
                },
            },
        ],
    }


def test_manual_payload_configuration_parses_controller_peer_and_per_wheel_limits(tmp_path):
    path = tmp_path / "manual-payload.json"
    path.write_text(json.dumps(_manual_configuration()), encoding="utf-8")

    config = ManualPayloadRuntimeConfiguration.from_file(path)

    assert config.controller_peer_id == 3
    assert config.uwb_target_ids == (2,)
    assert len(config.wheels) == 3
    assert [wheel.max_surface_velocity for wheel in config.wheels] == [0.2, 0.2, 0.2]


def test_manual_payload_configuration_requires_controller_peer_id(tmp_path):
    document = _manual_configuration()
    del document["controller_peer_id"]
    document["plane_peer_id"] = 3
    path = tmp_path / "manual-payload.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError):
        ManualPayloadRuntimeConfiguration.from_file(path)


def test_manual_payload_configuration_parses_positioning_geometry(tmp_path):
    document = _manual_configuration()
    document["positioning"] = {
        "anchors": {
            "a": {"x": 0.0, "y": 0.0, "z": 0.1},
            "b": {"x": 1.0, "y": 0.0, "z": 0.1},
            "c": {"x": 0.5, "y": 1.0, "z": 0.1},
        },
        "tag_height_m": 0.04,
        "range_std_m": 0.015,
        "payload_to_tag_xy_m": [0.02, -0.01],
    }
    path = tmp_path / "manual-payload.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    config = ManualPayloadRuntimeConfiguration.from_file(path)

    assert config.tag_height_m == pytest.approx(0.04)
    assert config.range_std_m == pytest.approx(0.015)
    assert config.anchor_positions_by_id["b"].x == pytest.approx(1.0)
    assert config.payload_to_tag_xy_m == pytest.approx((0.02, -0.01))
