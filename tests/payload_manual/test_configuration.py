import json

import pytest

from payload.infrastructure.configuration import ManualPayloadRuntimeConfiguration


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
