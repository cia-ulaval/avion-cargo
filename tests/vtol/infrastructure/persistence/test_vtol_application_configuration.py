import json

from vtol.infrastructure.persistence.vtol_application_configuration import VtolApplicationConfiguration


def test_vtol_application_configuration_loads_autolanding_and_all_d3k_devices(tmp_path, valid_config_data):
    document = {
        "autolanding": valid_config_data,
        "d3kble": {
            "devices": [
                {
                    "device_id": "anchor-a",
                    "uart_device": "/dev/ttyUSB0",
                    "uwb": {
                        "enabled": True,
                        "role": "responder",
                        "period_ms": 100,
                        "timeout_ms": 120,
                        "retries": 1,
                    },
                },
                {
                    "device_id": "anchor-b",
                    "uart_device": "/dev/ttyUSB1",
                    "uwb": {
                        "enabled": True,
                        "role": "responder",
                        "period_ms": 100,
                        "timeout_ms": 120,
                        "retries": 1,
                    },
                    "ble": {
                        "payloads": [{"payload_id": "payload-a", "peer_id": 20}],
                    },
                },
            ]
        },
    }
    path = tmp_path / "vtol.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    config = VtolApplicationConfiguration.from_file(path)

    assert config.autolanding.camera_config.id == valid_config_data["camera"]["id"]
    assert config.autolanding.drone_connection_config.address == valid_config_data["drone_connection"]["address"]
    assert len(config.d3kble.devices) == 2
    assert config.d3kble.ble_device.device_id == "anchor-b"
    assert [device.ble_enabled for device in config.d3kble.devices] == [None, None]
    assert [peer.payload_id for peer in config.d3kble.payloads] == ["payload-a"]
