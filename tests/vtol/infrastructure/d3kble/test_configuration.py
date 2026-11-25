import json

import pytest

from vtol.infrastructure.d3kble.configuration import VtolRuntimeConfiguration


def d3k_device(
    device_id,
    uart_device,
    *,
    ble_enabled=None,
    payloads=None,
    role="responder",
    targets=None,
    uwb_overrides=None,
    include_uwb=True,
):
    ble = {}
    if ble_enabled is not None:
        ble["enabled"] = ble_enabled
    if payloads is not None:
        ble["payloads"] = payloads
    uwb = {"enabled": True, "role": role}
    if targets is not None:
        uwb["target_ids"] = targets
    if uwb_overrides is not None:
        uwb.update(uwb_overrides)
    device = {
        "device_id": device_id,
        "uart_device": uart_device,
        "baudrate": 460800,
        "ready_timeout_s": 10.0,
    }
    if ble_enabled is not None or payloads is not None:
        device["ble"] = ble
    if include_uwb:
        device["uwb"] = uwb
    return device


def vtol_document(devices):
    return {"d3kble": {"devices": devices}}


def test_vtol_runtime_configuration_parses_multiple_local_devices_and_payload_peers(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device("anchor-a", "/dev/ttyUSB0"),
                    d3k_device("anchor-b", "/dev/ttyUSB1"),
                    d3k_device(
                        "ble-anchor",
                        "/dev/ttyUSB2",
                        ble_enabled=True,
                        payloads=[
                            {"payload_id": "payload-a", "peer_id": 20},
                            {"payload_id": "payload-b", "peer_id": 21},
                        ],
                    ),
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)

    assert [device.device_id for device in config.devices] == ["anchor-a", "anchor-b", "ble-anchor"]
    assert [device.uart_device for device in config.devices] == ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
    assert all(device.uwb.enabled is True and device.uwb.role == "responder" for device in config.devices)
    assert all(device.uwb.period_ms is None for device in config.devices)
    assert all(device.uwb.timeout_ms is None for device in config.devices)
    assert all(device.uwb.retries is None for device in config.devices)
    assert config.ble_device.device_id == "ble-anchor"
    assert [(peer.payload_id, peer.peer_id) for peer in config.payloads] == [
        ("payload-a", 20),
        ("payload-b", 21),
    ]


def test_vtol_runtime_configuration_parses_optional_initiator_targets(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "initiator",
                        "/dev/ttyUSB0",
                        role="initiator",
                        targets=[200, 201],
                        ble_enabled=True,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)

    assert config.devices[0].uwb.target_ids == (200, 201)


def test_uwb_section_and_all_uwb_fields_are_optional(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "ble-node",
                        "/dev/ttyUSB0",
                        ble_enabled=True,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                        include_uwb=False,
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)
    uwb = config.devices[0].uwb

    assert uwb.enabled is None
    assert uwb.role is None
    assert uwb.target_ids is None
    assert uwb.device_id is None
    assert uwb.pan_id is None
    assert uwb.channel is None
    assert uwb.tx_antenna_delay is None
    assert uwb.rx_antenna_delay is None
    assert uwb.period_ms is None
    assert uwb.timeout_ms is None
    assert uwb.retries is None
    assert uwb.to_d3kble_desired() == {}


def test_uwb_partial_overrides_include_only_explicit_values(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "ble-node",
                        "/dev/ttyUSB0",
                        ble_enabled=True,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                        role=None,
                        uwb_overrides={
                            "device_id": 42,
                            "pan_id": 0xCAFE,
                            "channel": 5,
                            "tx_antenna_delay": 16436,
                            "rx_antenna_delay": 16436,
                            "period_ms": 150,
                        },
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)
    uwb = config.devices[0].uwb

    assert uwb.to_d3kble_desired() == {
        "ranging_enabled": True,
        "device_id": 42,
        "pan_id": 0xCAFE,
        "channel": 5,
        "tx_antenna_delay": 16436,
        "rx_antenna_delay": 16436,
        "period_ms": 150,
    }


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"channel": 7}, "channel must be 5 or 9"),
        ({"device_id": 0}, "device_id must be in 1..65534"),
        ({"period_ms": 10}, "period_ms must be in 20..60000"),
        ({"timeout_ms": 6000}, "timeout_ms must be in 50..5000"),
        ({"retries": 6}, "retries must be in 0..5"),
        ({"role": "invalid"}, "role must be 'idle', 'initiator', 'responder', or null"),
        ({"enabled": "yes"}, "enabled must be a boolean or null"),
    ],
)
def test_uwb_optional_overrides_are_validated(tmp_path, overrides, message):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "ble-node",
                        "/dev/ttyUSB0",
                        ble_enabled=True,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                        uwb_overrides=overrides,
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=message):
        VtolRuntimeConfiguration.from_file(path)


@pytest.mark.parametrize(
    ("devices", "message"),
    [
        (
            [
                d3k_device("first", "/dev/ttyUSB0", payloads=[{"payload_id": "payload-a", "peer_id": 20}]),
                d3k_device("second", "/dev/ttyUSB1", payloads=[{"payload_id": "payload-b", "peer_id": 21}]),
            ],
            "only one local D3K device",
        ),
        (
            [
                d3k_device("first", "/dev/ttyUSB0", ble_enabled=True),
                d3k_device("second", "/dev/ttyUSB1", payloads=[{"payload_id": "payload-a", "peer_id": 20}]),
            ],
            "only one local D3K device",
        ),
        (
            [
                d3k_device("first", "/dev/ttyUSB0", payloads=[{"payload_id": "payload-a", "peer_id": 20}]),
                d3k_device("second", "/dev/ttyUSB0"),
            ],
            "UART device paths must be unique",
        ),
    ],
)
def test_vtol_runtime_configuration_rejects_invalid_device_topology(tmp_path, devices, message):
    path = tmp_path / "vtol.json"
    path.write_text(json.dumps(vtol_document(devices)), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        VtolRuntimeConfiguration.from_file(path)


def test_vtol_runtime_configuration_rejects_duplicate_ble_peer_ids(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "ble-node",
                        "/dev/ttyUSB0",
                        ble_enabled=True,
                        payloads=[
                            {"payload_id": "payload-a", "peer_id": 20},
                            {"payload_id": "payload-b", "peer_id": 20},
                        ],
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="BLE peer IDs must be unique"):
        VtolRuntimeConfiguration.from_file(path)


def test_vtol_runtime_configuration_leaves_initiator_targets_to_board_when_omitted(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "initiator",
                        "/dev/ttyUSB0",
                        role="initiator",
                        ble_enabled=True,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                    )
                ]
            )
        ),
        encoding="utf-8",
    )

    # The target list may already be programmed on the board with d3km. The
    # application's partial config must not assume that an omitted list is empty.
    config = VtolRuntimeConfiguration.from_file(path)
    assert config.devices[0].uwb.target_ids is None
    assert config.devices[0].uwb.to_d3kble_desired() == {
        "ranging_enabled": True,
        "role": "initiator",
    }


def test_ble_section_is_optional_and_missing_enabled_preserves_firmware_default(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "front-left",
                        "/dev/ttyUSB0",
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                    ),
                    d3k_device("front-right", "/dev/ttyUSB1"),
                    d3k_device("back", "/dev/ttyUSB2"),
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)

    assert config.devices[0].ble_enabled is None
    assert config.devices[1].ble_enabled is None
    assert config.devices[2].ble_enabled is None
    assert config.ble_device.device_id == "front-left"
    assert [(peer.payload_id, peer.peer_id) for peer in config.payloads] == [("payload-a", 20)]


def test_vtol_runtime_configuration_allows_no_ble_configuration(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(vtol_document([d3k_device("front-left", "/dev/ttyUSB0"), d3k_device("back", "/dev/ttyUSB1")])),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)

    assert all(device.ble_enabled is None for device in config.devices)
    assert config.ble_device is None
    assert config.payloads == ()


def test_explicit_ble_off_is_valid_with_peer_mapping_and_keeps_host_selection(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(
            vtol_document(
                [
                    d3k_device(
                        "front-left",
                        "/dev/ttyUSB0",
                        ble_enabled=False,
                        payloads=[{"payload_id": "payload-a", "peer_id": 20}],
                    ),
                    d3k_device("front-right", "/dev/ttyUSB1"),
                ]
            )
        ),
        encoding="utf-8",
    )

    config = VtolRuntimeConfiguration.from_file(path)

    assert config.devices[0].ble_enabled is False
    assert config.ble_device.device_id == "front-left"
    assert config.payloads[0].payload_id == "payload-a"


def test_ble_enabled_must_be_boolean_or_null(tmp_path):
    path = tmp_path / "vtol.json"
    path.write_text(
        json.dumps(vtol_document([d3k_device("front-left", "/dev/ttyUSB0", ble_enabled="yes")])),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="ble.enabled must be a boolean or null"):
        VtolRuntimeConfiguration.from_file(path)
