import json

import pytest

from d3k_diagnostics.main import DiagnosticsConfiguration


def test_loads_two_device_configuration(tmp_path):
    path = tmp_path / "diagnostics.json"
    path.write_text(
        json.dumps(
            {
                "devices": [
                    {"name": "plane", "uart_device": "/tmp/plane", "peer_id": 2},
                    {"name": "payload", "uart_device": "/tmp/payload", "peer_id": 1},
                ]
            }
        ),
        encoding="utf-8",
    )

    config = DiagnosticsConfiguration.from_file(path)

    assert [device.name for device in config.devices] == ["plane", "payload"]
    assert config.devices[0].baudrate == 460800
    assert config.devices[1].peer_id == 1


def test_rejects_duplicate_uart_paths(tmp_path):
    path = tmp_path / "diagnostics.json"
    path.write_text(
        json.dumps(
            {
                "devices": [
                    {"name": "plane", "uart_device": "/tmp/shared"},
                    {"name": "payload", "uart_device": "/tmp/shared"},
                ]
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="UART device paths must be unique"):
        DiagnosticsConfiguration.from_file(path)


def test_rejects_non_finite_poll_interval(tmp_path):
    path = tmp_path / "diagnostics.json"
    path.write_text(
        json.dumps(
            {
                "poll_interval_s": float("nan"),
                "devices": [{"name": "plane", "uart_device": "/tmp/plane"}],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="poll_interval_s must be finite and positive"):
        DiagnosticsConfiguration.from_file(path)
