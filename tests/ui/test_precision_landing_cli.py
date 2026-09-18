from unittest.mock import Mock, call

import pytest
from click.testing import CliRunner

from tests.conftest import write_config
from ui import precision_landing_cli


@pytest.mark.parametrize("simulation", [False, True])
def test_landing_cli_assembles_then_connects_and_starts_services(
    monkeypatch: pytest.MonkeyPatch, tmp_path, valid_config_data, simulation: bool
) -> None:
    path = write_config(tmp_path, valid_config_data)
    service = Mock()
    factory = Mock(return_value=service)
    monkeypatch.setattr(precision_landing_cli, "build_landing_service", factory)

    args = [str(path)] + (["--gz-simulation"] if simulation else [])
    result = CliRunner().invoke(precision_landing_cli.main, args)

    assert result.exit_code == 0, result.output
    factory.assert_called_once()
    assert factory.call_args.kwargs == {"use_simulated_cam": simulation}
    config = factory.call_args.args[0]
    assert config.camera_config.id == valid_config_data["camera"]["id"]
    assert config.targeted_marker.dictionary == valid_config_data["vision"]["targeted_marker"]["aruco_dictionary"]
    assert service.mock_calls == [
        call.drone.connect(),
        call.track_target(),
        call.stream_video(),
        call.perform_precision_landing(),
        call.stop(),
    ]


@pytest.mark.parametrize("stage", ["drone.connect", "track_target", "stream_video", "perform_precision_landing"])
def test_cli_failure_is_nonzero_and_always_closes_resources(monkeypatch, tmp_path, valid_config_data, stage):
    service = Mock()
    operation = service
    for name in stage.split("."):
        operation = getattr(operation, name)
    operation.side_effect = RuntimeError("injected failure")
    monkeypatch.setattr(precision_landing_cli, "build_landing_service", Mock(return_value=service))
    result = CliRunner().invoke(precision_landing_cli.main, [str(write_config(tmp_path, valid_config_data))])
    assert result.exit_code != 0
    service.stop.assert_called_once()


def test_cli_interrupt_closes_resources(monkeypatch, tmp_path, valid_config_data):
    service = Mock()
    service.perform_precision_landing.side_effect = KeyboardInterrupt
    monkeypatch.setattr(precision_landing_cli, "build_landing_service", Mock(return_value=service))
    result = CliRunner().invoke(precision_landing_cli.main, [str(write_config(tmp_path, valid_config_data))])
    assert result.exit_code != 0
    service.stop.assert_called_once()
