from pathlib import Path
from unittest.mock import Mock

import pytest
from click.testing import CliRunner

from domain.models import CalibrationReport
from ui import camera_calibration_cli

BOARD_OPTIONS = ["-w", "4", "-hm", "5", "-l", "0.03", "-s", "0.01"]


@pytest.mark.parametrize("dictionary_id", [None, *range(17)])
def test_cli_passes_the_same_dictionary_to_collection_and_calibration(
    monkeypatch: pytest.MonkeyPatch,
    sample_calibration_report: CalibrationReport,
    dictionary_id: int | None,
) -> None:
    build_camera = Mock()
    monkeypatch.setattr(camera_calibration_cli, "build_camera", build_camera)
    service = Mock()
    service.calibrate.return_value = (sample_calibration_report, Path("calibration.npz"))
    create_service = Mock(return_value=service)
    monkeypatch.setattr(camera_calibration_cli, "build_camera_calibration_service", create_service)
    args = BOARD_OPTIONS + ([] if dictionary_id is None else ["-d", str(dictionary_id)])

    result = CliRunner().invoke(camera_calibration_cli.main, args)

    assert result.exit_code == 0, result.output
    build_camera.assert_called_once()
    create_service.assert_called_once()
    camera, parameters = create_service.call_args.args
    assert camera is build_camera.return_value
    expected_id = 16 if dictionary_id is None else dictionary_id
    assert parameters.dictionary_id == expected_id
    assert parameters.board_specifications.dictionary_id == expected_id
    assert parameters.target.dictionary == expected_id
    service.calibrate.assert_called_once_with()


@pytest.mark.parametrize("dictionary_id", ["-1", "17", "1.5", "invalid"])
def test_cli_rejects_invalid_dictionary_before_building_camera(
    monkeypatch: pytest.MonkeyPatch, dictionary_id: str
) -> None:
    build_camera = Mock()
    monkeypatch.setattr(camera_calibration_cli, "build_camera", build_camera)

    result = CliRunner().invoke(camera_calibration_cli.main, BOARD_OPTIONS + ["-d", dictionary_id])

    assert result.exit_code == 2, result.output
    assert "Invalid value for '-d'" in result.output
    build_camera.assert_not_called()
