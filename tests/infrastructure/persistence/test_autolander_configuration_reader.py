from pathlib import Path

from domain.models import TargetedMarker
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from infrastructure.persistence.configuration_models import (
    DroneConnectionConfiguration,
    StreamingDataConfiguration,
    StreamingVideoConfiguration,
)
from tests.conftest import write_config


def test_reader_maps_a_valid_json_file_to_runtime_configuration_objects(
    tmp_path: Path,
    valid_config_data: dict,
) -> None:
    config_path = write_config(tmp_path, valid_config_data)

    config = AutolanderConfigurationReader(config_path).read()

    assert config.targeted_marker == TargetedMarker(id=29, length=0.896, dictionary=0)
    assert config.camera_config.id == 0
    assert config.camera_config.use_picamera is False
    assert config.camera_config.fps == 25
    assert config.camera_config.calibration_filepath == (tmp_path / "camera_calibration.yaml").resolve()
    assert config.camera_config.simulation_topic_name == "/world/test/model/drone/link/camera/sensor/image"
    assert config.streaming_config.port == 8085
    assert config.streaming_config.data == StreamingDataConfiguration(dps=5)
    assert config.streaming_config.video == StreamingVideoConfiguration(fps=15)
    assert config.drone_connection_config == DroneConnectionConfiguration(
        use_serial=False,
        address="127.0.0.1",
        port=14550,
        baud_rate=921600,
    )


def test_reader_resolves_relative_calibration_paths_from_the_config_file_directory(
    tmp_path: Path,
    valid_config_data: dict,
    monkeypatch,
) -> None:
    project_elsewhere = tmp_path / "other-working-dir"
    project_elsewhere.mkdir()
    config_dir = tmp_path / "configs"
    config_dir.mkdir()
    calibration_file = config_dir / "relative_camera.yaml"
    calibration_file.write_text("calibration: placeholder\n", encoding="utf-8")
    valid_config_data["camera"]["calibration_filepath"] = calibration_file.name
    config_path = write_config(config_dir, valid_config_data)
    monkeypatch.chdir(project_elsewhere)

    config = AutolanderConfigurationReader(config_path).read()

    assert config.camera_config.calibration_filepath == calibration_file.resolve()
