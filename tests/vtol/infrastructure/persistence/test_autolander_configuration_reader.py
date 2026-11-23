from pathlib import Path

import pytest

from vtol.domain.markers import FractalType, MarkerType
from vtol.domain.models import TargetedMarker
from vtol.infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader
from vtol.infrastructure.persistence.configuration_models import (
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
    assert config.targeted_marker.marker_type is MarkerType.ARUCO
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
    assert config.drone_connection_config.timeout == 10.0
    assert config.drone_connection_config.fail_on_timeout is True


def test_reader_preserves_optional_fc_timeout_settings(tmp_path, valid_config_data):
    valid_config_data["drone_connection"].update(timeout=0.25, fail_on_timeout=False)

    config = AutolanderConfigurationReader(write_config(tmp_path, valid_config_data)).read()

    assert config.drone_connection_config.timeout == 0.25
    assert config.drone_connection_config.fail_on_timeout is False


def test_alignment_requires_explicit_configuration_and_preserves_marker_direction(tmp_path, valid_config_data):
    path = write_config(tmp_path, valid_config_data)
    assert not AutolanderConfigurationReader(path).read().yaw_alignment.enabled
    valid_config_data["yaw_alignment"] = {
        "enabled": True,
        "marker_forward_deg": 90,
        "trigger_deg": 15,
        "detection_stable_time_s": 2,
        "detection_loss_timeout_s": 3,
        "attempt_timeout_s": 40,
        "max_attempts": 5,
        "retry_descent_m": 2.5,
        "min_height_m": 4,
        "cooldown_s": 0.5,
    }
    config = AutolanderConfigurationReader(write_config(tmp_path, valid_config_data)).read()
    assert config.yaw_alignment.enabled
    assert config.yaw_alignment.marker_forward_deg == 90
    assert config.yaw_alignment.trigger_deg == 15
    assert config.yaw_alignment.detection_stable_time_s == 2
    assert config.yaw_alignment.detection_loss_timeout_s == 3
    assert config.yaw_alignment.attempt_timeout_s == 40
    assert config.yaw_alignment.max_attempts == 5
    assert config.yaw_alignment.retry_descent_m == 2.5
    assert config.yaw_alignment.min_height_m == 4
    assert config.yaw_alignment.cooldown_s == 0.5


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


def test_reader_preserves_configured_camera_mount(tmp_path, valid_config_data):
    valid_config_data["camera"]["mount"] = {
        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "translation_m": [0.1, 0.2, 0.3],
    }
    config = AutolanderConfigurationReader(write_config(tmp_path, valid_config_data)).read()
    from vtol.domain.models import Pose3D

    assert config.camera_config.mount.to_body(Pose3D(1, 2, 3)) == Pose3D(1.1, 2.2, 3.3)


@pytest.mark.parametrize("legacy_root_id", [False, True])
def test_reader_maps_faruco_configuration_to_automatic_tracking(
    tmp_path: Path, valid_config_data: dict, legacy_root_id: bool
) -> None:
    valid_config_data["vision"]["targeted_marker"] = {
        "type": "faruco",
        "length": 0.896,
        "fractal_type": "FRACTAL_3L_6",
    }
    if legacy_root_id:
        valid_config_data["vision"]["targeted_marker"]["fractal_id"] = 0
    config = AutolanderConfigurationReader(write_config(tmp_path, valid_config_data)).read()

    assert config.targeted_marker.marker_type is MarkerType.FARUCO
    assert config.targeted_marker.fractal_type is FractalType.FRACTAL_3L_6
    assert config.targeted_marker == TargetedMarker(
        id=None, length=0.896, marker_type="faruco", fractal_type="FRACTAL_3L_6"
    )


def test_reader_accepts_autolanding_section_inside_combined_vtol_configuration(tmp_path, valid_config_data):
    combined = {
        "autolanding": valid_config_data,
        "d3kble": {"devices": []},
    }
    config = AutolanderConfigurationReader(write_config(tmp_path, combined)).read()

    assert config.camera_config.id == valid_config_data["camera"]["id"]
    assert config.drone_connection_config.address == valid_config_data["drone_connection"]["address"]
