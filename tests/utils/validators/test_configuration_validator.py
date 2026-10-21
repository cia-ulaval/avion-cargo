from pathlib import Path

import pytest

from tests.conftest import changed_config, write_config
from utils.validators.configuration_validator import ConfigurationValidator
from utils.validators.validation_error import ValidationError


def test_validator_accepts_a_coherent_udp_configuration_with_relative_calibration_path(
    tmp_path: Path,
    valid_config_data: dict,
) -> None:
    config_path = write_config(tmp_path, valid_config_data)

    ConfigurationValidator(config_path).validate()


def test_validator_rejects_serial_configuration_that_still_uses_a_network_port(
    tmp_path: Path,
    valid_config_data: dict,
) -> None:
    data = changed_config(valid_config_data, "drone_connection", "use_serial", value=True)
    data["drone_connection"]["address"] = "/dev/serial0"
    data["drone_connection"]["port"] = 14550
    config_path = write_config(tmp_path, data)

    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(config_path).validate()

    assert error.value.path == "drone_connection.port"
    assert "use_serial=true" in error.value.message


@pytest.mark.parametrize(
    ("path", "value", "expected_error_path"),
    [
        (("camera", "fps"), True, "camera.fps"),
        (("streaming", "data", "dps"), False, "streaming.data.dps"),
        (("vision", "targeted_marker", "id"), True, "vision.targeted_marker.id"),
    ],
)
def test_validator_rejects_boolean_values_where_configuration_requires_integers(
    tmp_path: Path,
    valid_config_data: dict,
    path: tuple[str, ...],
    value: bool,
    expected_error_path: str,
) -> None:
    config_path = write_config(tmp_path, changed_config(valid_config_data, *path, value=value))

    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(config_path).validate()

    assert error.value.path == expected_error_path
    assert "integer" in error.value.message


def test_validator_rejects_unknown_nested_keys_before_they_reach_runtime_parsing(
    tmp_path: Path,
    valid_config_data: dict,
) -> None:
    valid_config_data["streaming"]["video"]["bitrate"] = 8_000
    config_path = write_config(tmp_path, valid_config_data)

    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(config_path).validate()

    assert error.value.path == "streaming.video"
    assert "Unknown keys" in error.value.message


@pytest.mark.parametrize("dictionary_id", [-1, 17])
def test_validator_rejects_aruco_dictionaries_outside_supported_opencv_range(
    tmp_path: Path,
    valid_config_data: dict,
    dictionary_id: int,
) -> None:
    valid_config_data["vision"]["targeted_marker"]["aruco_dictionary"] = dictionary_id
    config_path = write_config(tmp_path, valid_config_data)

    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(config_path).validate()

    assert error.value.path == "vision.targeted_marker.aruco_dictionary"


def test_real_camera_does_not_require_a_simulation_topic(tmp_path, valid_config_data):
    del valid_config_data["camera"]["gz_simulation"]
    ConfigurationValidator(write_config(tmp_path, valid_config_data)).validate()


@pytest.mark.parametrize(
    "path,value",
    [
        (("camera", "calibration_filepath"), ""),
        (("vision", "targeted_marker", "length"), float("nan")),
        (("vision", "targeted_marker", "length"), float("inf")),
        (("vision", "targeted_marker", "id"), 50),
        (("camera", "gz_simulation"), {"topic_name": "/image", "typo": True}),
    ],
)
def test_invalid_configuration_fails_before_startup(tmp_path, valid_config_data, path, value):
    with pytest.raises(ValidationError):
        ConfigurationValidator(write_config(tmp_path, changed_config(valid_config_data, *path, value=value))).validate()


@pytest.mark.parametrize("legacy_root_id", [False, True])
def test_validator_accepts_faruco_configuration(tmp_path: Path, valid_config_data: dict, legacy_root_id: bool) -> None:
    valid_config_data["vision"]["targeted_marker"] = {
        "type": "faruco",
        "length": 0.896,
        "fractal_type": "FRACTAL_3L_6",
    }
    if legacy_root_id:
        valid_config_data["vision"]["targeted_marker"]["fractal_id"] = 0

    ConfigurationValidator(write_config(tmp_path, valid_config_data)).validate()


def test_validator_rejects_unknown_faruco_preset(tmp_path: Path, valid_config_data: dict) -> None:
    valid_config_data["vision"]["targeted_marker"] = {
        "type": "faruco",
        "length": 0.896,
        "fractal_type": "FRACTAL_99L_99",
    }

    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(write_config(tmp_path, valid_config_data)).validate()

    assert error.value.path == "vision.targeted_marker.fractal_type"


@pytest.mark.parametrize("legacy_id", [True, None, "0", -1, 1])
def test_validator_rejects_invalid_legacy_faruco_id(tmp_path: Path, valid_config_data: dict, legacy_id) -> None:
    valid_config_data["vision"]["targeted_marker"] = {
        "type": "faruco",
        "length": 0.896,
        "fractal_type": "FRACTAL_3L_6",
        "fractal_id": legacy_id,
    }
    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(write_config(tmp_path, valid_config_data)).validate()
    assert error.value.path == "vision.targeted_marker.fractal_id"


def test_validator_still_requires_aruco_id(tmp_path: Path, valid_config_data: dict) -> None:
    del valid_config_data["vision"]["targeted_marker"]["id"]
    with pytest.raises(ValidationError) as error:
        ConfigurationValidator(write_config(tmp_path, valid_config_data)).validate()
    assert error.value.path == "vision.targeted_marker.id"
