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
