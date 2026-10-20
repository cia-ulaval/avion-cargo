import re
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
from click.testing import CliRunner
from PIL import Image

from domain.models import SUPPORTED_FRACTAL_TYPES
from ui.marker_generation_cli import main


@pytest.mark.parametrize("dictionary_id", range(17))
def test_generated_aruco_png_is_detectable_in_each_supported_dictionary(tmp_path: Path, dictionary_id: int) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker_id = dictionary.bytesList.shape[0] - 1
    output = tmp_path / "nested" / "marker.png"

    result = CliRunner().invoke(
        main, ["aruco", "--id", str(marker_id), "-d", str(dictionary_id), "-l", "0.04", "-o", str(output)]
    )

    assert result.exit_code == 0, result.output
    image = cv2.imread(str(output), cv2.IMREAD_GRAYSCALE)
    _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    assert ids.flatten().tolist() == [marker_id]
    with Image.open(output) as png:
        assert png.info["dpi"] == pytest.approx((image.shape[0] / 0.06 * 0.0254,) * 2, abs=2)


def test_board_contains_calibration_ids_and_independent_separation_and_margin(tmp_path: Path) -> None:
    output = tmp_path / "board.png"
    result = CliRunner().invoke(
        main,
        [
            "board",
            "-w",
            "3",
            "-hm",
            "2",
            "-l",
            "0.0254",
            "-s",
            "0.0127",
            "--margin",
            "0.00508",
            "-d",
            "0",
            "-o",
            str(output),
        ],
    )

    assert result.exit_code == 0, result.output
    image = cv2.imread(str(output), cv2.IMREAD_GRAYSCALE)
    # 300 px markers, 150 px gaps, 60 px outer margins.
    assert image.shape == (870, 1320)
    assert np.all(image[:60] == 255)
    dictionary = cv2.aruco.getPredefinedDictionary(0)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    by_id = dict(zip(ids.flatten(), corners))
    assert sorted(by_id) == list(range(6))
    assert by_id[0][0, 0] == pytest.approx((60, 60), abs=1)
    assert by_id[1][0, 0] == pytest.approx((510, 60), abs=1)
    assert by_id[3][0, 0] == pytest.approx((60, 510), abs=1)


@pytest.mark.parametrize(
    ("command", "expected_size_m"),
    [
        (["aruco", "--id", "29", "-l", "0.2"], (0.22, 0.22)),
        (["board", "-w", "4", "-hm", "5", "-l", "0.03", "-s", "0.01"], (0.17, 0.21)),
    ],
)
def test_pdf_has_requested_physical_page_size(tmp_path: Path, command: list[str], expected_size_m: tuple) -> None:
    output = tmp_path / "marker.pdf"
    result = CliRunner().invoke(main, [*command, "-o", str(output)])

    assert result.exit_code == 0, result.output
    data = output.read_bytes()
    assert data.startswith(b"%PDF-")
    dimensions = re.search(rb"/MediaBox\s*\[\s*0\s+0\s+([\d.]+)\s+([\d.]+)\s*\]", data)
    assert dimensions is not None
    assert tuple(float(value) * 0.0254 / 72 for value in dimensions.groups()) == pytest.approx(expected_size_m)


@pytest.mark.parametrize("fractal_type", SUPPORTED_FRACTAL_TYPES)
@pytest.mark.parametrize("suffix", [".pdf", ".png"])
def test_faruco_uses_bridge_generation_and_removes_temporary_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, fractal_type: str, suffix: str
) -> None:
    from types import SimpleNamespace

    temporary_paths = []
    native_image = np.full((80, 80), 255, dtype=np.uint8)
    native_image[10:70, 10:70] = 0

    def generate_marker(*, fractal_type: str, output: Path, bit_size: int) -> Path:
        assert fractal_type == expected_type
        assert bit_size == 12
        temporary_paths.append(output)
        Image.fromarray(native_image).save(output)
        return output

    expected_type = fractal_type
    monkeypatch.setitem(sys.modules, "faruco_bridge", SimpleNamespace(generate_marker=generate_marker))
    output = tmp_path / f"marker{suffix}"
    result = CliRunner().invoke(
        main, ["faruco", "--fractal-type", fractal_type, "--bit-size", "12", "-l", "0.08", "-o", str(output)]
    )

    assert result.exit_code == 0, result.output
    assert output.is_file()
    assert len(temporary_paths) == 1
    assert not temporary_paths[0].parent.exists()
    if suffix == ".png":
        with Image.open(output) as image:
            # Native cells are retained, with only a white margin added.
            np.testing.assert_array_equal(np.asarray(image)[10:-10, 10:-10], native_image)
            assert image.info["dpi"] == pytest.approx((25.4, 25.4), abs=0.02)


def test_optional_bridge_is_needed_only_for_faruco(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "faruco_bridge", None)
    output = tmp_path / "marker.pdf"
    aruco = CliRunner().invoke(main, ["aruco", "--id", "0", "-l", "0.1", "-o", str(output)])
    assert aruco.exit_code == 0, aruco.output
    output.unlink()

    faruco = CliRunner().invoke(main, ["faruco", "-l", "0.1", "-o", str(output)])
    assert faruco.exit_code == 1
    assert "poetry install --with faruco" in faruco.output
    assert not output.exists()


@pytest.mark.parametrize(
    "arguments",
    [
        ["aruco", "--id", "50", "-d", "0"],
        ["aruco", "--id", "1024", "-d", "16"],
        ["aruco", "--id", "0", "-d", "17"],
        ["aruco", "--id", "0", "-d", "-1"],
        ["aruco", "--id", "0", "--dpi", "100"],
        ["aruco", "--id", "0", "--border-bits", "0"],
        ["aruco", "--id", "0", "--margin", "nan"],
        ["aruco", "--id", "0", "--margin", "-0.01"],
        ["aruco", "--id", "0", "-l", "nan"],
        ["aruco", "--id", "0", "-l", "inf"],
        ["aruco", "--id", "0", "-l", "0"],
        ["aruco", "--id", "0", "-l", "0.000001"],
        ["board", "-w", "10", "-hm", "6", "-d", "0", "-s", "0.01"],
        ["board", "-w", "2", "-hm", "2", "-s", "0"],
        ["board", "-w", "2", "-hm", "2", "-s", "nan"],
        ["faruco", "--fractal-type", "invalid"],
        ["faruco", "--bit-size", "1"],
        ["faruco", "--id", "42"],
    ],
)
def test_invalid_parameters_do_not_create_output(tmp_path: Path, arguments: list[str]) -> None:
    output = tmp_path / "marker.pdf"
    result = CliRunner().invoke(main, [arguments[0], "-l", "0.1", *arguments[1:], "-o", str(output)])
    assert result.exit_code != 0
    assert "Error:" in result.output
    assert not output.exists()


def test_rejects_unsupported_output_extension(tmp_path: Path) -> None:
    output = tmp_path / "marker.jpg"
    result = CliRunner().invoke(main, ["aruco", "--id", "0", "-l", "0.1", "-o", str(output)])
    assert result.exit_code == 2
    assert ".pdf or .png" in result.output
    assert not output.exists()


def test_lists_only_dictionaries_supported_by_current_configuration() -> None:
    result = CliRunner().invoke(main, ["dictionaries"])
    assert result.exit_code == 0, result.output
    assert "DICT_4X4_50" in result.output
    assert "DICT_ARUCO_ORIGINAL" in result.output
    assert "0..1023" in result.output
    assert "APRILTAG" not in result.output
