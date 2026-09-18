from dataclasses import replace

import pytest

from domain.models import CalibrationReport
from ui.calibration_report_presenter import show_calibration_report


@pytest.mark.parametrize("aspect_ratio", [None, 1.0])
def test_report_presentation_preserves_calibration_values(
    capsys: pytest.CaptureFixture[str], sample_calibration_report: CalibrationReport, aspect_ratio: float | None
) -> None:
    report = replace(sample_calibration_report, aspect_ratio=aspect_ratio)

    show_calibration_report(report)

    output = capsys.readouterr().out
    assert "2026-01-02 03:04:05" in output
    assert "1280×720" in output
    assert "0.420000" in output
    assert "Aspect ratio" in output
    assert ("—" if aspect_ratio is None else "1.000000") in output
    assert "Camera matrix (K)" in output
    assert "620.000000" in output
    assert "Distortion coefficients (D)" in output
    assert "-0.02000000" in output
