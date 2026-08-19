from pathlib import Path

import numpy as np

from domain.models import CalibrationReport
from infrastructure.persistence.calibration_repository import CalibrationRepository


def test_saved_npz_calibration_report_can_be_loaded_back_as_calibration_data(
    tmp_path: Path,
    monkeypatch,
    sample_calibration_report: CalibrationReport,
) -> None:
    monkeypatch.chdir(tmp_path)
    repository = CalibrationRepository()

    saved_path = repository.save_report(sample_calibration_report)
    calibration_data = CalibrationRepository().set_calibration_filepath(saved_path).load_calibration_data()

    assert saved_path.parent == Path("calibration_results")
    assert saved_path.suffix == ".npz"
    np.testing.assert_array_equal(calibration_data.camera_matrix, sample_calibration_report.camera_matrix)
    np.testing.assert_array_equal(calibration_data.dist_coeffs, sample_calibration_report.camera_distortion_matrix)
    assert calibration_data.camera_width == sample_calibration_report.image_width
    assert calibration_data.camera_height == sample_calibration_report.image_height
