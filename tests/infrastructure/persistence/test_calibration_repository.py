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


def test_report_archive_preserves_quality_and_accepts_uppercase_extension(tmp_path, sample_calibration_report):
    repo = CalibrationRepository()
    repo.default_calibration_filedir = tmp_path
    path = repo.save_report(sample_calibration_report)
    with np.load(path, allow_pickle=False) as archive:
        assert str(archive['calibration_date']) == sample_calibration_report.calibration_date.isoformat()
        assert float(archive['avg_reprojection_error']) == sample_calibration_report.avg_reprojection_error
        assert float(archive['aspect_ratio']) == sample_calibration_report.aspect_ratio
    uppercase = path.with_suffix('.NPZ')
    path.rename(uppercase)
    loaded = repo.set_calibration_filepath(uppercase).load_calibration_data()
    np.testing.assert_array_equal(loaded.camera_matrix, sample_calibration_report.camera_matrix)
