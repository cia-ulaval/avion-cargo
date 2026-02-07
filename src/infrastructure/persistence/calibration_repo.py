from pathlib import Path

import numpy as np

from domain.models import CalibrationData, CalibrationReport


class CalibrationRepository:
    def save_report(self, file_path: Path, calib: CalibrationReport) -> None:
        np.savez(file_path, camera_matrix=calib.camera_matrix, camera_distortion_matrix=calib.camera_distortion_matrix)

    def load_report(self, file_path: Path) -> CalibrationData:
        file_path = Path(file_path)
        data = np.load(file_path)
        return CalibrationData(camera_matrix=data["camera_matrix"], dist_coeffs=data["camera_distortion_matrix"])
