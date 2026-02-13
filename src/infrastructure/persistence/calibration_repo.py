from datetime import datetime
from pathlib import Path

import numpy as np

from domain.models import CalibrationData, CalibrationReport


class CalibrationRepository:
    def __init__(self):
        self.default_calibration_filedir: Path = Path("calibration_results")
        self.calibration_file_extension = ".npz"
        self.calibration_filename = "calibration"
        self.saving_datetime_format = "%Y-%m-%d_%H-%M-%S"

    def save_report(self, calib: CalibrationReport) -> Path:
        self.default_calibration_filedir.mkdir(parents=True, exist_ok=True)
        file_created_datetime = datetime.now().strftime(self.saving_datetime_format)
        file_path = f"{self.calibration_filename}_{file_created_datetime}{self.calibration_file_extension}"
        file_path = self.default_calibration_filedir / file_path

        np.savez(file_path, camera_matrix=calib.camera_matrix, camera_distortion_matrix=calib.camera_distortion_matrix)
        return file_path

    def load_report(self, file_path: Path) -> CalibrationData:
        if file_path is None or not file_path.exists():
            raise FileNotFoundError(f"File not found at {file_path}")

        file_path = Path(file_path)
        data = np.load(file_path)
        return CalibrationData(camera_matrix=data["camera_matrix"], dist_coeffs=data["camera_distortion_matrix"])
