from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from domain.models import CalibrationData, CalibrationReport


class CalibrationRepository:
    def __init__(self):
        self.calibration_filepath: Optional[Path] = None
        self.default_calibration_filedir: Path = Path("calibration_results")
        self.default_calibration_file_extension = ".npz"
        self.calibration_filename = "calibration"
        self.saving_datetime_format = "%Y-%m-%d_%H-%M-%S"

    def save_report(self, calib: CalibrationReport) -> Path:
        self.default_calibration_filedir.mkdir(parents=True, exist_ok=True)
        file_created_datetime = datetime.now().strftime(self.saving_datetime_format)
        file_path = f"{self.calibration_filename}_{file_created_datetime}{self.default_calibration_file_extension}"
        file_path = self.default_calibration_filedir / file_path

        np.savez(
            file_path,
            camera_width=calib.image_width,
            camera_height=calib.image_height,
            camera_matrix=calib.camera_matrix,
            camera_distortion_matrix=calib.camera_distortion_matrix,
        )
        return file_path

    def set_calibration_filepath(self, calibration_filepath: Path) -> "CalibrationRepository":
        self.calibration_filepath = calibration_filepath
        return self

    def load_calibration_data(self) -> CalibrationData:
        return (
            self._load_calibration_data_from_npz_file(self.calibration_filepath)
            if self._is_npz_file(self.calibration_filepath)
            else self._load_calibration_data_from_yaml_file(self.calibration_filepath)
        )

    def _load_calibration_data_from_npz_file(self, file_path: Path) -> CalibrationData:
        self._require_existing_calibration_file()
        file_path = Path(file_path)
        data = np.load(file_path)
        return CalibrationData(
            camera_matrix=data["camera_matrix"],
            dist_coeffs=data["camera_distortion_matrix"],
            camera_height=data["height"],
            camera_width=data["width"],
        )

    def _load_calibration_data_from_yaml_file(self, file_path: Path) -> CalibrationData:
        self._require_existing_calibration_file()
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        return CalibrationData(
            camera_matrix=fs.getNode("camera_matrix").mat(),
            dist_coeffs=fs.getNode("dist_coeffs").mat(),
            camera_height=int(fs.getNode("resolution_height").real()),
            camera_width=int(fs.getNode("resolution_width").real()),
        )

    def _require_existing_calibration_file(self) -> None:
        if self.calibration_filepath is None or not self.calibration_filepath.exists():
            raise FileNotFoundError(f"File not found at {self.calibration_filepath}")

    def _is_npz_file(self, file_path: Path) -> bool:
        self._require_existing_calibration_file()
        return file_path.suffix == self.default_calibration_file_extension
