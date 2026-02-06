from __future__ import annotations

from pathlib import Path

import numpy as np

from domain.models import CalibrationData


class CalibrationRepository:
    def save_npz(self, path: Path, calib: CalibrationData) -> None:
        path = Path(path)
        np.savez(path, camera_matrix=calib.camera_matrix, dist_coeffs=calib.dist_coeffs)

    def load_npz(self, path: Path) -> CalibrationData:
        path = Path(path)
        data = np.load(path)
        return CalibrationData(
            camera_matrix=data["camera_matrix"], dist_coeffs=data["dist_coeffs"]
        )
