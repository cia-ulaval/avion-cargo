from pathlib import Path
from typing import Tuple, Optional

import numpy as np
from abc import ABC, abstractmethod

class Camera(ABC):
    def __init__(self, resolution: Tuple[int, int], framerate: int, _format:str):
        self.source = None
        self.matrix:Optional[np.ndarray] = None
        self.distortion_matrix:Optional[np.ndarray] = None

    def load_calibration_settings(self, calibration_file_path:Path):
        pass

    @abstractmethod
    def open(self):
        raise NotImplementedError()

    @abstractmethod
    def close(self):
        raise NotImplementedError()

    def get_matrix(self) -> np.ndarray:
        return self.matrix

    def get_distortion_matrix(self) -> np.ndarray:
        return self.distortion_matrix

    @staticmethod
    def create_new_instance(resolution: Tuple[int, int], framerate: int, _format:str, calibration_file_path:Path) -> "Camera":
        new_camera = Camera(resolution, framerate, _format)
        new_camera.load_calibration_settings(calibration_file_path)
        pass

    def connect_to_camera(self, width, height, test_frame, timeout):
        raise NotImplementedError()

    def get_frame(self):
        raise NotImplementedError()