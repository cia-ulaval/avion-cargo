from typing import Tuple

import numpy as np

from domain.frame_processor import FrameProcessor


class ArucoAxisAddingProcessor(FrameProcessor):
    def __init__(self):
        pass

    def apply(self, frame: np.ndarray) -> Tuple[np.ndarray, dict]:
        return frame, {}