from abc import ABC
from typing import List

import numpy as np


class FrameCollector(ABC):
    def collect(self) -> List[np.ndarray]:
        raise NotImplementedError()
