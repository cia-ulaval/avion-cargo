from abc import ABC, abstractmethod
from pathlib import Path


class MarkerGenerator(ABC):
    """
    Abstract class for marker generators.
    Generates marker or group of markers
    """

    @abstractmethod
    def generate(self) -> Path:
        raise NotImplementedError()
