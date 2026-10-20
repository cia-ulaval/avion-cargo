from abc import ABC, abstractmethod
from pathlib import Path


class MarkerGenerator(ABC):
    """Generate a printable marker or board at the requested destination."""

    @abstractmethod
    def generate(self, output: Path) -> Path:
        raise NotImplementedError
