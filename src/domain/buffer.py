from abc import ABC, abstractmethod
from typing import Any


class Buffer(ABC):
    @abstractmethod
    def set_value(self, value: Any):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self) -> Any:
        raise NotImplementedError()
