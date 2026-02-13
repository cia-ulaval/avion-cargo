from abc import ABC, abstractmethod
from typing import Any, Dict


class ContentDiffuser(ABC):

    @abstractmethod
    def diffuse_video(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def diffuse_data(self, data: Dict[str, Any]) -> None:
        raise NotImplementedError()
