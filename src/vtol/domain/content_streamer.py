from abc import ABC, abstractmethod
from typing import Any, Dict


class ContentStreamer(ABC):

    @abstractmethod
    def stream_video(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def send_data(self, data: Dict[str, Any]) -> None:
        raise NotImplementedError()

    @abstractmethod
    def stop(self) -> None:
        """Request server shutdown without blocking the calling thread."""
        raise NotImplementedError()
