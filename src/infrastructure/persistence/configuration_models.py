from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from domain.models import TargetedMarker


@dataclass(frozen=True, slots=True)
class CameraConfiguration:
    id: int
    use_picamera: bool
    fps: int
    calibration_filepath: Path
    simulation_topic_name: Optional[str] = None


@dataclass(frozen=True, slots=True)
class StreamingDataConfiguration:
    dps: int


@dataclass(frozen=True, slots=True)
class StreamingVideoConfiguration:
    fps: int


@dataclass(frozen=True, slots=True)
class StreamingConfiguration:
    port: int
    data: StreamingDataConfiguration
    video: StreamingVideoConfiguration


@dataclass(frozen=True, slots=True)
class DroneConnectionConfiguration:
    use_serial: bool
    address: str
    port: Optional[int]
    baud_rate: int


@dataclass(slots=True, frozen=True)
class AutolanderConfiguration:
    targeted_marker: TargetedMarker
    camera_config: CameraConfiguration
    streaming_config: StreamingConfiguration
    drone_connection_config: DroneConnectionConfiguration
