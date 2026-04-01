import json
from pathlib import Path
from typing import Any, Optional

from domain.models import TargetedMarker
from infrastructure.persistence.configuration_models import (
    AutolanderConfiguration,
    CameraConfiguration,
    DroneConnectionConfiguration,
    StreamingConfiguration,
    StreamingDataConfiguration,
    StreamingVideoConfiguration,
)
from utils.validators.configuration_validator import ConfigurationValidator


class AutolanderConfigurationReader:
    def __init__(self, config_path: Path) -> None:
        """
        :param config_path: Path to the configuration file
        :raises ValidationError
        """
        ConfigurationValidator(config_path).validate()
        self.config_path = config_path

    def read(self) -> AutolanderConfiguration:
        """
        Reads configuration from the configuration file (JSON only)
        :return: AutolanderConfiguration object
        """
        raw = self._read_json(self.config_path)

        camera_cfg = self._parse_camera(raw)
        targeted_marker = self._parse_targeted_marker(raw)
        streaming_cfg = self._parse_streaming(raw)
        drone_cfg = self._parse_drone_connection(raw)

        return AutolanderConfiguration(
            targeted_marker=targeted_marker,
            camera_config=camera_cfg,
            streaming_config=streaming_cfg,
            drone_connection_config=drone_cfg,
        )

    # --------- internals ---------

    def _read_json(self, path: Path) -> dict[str, Any]:
        if path.suffix.lower() != ".json":
            raise ValueError(f"Configuration file must be a .json file, got: {path.name}")

        try:
            text = path.read_text(encoding="utf-8")
            data = json.loads(text)
        except FileNotFoundError:
            raise
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in config file: {path}") from e

        if not isinstance(data, dict):
            raise ValueError("Root JSON must be an object/dict.")
        return data

    def _require_dict(self, obj: Any, path: str) -> dict[str, Any]:
        if not isinstance(obj, dict):
            raise ValueError(f"Expected object at '{path}', got {type(obj).__name__}")
        return obj

    def _require(self, d: dict[str, Any], key: str, path: str) -> Any:
        if key not in d:
            raise ValueError(f"Missing required key '{key}' at '{path}'")
        return d[key]

    def _parse_camera(self, raw: dict[str, Any]) -> CameraConfiguration:
        camera = self._require_dict(self._require(raw, "camera", "root"), "camera")

        camera_id = int(self._require(camera, "id", "camera"))
        use_picamera = bool(self._require(camera, "use_picamera", "camera"))
        fps = int(self._require(camera, "fps", "camera"))
        calibration_filepath = str(self._require(camera, "calibration_filepath", "camera"))
        gz_simulation = self._require(camera, "gz_simulation", "camera")


        return CameraConfiguration(
            id=camera_id,
            use_picamera=use_picamera,
            fps=fps,
            calibration_filepath=Path(calibration_filepath).resolve(),
            simulation_topic_name=gz_simulation.get("topic_name")
        )

    def _parse_targeted_marker(self, raw: dict[str, Any]) -> TargetedMarker:
        vision = self._require_dict(self._require(raw, "vision", "root"), "vision")
        tm = self._require_dict(self._require(vision, "targeted_marker", "vision"), "vision.targeted_marker")

        length = float(self._require(tm, "length", "vision.targeted_marker"))
        marker_id = int(self._require(tm, "id", "vision.targeted_marker"))
        aruco_dictionary = int(self._require(tm, "aruco_dictionary", "vision.targeted_marker"))

        return TargetedMarker(length=length, id=marker_id, dictionary=aruco_dictionary)

    def _parse_streaming(self, raw: dict[str, Any]) -> StreamingConfiguration:
        streaming = self._require_dict(self._require(raw, "streaming", "root"), "streaming")

        port = int(self._require(streaming, "port", "streaming"))

        data = self._require_dict(self._require(streaming, "data", "streaming"), "streaming.data")
        dps = int(self._require(data, "dps", "streaming.data"))

        video = self._require_dict(self._require(streaming, "video", "streaming"), "streaming.video")
        video_fps = int(self._require(video, "fps", "streaming.video"))

        return StreamingConfiguration(
            port=port,
            data=StreamingDataConfiguration(dps=dps),
            video=StreamingVideoConfiguration(fps=video_fps),
        )

    def _parse_drone_connection(self, raw: dict[str, Any]) -> DroneConnectionConfiguration:
        dc = self._require_dict(self._require(raw, "drone_connection", "root"), "drone_connection")

        use_serial = bool(self._require(dc, "use_serial", "drone_connection"))
        address = str(self._require(dc, "address", "drone_connection"))

        port_val = dc.get("port", None)
        port: Optional[int] = None if port_val is None else int(port_val)

        baud_rate = int(self._require(dc, "baud_rate", "drone_connection"))

        return DroneConnectionConfiguration(
            use_serial=use_serial,
            address=address,
            port=port,
            baud_rate=baud_rate,
        )
