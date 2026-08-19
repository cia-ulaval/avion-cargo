import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from domain.drone import DroneMode, DroneStatus
from domain.models import CalibrationReport


@pytest.fixture
def valid_config_data(tmp_path: Path) -> dict[str, Any]:
    calibration_file = tmp_path / "camera_calibration.yaml"
    calibration_file.write_text("calibration: placeholder\n", encoding="utf-8")

    return {
        "camera": {
            "id": 0,
            "use_picamera": False,
            "fps": 25,
            "calibration_filepath": calibration_file.name,
            "gz_simulation": {
                "topic_name": "/world/test/model/drone/link/camera/sensor/image",
            },
        },
        "vision": {
            "targeted_marker": {
                "length": 0.896,
                "id": 29,
                "aruco_dictionary": 0,
            },
        },
        "streaming": {
            "port": 8085,
            "data": {
                "dps": 5,
            },
            "video": {
                "fps": 15,
            },
        },
        "drone_connection": {
            "use_serial": False,
            "address": "127.0.0.1",
            "port": 14550,
            "baud_rate": 921600,
        },
    }


def write_config(tmp_path: Path, data: dict[str, Any], filename: str = "autolander.json") -> Path:
    config_path = tmp_path / filename
    config_path.write_text(json.dumps(data), encoding="utf-8")
    return config_path


def changed_config(config: dict[str, Any], *path: str, value: Any) -> dict[str, Any]:
    copy = deepcopy(config)
    cursor = copy
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = value
    return copy


@pytest.fixture
def sample_calibration_report(
    *,
    image_width: int = 1280,
    image_height: int = 720,
) -> CalibrationReport:
    return CalibrationReport(
        calibration_date=datetime(2026, 1, 2, 3, 4, 5),
        image_width=image_width,
        image_height=image_height,
        camera_matrix=np.array(
            [
                [620.0, 0.0, image_width / 2],
                [0.0, 618.0, image_height / 2],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        ),
        camera_distortion_matrix=np.array([[0.1, -0.02, 0.003, 0.004, 0.0]], dtype=float),
        avg_reprojection_error=0.42,
        aspect_ratio=1.0,
    )


@pytest.fixture
def sample_drone_status() -> Any:
    def make_status(
        *,
        relative_altitude: float = 6.0,
        last_heartbeat_s: float = 10.0,
    ) -> DroneStatus:
        return DroneStatus(
            mode=DroneMode.LAND,
            alt_m=12.5,
            groundspeed_mps=1.2,
            battery_voltage_v=15.1,
            battery_remaining_pct=73,
            gps_fix_type=3,
            armed=True,
            last_heartbeat_s=last_heartbeat_s,
            last_signal_gpio_s=9.5,
            speed=1.0,
            relative_altitude=relative_altitude,
            latitude=46.78,
            longitude=-71.27,
            relative_altitude_ms=relative_altitude * 1000,
            heading_deg=181.0,
        )

    return make_status
