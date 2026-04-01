from typing import Optional

from domain.camera import Camera
from domain.drone import Drone
from domain.models import CalibrationData
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.communication.mavlink import (
    DroneMavlinkSerialConnector,
    DroneMavlinkUDPConnector,
    MavlinkConnectionParams,
)
from infrastructure.persistence.configuration_models import CameraConfiguration, DroneConnectionConfiguration
from simulation.gazebo_camera import GazeboCamera


def build_camera(
    camera_config: CameraConfiguration, calibration_data: CalibrationData, use_simulated_cam: Optional[bool] = False
) -> Camera:
    if use_simulated_cam:
        if not camera_config.simulation_topic_name:
            raise ValueError("The simulation topic name must be provided")

        return GazeboCamera(camera_config.simulation_topic_name)

    if camera_config.use_picamera:
        from infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(
            width=calibration_data.camera_width, height=calibration_data.camera_height, fps=camera_config.fps
        )

    return OpenCVCamera(
        source=camera_config.id,
        width=calibration_data.camera_width,
        height=calibration_data.camera_height,
        fps=camera_config.fps,
    )


def build_drone(drone_connection_config: DroneConnectionConfiguration) -> Drone:
    mavlink_params = MavlinkConnectionParams(
        address=drone_connection_config.address,
        port=drone_connection_config.port,
        baud_rate=drone_connection_config.baud_rate,
    )
    if drone_connection_config.use_serial:
        return DroneMavlinkSerialConnector(mavlink_params)

    else:
        return DroneMavlinkUDPConnector(mavlink_params)
