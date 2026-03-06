from domain.camera import Camera
from domain.drone import Drone
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.communication.drone_mavlink_connector import DroneMavlinkSerial, MavlinkConnectionParams, \
    DroneMavlinkUDP
from infrastructure.persistence.configuration_models import DroneConnectionConfiguration


def build_camera(*, picam: bool, cam_id: int, width: int, height: int, fps: int) -> Camera:
    if picam:
        from infrastructure.camera.picamera_adapter import PiCameraAdapter

        return PiCameraAdapter(width=width, height=height, fps=fps, rgb=False)

    return OpenCVCamera(source=cam_id, width=width, height=height, fps=fps, rgb=False)


def build_drone(drone_connection_config: DroneConnectionConfiguration) -> Drone:
    mavlink_params = MavlinkConnectionParams(
        address=drone_connection_config.address,
        port=drone_connection_config.port,
        baud_rate=drone_connection_config.baud_rate,
    )
    if drone_connection_config.use_serial:
        return DroneMavlinkSerial(
           mavlink_params,
        )
    else:
        return DroneMavlinkUDP(
            mavlink_params
        )
