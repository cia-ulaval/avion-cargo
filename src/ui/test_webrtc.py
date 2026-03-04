from pathlib import Path

import cv2

from application.tracking_service import TrackingService
from domain.camera import LastestFrameBuffer
from domain.models import TargetedMarker
from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
from infrastructure.communication.webrtc_content_diffuser import WebRTCConfig, WebRTCContentDiffuser
from infrastructure.vision.opencv_aruco_detector import OpenCVArucoDetectorConfig
from infrastructure.vision.processor.aruco_axis_adding_processor import ArucoAxisAddingProcessor
from infrastructure.vision.threaded_pipeline import ThreadedPipeline

buf = LastestFrameBuffer()
camera = OpenCVCamera()
frame_processor = ArucoAxisAddingProcessor()
target = TargetedMarker(marker_id=None, marker_length_m=0.181)
# calib_path = Path(
#     "/home/bertrand-awz/Documents/avionCargo/autolander/calibration_results/calibration_2026-02-07_21-45-24.npz"
# )
detector_config = OpenCVArucoDetectorConfig(dictionary_id=cv2.aruco.DICT_ARUCO_ORIGINAL)
# tracking_service = TrackingService.create(camera, target, calib_path, detector_config)


webrtc = WebRTCContentDiffuser(buf, WebRTCConfig(port=8080, stream_fps=30))
pipeline = ThreadedPipeline(
    camera=camera, frame_processor=camera.get_frame, frame_buffer=buf, com_channel=webrtc.diffuse_data
)
pipeline.start()

webrtc.diffuse_video()  # bloque, ctrl-c pour arrêter

pipeline.stop()
