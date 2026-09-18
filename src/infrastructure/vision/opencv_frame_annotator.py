import numpy as np

from domain.frame_annotator import FrameAnnotator
from domain.models import CalibrationData
from infrastructure.vision.opencv_frame_manipution_tool import FrameManipulationTool


class OpenCVFrameAnnotator(FrameAnnotator):
    """Draw marker outlines and pose axes using OpenCV."""

    def annotate(
        self,
        frame: np.ndarray,
        *,
        marker_id: int,
        corners: np.ndarray,
        calibration: CalibrationData,
        rotation_vectors: np.ndarray,
        translation_vectors: np.ndarray,
    ) -> np.ndarray:
        frame = FrameManipulationTool.draw_detected_markers(frame, [corners], np.array([[marker_id]], dtype=np.int32))
        return FrameManipulationTool.draw_axes_for_poses(
            frame, calibration.camera_matrix, calibration.dist_coeffs, rotation_vectors, translation_vectors
        )
