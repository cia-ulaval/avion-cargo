from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

import cv2
import numpy as np


class Color(Enum):
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    CYAN = (255, 255, 0)
    MAGENTA = (255, 0, 255)
    YELLOW = (0, 255, 255)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)

    def bgr(self) -> tuple[int, int, int]:
        b, g, r = self.value
        return int(b), int(g), int(r)


@dataclass(frozen=True, slots=True)
class TextStyle:
    org: tuple[int, int] = (10, 22)
    font_face: int = cv2.FONT_HERSHEY_TRIPLEX
    font_scale: float = 0.5
    thickness: int = 1
    line_type: int = cv2.LINE_AA


@dataclass(frozen=True, slots=True)
class AxisStyle:
    axis_length_m: float = 0.05
    thickness: int = 2


@dataclass(frozen=True)
class FrameManipulationTool:

    @staticmethod
    def write_text_on_frame(
        frame: np.ndarray,
        text: str,
        color: Color = Color.BLUE,
        style: TextStyle = TextStyle(),
        copy: bool = False,
    ) -> np.ndarray:
        """
        Draw text on a frame. Returns the (possibly copied) frame.
        """
        out = frame.copy() if copy else frame
        cv2.putText(
            out,
            text,
            style.org,
            style.font_face,
            style.font_scale,
            color.bgr(),
            style.thickness,
            style.line_type,
        )
        return out

    @staticmethod
    def draw_detected_markers(
        frame: np.ndarray,
        corners: Sequence[np.ndarray],
        ids: Optional[np.ndarray],
        copy: bool = False,
    ) -> np.ndarray:
        """
        Draw ArUco detected markers on frame.
        Note: cv2.aruco.drawDetectedMarkers ignores custom color in many OpenCV builds.
        """
        out = frame.copy() if copy else frame
        if len(corners) == 0:
            return out
        cv2.aruco.drawDetectedMarkers(out, list(corners), ids)
        return out

    @staticmethod
    def estimate_pose_single_markers(
        corners: Sequence[np.ndarray],
        marker_length_m: float,
        camera_matrix: np.ndarray,
        distortion_coefficients: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns (rotation_vectors, translation_vectors) with shape (N, 1, 3).
        """
        rotation_vectors, translation_vectors, _ = cv2.aruco.estimatePoseSingleMarkers(
            list(corners),
            float(marker_length_m),
            camera_matrix,
            distortion_coefficients,
        )
        return rotation_vectors, translation_vectors

    @staticmethod
    def draw_axes_for_poses(
        frame: np.ndarray,
        camera_matrix: np.ndarray,
        distortion_coefficients: np.ndarray,
        rotation_vectors: np.ndarray,
        translation_vectors: np.ndarray,
        axis_style: AxisStyle = AxisStyle(),
        copy: bool = False,
    ) -> np.ndarray:
        """
        Draw 3D axes for already-estimated poses.
        """
        out = frame.copy() if copy else frame

        n = min(len(rotation_vectors), len(translation_vectors))
        for i in range(n):
            cv2.drawFrameAxes(
                out,
                camera_matrix,
                distortion_coefficients,
                rotation_vectors[i],
                translation_vectors[i],
                axis_style.axis_length_m,
                axis_style.thickness,
            )
        return out
