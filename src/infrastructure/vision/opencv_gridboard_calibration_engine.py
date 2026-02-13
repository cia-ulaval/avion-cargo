from dataclasses import dataclass
from datetime import datetime
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np

from domain.camera_calibration_engine import CameraCalibrationEngine
from domain.errors import InvalidCalibrationError
from domain.models import CalibrationReport

DICT_ID_TO_NAME = {
    0: "DICT_4X4_50",
    1: "DICT_4X4_100",
    2: "DICT_4X4_250",
    3: "DICT_4X4_1000",
    4: "DICT_5X5_50",
    5: "DICT_5X5_100",
    6: "DICT_5X5_250",
    7: "DICT_5X5_1000",
    8: "DICT_6X6_50",
    9: "DICT_6X6_100",
    10: "DICT_6X6_250",
    11: "DICT_6X6_1000",
    12: "DICT_7X7_50",
    13: "DICT_7X7_100",
    14: "DICT_7X7_250",
    15: "DICT_7X7_1000",
    16: "DICT_ARUCO_ORIGINAL",
}


def _make_detector_params() -> "cv2.aruco.DetectorParameters":
    p = cv2.aruco.DetectorParameters()
    # Paramètres "tolérants" (bons pour calibration)
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 45
    p.adaptiveThreshWinSizeStep = 6
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.minCornerDistanceRate = 0.01
    p.minMarkerDistanceRate = 0.02
    p.polygonalApproxAccuracyRate = 0.05
    return p


@dataclass(frozen=True, slots=True)
class GridBoardSpec:
    markers_x: int
    markers_y: int
    marker_length_m: float
    marker_separation_m: float
    dictionary_id: int


@dataclass(frozen=True, slots=True)
class GridBoardCalibrationConfig:
    refine_strategy: bool = False
    fix_aspect_ratio: Optional[float] = None
    zero_tangent_dist: bool = False
    fix_principal_point: bool = False


class NotEnoughFramesError(InvalidCalibrationError):
    def __init__(self) -> None:
        super().__init__(
            "Not enough usable frames for calibration.",
        )


class OpenCVGridBoardCameraCalibrationEngine(CameraCalibrationEngine):
    """
    OpenCV-based calibration engine using ArUco GridBoard.
    Refactored from cam_calibration_gridboard.py, without CLI and without PiCamera2.
    """

    def __init__(
        self,
        board: GridBoardSpec,
        cfg: GridBoardCalibrationConfig = GridBoardCalibrationConfig(),
    ) -> None:
        self._board_spec = board
        self._cfg = cfg

        if board.dictionary_id not in DICT_ID_TO_NAME:
            raise ValueError("dictionary_id must be in 0..16")

        dict_name = DICT_ID_TO_NAME[board.dictionary_id]
        if not hasattr(cv2.aruco, dict_name):
            raise RuntimeError(f"OpenCV does not provide {dict_name}")

        self._dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self._board = cv2.aruco.GridBoard(
            (board.markers_x, board.markers_y),
            board.marker_length_m,
            board.marker_separation_m,
            self._dictionary,
        )
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, _make_detector_params())

    def _flags_and_initial_k(self) -> Tuple[int, float, Optional[np.ndarray]]:
        flags = 0
        aspect_ratio = 1.0

        if self._cfg.fix_aspect_ratio is not None:
            flags |= cv2.CALIB_FIX_ASPECT_RATIO
            aspect_ratio = float(self._cfg.fix_aspect_ratio)

        if self._cfg.zero_tangent_dist:
            flags |= cv2.CALIB_ZERO_TANGENT_DIST

        if self._cfg.fix_principal_point:
            flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

        camera_matrix = None
        if flags & cv2.CALIB_FIX_ASPECT_RATIO:
            camera_matrix = np.eye(3, dtype=np.float64)
            camera_matrix[0, 0] = aspect_ratio

        return flags, aspect_ratio, camera_matrix

    def calibrate_from_frames(self, frames: Iterable[np.ndarray]) -> CalibrationReport:
        all_corners_per_frame: list[list[np.ndarray]] = []
        all_ids_per_frame: list[np.ndarray] = []
        img_size: Optional[tuple[int, int]] = None  # (w,h)

        for frame in frames:
            if frame is None:
                continue

            # frame expected BGR. If you standardize RGB elsewhere, convert before calling this engine.
            h, w = frame.shape[:2]
            img_size = (w, h)

            corners, ids, rejected = self._detector.detectMarkers(frame)

            if self._cfg.refine_strategy and ids is not None and len(ids) > 0:
                corners, ids, rejected, _ = cv2.aruco.refineDetectedMarkers(
                    image=frame,
                    board=self._board,
                    detectedCorners=corners,
                    detectedIds=ids,
                    rejectedCorners=rejected,
                )

            if ids is None or len(ids) == 0:
                continue

            all_corners_per_frame.append(corners)
            all_ids_per_frame.append(ids.copy())

        if img_size is None or len(all_ids_per_frame) < 1:
            raise NotEnoughFramesError()

        all_corners_concat: list[np.ndarray] = []
        all_ids_concat: list[int] = []
        marker_counter_per_frame: list[int] = []

        for corners_i, ids_i in zip(all_corners_per_frame, all_ids_per_frame):
            marker_counter_per_frame.append(len(corners_i))
            for c, mid in zip(corners_i, ids_i.flatten().tolist()):
                all_corners_concat.append(c)
                all_ids_concat.append(int(mid))

        flags, _aspect_ratio, camera_matrix_init = self._flags_and_initial_k()
        dist_coeffs_init = None

        all_ids_concat_np = np.array(all_ids_concat, dtype=np.int32).reshape(-1, 1)
        counter_np = np.array(marker_counter_per_frame, dtype=np.int32).reshape(-1, 1)

        rep_error, camera_matrix, dist_coeffs, *_ = cv2.aruco.calibrateCameraAruco(
            corners=all_corners_concat,
            ids=all_ids_concat_np,
            counter=counter_np,
            board=self._board,
            imageSize=img_size,
            cameraMatrix=camera_matrix_init,
            distCoeffs=dist_coeffs_init,
            flags=flags,
        )

        img_width, img_height = img_size

        return CalibrationReport(
            calibration_date=datetime.now(),
            image_width=img_width,
            image_height=img_height,
            camera_matrix=camera_matrix,
            camera_distortion_matrix=dist_coeffs,
            avg_reprojection_error=rep_error,
            aspect_ratio=(float(self._cfg.fix_aspect_ratio) if self._cfg.fix_aspect_ratio is not None else None),
        )
