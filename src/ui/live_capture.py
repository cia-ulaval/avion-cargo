from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from domain.camera import Camera
from domain.marker_detector import MarkerDetector
from domain.models import LandingTarget


@dataclass(slots=True)
class LiveCaptureConfig:
    window_name: str = "calibration"
    waitkey_ms: int = 10
    show_overlays: bool = True
    headless: bool = False


class LiveFrameCollector:
    """
    UI helper:
      - Shows live video
      - draws detected markers (optional)
      - Press 'c' to capture a frame
      - Press ESC to finish

    Returns captured frames (np.ndarray).
    """

    def __init__(
        self,
        camera: Camera,
        detector: MarkerDetector,
        target: LandingTarget,
        cfg: LiveCaptureConfig,
    ):
        self.camera = camera
        self.detector = detector
        self.target = target
        self.cfg = cfg

    def collect(self) -> List[np.ndarray]:
        self.camera.open()

        frames: List[np.ndarray] = []
        frame_i = 0

        print("Live calibration capture")
        print("  Press 'c' to capture current frame (if markers detected)")
        print("  Press 'ESC' to finish and calibrate")

        while True:
            frame = self.camera.get_frame()
            frame_i += 1

            detections = self.detector.detect(frame, self.target)

            # For visualization: draw markers if we can (OpenCV expects corners+ids)
            vis = frame.copy() if self.cfg.show_overlays else frame
            if self.cfg.show_overlays and len(detections) > 0:
                # convert to OpenCV draw format: corners list + ids array
                corners_list = [c for (_mid, c) in detections]
                ids_arr = np.array([[mid] for (mid, _c) in detections], dtype=np.int32)
                cv2.aruco.drawDetectedMarkers(vis, corners_list, ids_arr)

            if self.cfg.show_overlays:
                msg = f"Captures: {len(frames)} | 'c' capture | ESC finish"
                cv2.putText(
                    vis, msg, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
                )

            if not self.cfg.headless:
                cv2.imshow(self.cfg.window_name, vis)
                key = cv2.waitKey(self.cfg.waitkey_ms) & 0xFF
            else:
                key = 255

            # ESC
            if key == 27:
                break

            # capture
            if key == ord("c"):
                if len(detections) > 0:
                    print(f"[CAPTURE] frame {frame_i} ({len(detections)} markers)")
                    frames.append(frame.copy())  # copy to freeze it (safe)
                else:
                    print("[SKIP] no markers detected")

        if not self.cfg.headless:
            cv2.destroyAllWindows()

        return frames
