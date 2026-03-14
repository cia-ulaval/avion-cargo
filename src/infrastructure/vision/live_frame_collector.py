from dataclasses import dataclass
from typing import List

import cv2
import numpy as np

from domain.camera import Camera
from domain.frame_collector import FrameCollector
from domain.marker_detector import MarkerDetector
from domain.models import TargetedMarker
from infrastructure.vision.opencv_frame_manipution_tool import Color, FrameManipulationTool


@dataclass(slots=True)
class LiveFrameCollectorConfig:
    window_name: str = "calibration"
    waitkey_ms: int = 10
    show_overlays: bool = True
    headless: bool = False


class LiveFrameCollector(FrameCollector):
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
        target: TargetedMarker,
        cfg: LiveFrameCollectorConfig,
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
            vis = frame.copy() if self.cfg.show_overlays else frame

            if self.cfg.show_overlays and len(detections) > 0:
                corners_list = [c for (_mid, c) in detections]
                ids_arr = np.array([[mid] for (mid, _c) in detections], dtype=np.int32)
                FrameManipulationTool.draw_detected_markers(vis, corners_list, ids_arr)

            if self.cfg.show_overlays:
                msg = f"Captures: {len(frames)} | 'c' capture | ESC finish"
                FrameManipulationTool.write_text_on_frame(vis, msg, Color.BLUE)

            if not self.cfg.headless:
                cv2.imshow(self.cfg.window_name, vis)
                key = cv2.waitKey(self.cfg.waitkey_ms) & 0xFF
            else:
                key = 255

            if key == 27:
                break

            # capture
            if key == ord("c"):
                if len(detections) > 0:
                    print(f"[CAPTURE] frame {frame_i} ({len(detections)} markers)")
                    frames.append(frame.copy())
                else:
                    print("[SKIP] no markers detected")

        if not self.cfg.headless:
            cv2.destroyAllWindows()

        return frames
