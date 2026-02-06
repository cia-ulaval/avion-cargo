import argparse
from pathlib import Path

from domain.models import CalibrationData, LandingTarget
from infrastructure.camera.opencv_capture_adapter import OpenCVCaptureAdapter
from infrastructure.persistence.calibration_repo import CalibrationRepository
from infrastructure.vision.opencv_aruco_detector import (
    OpenCVArucoDetector,
    OpenCVArucoDetectorConfig,
)
from infrastructure.vision.opencv_gridboard_calibration_engine import (
    GridBoardCalibrationConfig,
    GridBoardSpec,
    OpenCVGridBoardCameraCalibrationEngine,
)
from ui.live_capture import LiveCaptureConfig, LiveFrameCollector


def build_camera(args):
    # webcam / video
    source = args.video if args.video else args.cam_id
    return OpenCVCaptureAdapter(
        source=source, width=args.width, height=args.height, fps=args.fps, rgb=False
    )


def main():
    ap = argparse.ArgumentParser(
        description="Live calibration with ArUco GridBoard (press 'c' to capture, ESC to calibrate)"
    )

    ap.add_argument("outfile", help="Output calibration file (npz)")
    ap.add_argument("-w", type=int, required=True, help="Markers X")
    ap.add_argument("-hm", dest="h_", type=int, required=True, help="Markers Y")
    ap.add_argument("-l", type=float, required=True, help="Marker length (meters)")
    ap.add_argument("-s", type=float, required=True, help="Marker separation (meters)")
    ap.add_argument("-d", type=int, default=2, help="Dictionary id (0..16)")

    ap.add_argument(
        "--rs", action="store_true", help="Refine strategy (during calibration)"
    )
    ap.add_argument("--zt", action="store_true", help="Zero tangential distortion")
    ap.add_argument("--pc", action="store_true", help="Fix principal point")
    ap.add_argument("-a", type=float, default=None, help="Fix aspect ratio fx/fy")

    ap.add_argument("--picam", action="store_true", help="Use PiCamera2 (Raspberry Pi)")
    ap.add_argument("--video", default="", help="Video file input (if empty -> camera)")
    ap.add_argument("--cam-id", type=int, default=0, help="Webcam id")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)

    ap.add_argument("--no-gui", action="store_true", help="Headless (no window)")
    ap.add_argument("--waitkey", type=int, default=10, help="cv2.waitKey ms")

    args = ap.parse_args()

    camera = build_camera(args)

    detector = OpenCVArucoDetector(
        OpenCVArucoDetectorConfig(dictionary_id=args.d, corner_refinement=True)
    )

    target = LandingTarget(marker_id=None, marker_length_m=args.l)

    collector = LiveFrameCollector(
        camera=camera,
        detector=detector,
        target=target,
        cfg=LiveCaptureConfig(
            window_name="out",
            waitkey_ms=args.waitkey,
            headless=args.no_gui,
        ),
    )
    frames = collector.collect()

    if len(frames) < 1:
        raise SystemExit("Not enough captures for calibration (0)")

    engine = OpenCVGridBoardCameraCalibrationEngine(
        board=GridBoardSpec(
            markers_x=args.w,
            markers_y=args.h_,
            marker_length_m=args.l,
            marker_separation_m=args.s,
            dictionary_id=args.d,
        ),
        cfg=GridBoardCalibrationConfig(
            refine_strategy=args.rs,
            fix_aspect_ratio=args.a,
            zero_tangent_dist=args.zt,
            fix_principal_point=args.pc,
        ),
    )

    calib = engine.calibrate_from_frames(frames)

    out = Path(args.outfile)
    CalibrationRepository().save_npz(
        out,
        CalibrationData(
            calib.camera_matrix, dist_coeffs=calib.camera_distortion_matrix
        ),
    )

    print(f"Calibration saved to {out}")


if __name__ == "__main__":
    main()
