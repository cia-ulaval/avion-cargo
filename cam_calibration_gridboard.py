import argparse
import time
from datetime import datetime

import cv2
import numpy as np

from picamera2 import Picamera2


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


def make_detector_params() -> "cv2.aruco.DetectorParameters":
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 45
    p.adaptiveThreshWinSizeStep = 6
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.minCornerDistanceRate = 0.01
    p.minMarkerDistanceRate = 0.02
    p.polygonalApproxAccuracyRate = 0.05
    return p


def save_camera_params_yaml(path: str, image_size, flags: int, aspect_ratio: float,
                           camera_matrix: np.ndarray, dist_coeffs: np.ndarray, avg_reproj_err: float):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Impossible d'écrire: {path}")

    fs.write("calibration_time", datetime.now().strftime("%a %d %b %Y %I:%M:%S %p %Z"))
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))

    if flags & cv2.CALIB_FIX_ASPECT_RATIO:
        fs.write("aspectRatio", float(aspect_ratio))

    fs.write("flags", int(flags))
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.write("avg_reprojection_error", float(avg_reproj_err))
    fs.release()


def open_picamera2(width: int, height: int, fps: int) -> Picamera2:
    """
    Configure Picamera2 pour fournir des frames RGB888 (stable pour OpenCV).
    """
    picam2 = Picamera2()

    #RGB888 = facile à convertir en BGR pour OpenCV.
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": float(fps)}
    )
    picam2.configure(config)
    picam2.start()
    # Petite pause pour laisser l'auto-expo/awb se stabiliser
    time.sleep(0.2)
    return picam2


def main():
    ap = argparse.ArgumentParser(
        description="Calibration caméra avec ArUco GridBoard (PiCamera2/libcamera)."
    )

    ap.add_argument("outfile", help="Fichier de sortie (ex: calibration.yml)")

    ap.add_argument("-w", type=int, required=True, help="Nombre de marqueurs en X (colonnes)")
    ap.add_argument("-hm", dest="h_", type=int, required=True, help="Nombre de marqueurs en Y (lignes)")
    ap.add_argument("-l", type=float, required=True, help="Taille du marqueur (mètres)")
    ap.add_argument("-s", type=float, required=True, help="Séparation entre marqueurs (mètres)")
    ap.add_argument("-d", type=int, required=True, help="Dictionnaire (0..16)")


    ap.add_argument("--waitkey", type=int, default=10, help="waitKey ms")
    ap.add_argument("--rs", action="store_true", help="Apply refine strategy")
    ap.add_argument("--zt", action="store_true", help="Zero tangential distortion")
    ap.add_argument("--pc", action="store_true", help="Fix principal point at center")
    ap.add_argument("-a", type=float, default=None, help="Fix aspect ratio fx/fy")

    ap.add_argument("--no-gui", action="store_true", help="Pas de fenêtre (headless)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)

    args = ap.parse_args()

    if args.d not in DICT_ID_TO_NAME:
        raise SystemExit("d doit être dans 0..16")

    dict_name = DICT_ID_TO_NAME[args.d]
    if not hasattr(cv2.aruco, dict_name):
        raise SystemExit(f"OpenCV ne fournit pas {dict_name}")

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    board = cv2.aruco.GridBoard((args.w, args.h_), args.l, args.s, dictionary)

    detector = cv2.aruco.ArucoDetector(dictionary, make_detector_params())

    # calibration flags
    calibration_flags = 0
    aspect_ratio = 1.0

    if args.a is not None:
        calibration_flags |= cv2.CALIB_FIX_ASPECT_RATIO
        aspect_ratio = float(args.a)
    if args.zt:
        calibration_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if args.pc:
        calibration_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

    picam2 = open_picamera2(args.width, args.height, args.fps)

    all_corners = []
    all_ids = []
    img_size = (args.width, args.height)  # (w,h)

    print("Calibration using an ArUco GridBoard (PiCamera2)")
    print("  Press 'c' to capture a frame with detected markers")
    print("  Press 'ESC' to finish capturing and calibrate")

    frame_i = 0
    try:
        while True:
            # Picamera2 renvoie RGB; OpenCV travaille généralement en BGR
            rgb = picam2.capture_array()
            if rgb is None:
                time.sleep(0.01)
                continue

            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            frame_i += 1

            corners, ids, rejected = detector.detectMarkers(frame)

            if args.rs and ids is not None and len(ids) > 0:
                corners, ids, rejected, _recovered = cv2.aruco.refineDetectedMarkers(
                    image=frame,
                    board=board,
                    detectedCorners=corners,
                    detectedIds=ids,
                    rejectedCorners=rejected
                )

            vis = frame.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            cv2.putText(
                vis,
                "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )

            if not args.no_gui:
                cv2.imshow("out", vis)
                key = cv2.waitKey(args.waitkey) & 0xFF
            else:
                key = 255

            if key == 27:  # ESC
                break

            if key == ord("c"):
                if ids is not None and len(ids) > 0:
                    print(f"[CAPTURE] frame {frame_i} with {len(ids)} markers")
                    all_corners.append(corners)
                    all_ids.append(ids.copy())
                else:
                    print("[SKIP] no markers detected on this frame")

    finally:
        picam2.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()

    if len(all_ids) < 1:
        raise SystemExit("Not enough captures for calibration (0)")

    all_corners_concat = []
    all_ids_concat = []
    marker_counter_per_frame = []

    for corners_i, ids_i in zip(all_corners, all_ids):
        marker_counter_per_frame.append(len(corners_i))
        for c, mid in zip(corners_i, ids_i.flatten().tolist()):
            all_corners_concat.append(c)
            all_ids_concat.append(mid)

    all_ids_concat = np.array(all_ids_concat, dtype=np.int32)

    camera_matrix = None
    dist_coeffs = None
    if calibration_flags & cv2.CALIB_FIX_ASPECT_RATIO:
        camera_matrix = np.eye(3, dtype=np.float64)
        camera_matrix[0, 0] = aspect_ratio

    if hasattr(cv2.aruco, "calibrateCameraAruco"):
        rep_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraAruco(
            corners=all_corners_concat,
            ids=all_ids_concat,
            counter=marker_counter_per_frame,
            board=board,
            imageSize=img_size,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=calibration_flags
        )
    else:
        if not hasattr(board, "matchImagePoints"):
            raise RuntimeError(
                "OpenCV aruco incomplet: ni calibrateCameraAruco ni board.matchImagePoints."
            )
        objpoints, imgpoints = [], []
        for corners_i, ids_i in zip(all_corners, all_ids):
            obj, img = board.matchImagePoints(corners_i, ids_i)
            if obj is None or img is None or len(obj) < 4:
                continue
            objpoints.append(obj)
            imgpoints.append(img)

        if len(objpoints) < 3:
            raise RuntimeError("Pas assez de frames exploitables pour calibrer (fallback).")

        rep_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=objpoints,
            imagePoints=imgpoints,
            imageSize=img_size,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=calibration_flags
        )

    save_camera_params_yaml(
        args.outfile,
        image_size=img_size,
        flags=calibration_flags,
        aspect_ratio=aspect_ratio,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        avg_reproj_err=rep_error
    )

    print(f"Rep Error: {rep_error}")
    print(f"Calibration saved to {args.outfile}")


if __name__ == "__main__":
    main()
