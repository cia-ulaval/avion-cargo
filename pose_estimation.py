import argparse
import time
from pathlib import Path

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


def load_calibration(yml_path: str):
    """
    Voir possibilité de conserver le fichier de calibration en format binaire numpy(npy) à la place de YAML
    """
    fs = cv2.FileStorage(yml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Impossible d'ouvrir le fichier de calibration: {yml_path}")
    K = fs.getNode("camera_matrix").mat()
    D = fs.getNode("distortion_coefficients").mat()
    fs.release()

    if K is None or D is None:
        raise RuntimeError("camera_matrix ou distortion_coefficients introuvable dans le YAML")

    K = np.array(K, dtype=np.float64)
    D = np.array(D, dtype=np.float64)
    return K, D


def make_detector_params():
    p = cv2.aruco.DetectorParameters()
    #
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 45
    p.adaptiveThreshWinSizeStep = 6
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.minCornerDistanceRate = 0.01
    p.minMarkerDistanceRate = 0.02
    p.polygonalApproxAccuracyRate = 0.05
    return p


def draw_text(img, name: str, value: float, xy):
    txt = f"{name}: {value:8.4f}"
    cv2.putText(img, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 252, 124), 2, cv2.LINE_AA)


def open_picamera2(width: int, height: int, fps: int) -> Picamera2:
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": float(fps)},
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(0.2)
    return picam2

#TODO: voir comment diriger tout le flux vidéo vers le sol, sur Raspberry Pi,
# nous ne devrions pas ouvrir une fenêtre video pour la pose estimation. Consommation RAM, etc !
def main():
    ap = argparse.ArgumentParser(description="Pose estimation ArUco (PiCamera2 + OpenCV ArUco)")
    ap.add_argument("-d", type=int, default=16, help="Dictionary id (0..16) comme FDCL")
    ap.add_argument("-l", type=float, required=True, help="Marker length en mètres (ex: 0.064)")
    ap.add_argument("--calib", default="calibration_params.yml", help="Fichier calibration yml")

    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--wait", type=int, default=10, help="waitKey (ms) si GUI")

    ap.add_argument("--axis", type=float, default=0.10, help="Longueur des axes (m) pour drawFrameAxes")
    ap.add_argument("--no-gui", action="store_true", help="Ne pas ouvrir de fenêtre (headless)")
    ap.add_argument("--save-every", type=int, default=0, help="Sauvegarde une frame toutes les N frames")
    ap.add_argument("--out", default="pose_debug.jpg", help="Nom fichier debug si --save-every > 0")
    args = ap.parse_args()

    if args.l <= 0:
        raise SystemExit("Marker length (-l) doit être > 0 (en mètres).")

    if args.d not in DICT_ID_TO_NAME:
        raise SystemExit("d doit être dans 0..16")

    dict_name = DICT_ID_TO_NAME[args.d]
    if not hasattr(cv2.aruco, dict_name):
        raise SystemExit(f"Ton OpenCV ne fournit pas {dict_name}")

    camera_matrix, dist_coeffs = load_calibration(args.calib)

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    detector = cv2.aruco.ArucoDetector(dictionary, make_detector_params())

    # PiCamera2
    picam2 = open_picamera2(args.width, args.height, args.fps)

    frame_i = 0
    out_path = Path(args.out)

    print("Pose estimation (PiCamera2). ESC pour quitter." if not args.no_gui else "Pose estimation (headless). Ctrl+C pour quitter.")

    try:
        while True:
            rgb = picam2.capture_array()
            if rgb is None:
                time.sleep(0.01)
                continue

            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            vis = frame.copy()
            frame_i += 1

            corners, ids, rejected = detector.detectMarkers(frame)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)

                rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.l, camera_matrix, dist_coeffs
                )


                print(f"Translation: {tvecs[0].ravel()} \tRotation: {rvecs[0].ravel()}")

                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        vis, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], args.axis
                    )

                draw_text(vis, "x", float(tvecs[0][0][0]), (10, 30))
                draw_text(vis, "y", float(tvecs[0][0][1]), (10, 55))
                draw_text(vis, "z", float(tvecs[0][0][2]), (10, 80))

            # Debug save
            if args.save_every and (frame_i % args.save_every == 0):
                cv2.imwrite(str(out_path), vis)
                print(f"[INFO] wrote {out_path} (markers={0 if ids is None else len(ids)})")

            # GUI
            #TODO: ce flux doit être redirigé au sol
            if not args.no_gui:
                cv2.imshow("Pose estimation", vis)
                key = cv2.waitKey(args.wait) & 0xFF
                if key == 27:  # ESC
                    break

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
