import os
import time
import threading
import argparse
import logging
import math

import cv2
import numpy as np

# -- Configuration par défaut (remplacer par ta calibration pour précision)
MARKER_DICT = cv2.aruco.DICT_4X4_50
MARKER_LENGTH_M = 0.05  # taille en mètres
CAMERA_MATRIX = np.array([[600.0, 0.0, 320.0],
                          [0.0, 600.0, 240.0],
                          [0.0,   0.0,   1.0]], dtype=float)
DIST_COEFFS = np.zeros((5,), dtype=float)

# global frame shared with MJPEG server
latest_frame = None
frame_lock = threading.Lock()

# Try to import dronekit optionally
try:
    from dronekit import connect
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except Exception:
    DRONEKIT_AVAILABLE = False

# ArUco detector init
aruco_dict = cv2.aruco.getPredefinedDictionary(MARKER_DICT)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


def open_camera_try(width=640, height=480, test_frames=3, timeout=0.1):
    """
    Try Picamera2 then several /dev/video* indices (V4L2). Returns ("picamera2", obj) or ("opencv", cap) or (None, None).
    """
    # try Picamera2
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_preview_configuration({"format": "XRGB8888", "size": (width, height)})
        picam2.configure(cfg)
        picam2.start()
        time.sleep(0.1)
        arr = picam2.capture_array()
        if arr is None:
            picam2.stop()
            raise RuntimeError("Picamera2 returned no frame")
        logging.info("PiCamera opened via Picamera2")
        return ("picamera2", picam2)
    except Exception as e:
        logging.debug("Picamera2 not usable: %s", e)

    # try OpenCV VideoCapture (V4L2)
    device_candidates = ["/dev/video0", "/dev/video1", 0, 1, 2, 3]
    backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
    for backend in backends:
        for dev in device_candidates:
            try:
                cap = cv2.VideoCapture(dev, backend)
            except Exception:
                cap = None
            if cap is None:
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            time.sleep(timeout)
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                continue
            ok = False
            for _ in range(test_frames):
                ret, f = cap.read()
                if ret and f is not None:
                    ok = True
                    break
                time.sleep(timeout)
            if ok:
                logging.info("PiCamera opened via OpenCV device=%s backend=%s", str(dev), backend)
                return ("opencv", cap)
            try:
                cap.release()
            except Exception:
                pass
    return (None, None)


def read_frame(source):
    kind, obj = source
    if kind == "picamera2":
        arr = obj.capture_array()
        if arr is None:
            return None
        # convert XRGB/XBGR -> BGR
        if arr.ndim == 3 and arr.shape[2] == 4:
            return arr[:, :, :3][:, :, ::-1]
        return arr
    elif kind == "opencv":
        ret, frame = obj.read()
        return frame if ret else None
    return None


def release_camera(source):
    kind, obj = source
    try:
        if kind == "picamera2":
            obj.stop()
        elif kind == "opencv":
            obj.release()
    except Exception:
        pass


def mjpeg_server_thread(host, port, jpeg_quality=80, max_w=800):
    try:
        from flask import Flask, Response, render_template_string
    except Exception:
        logging.error("Flask non installé. pip install flask pour MJPEG.")
        return
    app = Flask(__name__)
    HTML = "<html><body><h3>ArUco PiCamera</h3><img src='/video_feed'></body></html>"

    def gen():
        while True:
            try:
                with frame_lock:
                    f = latest_frame.copy() if latest_frame is not None else None
                if f is None:
                    time.sleep(0.05)
                    continue
                h, w = f.shape[:2]
                if w > max_w:
                    new_h = int(h * (max_w / w))
                    f = cv2.resize(f, (max_w, new_h), interpolation=cv2.INTER_LINEAR)
                try:
                    _, jpg = cv2.imencode('.jpg', f, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
                    b = jpg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + b + b'\r\n')
                except Exception:
                    logging.exception("JPEG encoding failed")
                    time.sleep(0.05)
                    continue
                time.sleep(0.03)
            except GeneratorExit:
                break
            except Exception:
                logging.exception("Unhandled error in MJPEG generator")
                time.sleep(0.1)
                continue

    @app.route('/')
    def index():
        return render_template_string(HTML)

    @app.route('/video_feed')
    def video_feed():
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    logging.info("Starting MJPEG server at http://%s:%s", host, port)
    app.run(host=host, port=port, threaded=True)

import numpy as np
import cv2

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    obj = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0],
    ], dtype=np.float32)

    rvecs, tvecs, oks = [], [], []

    for c in corners:
        img = np.asarray(c, dtype=np.float32).reshape(-1, 2)  # (4,2)

        ok, rvec, tvec = cv2.solvePnP(
            obj, img, mtx, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        oks.append(ok)
        rvecs.append(rvec)
        tvecs.append(tvec)

    return rvecs, tvecs, oks


def detection_loop(source, show_gui, marker_length, camera_matrix, dist_coeffs, send_mav=False, vehicle=None):
    global latest_frame
    failure_count = 0
    while True:
        try:
            frame = read_frame(source)
            if frame is None:
                logging.warning("Frame None — try to reopen camera")
                release_camera(source)
                time.sleep(0.5)
                source = open_camera_try()
                if source[0] is None:
                    logging.error("Cannot reopen camera")
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                try:
                    rvecs, tvecs, _ = estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
                    ids_flat = ids.flatten()

                    # dessine tous les markers détectés une seule fois
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    for rvec, tvec, mid in zip(rvecs, tvecs, ids_flat):
                        t = np.asarray(tvec, dtype=np.float64).reshape(3)
                        r = np.asarray(rvec, dtype=np.float64).reshape(3)

                        dist = float(np.linalg.norm(t))
                        ang_x = math.atan2(float(t[0]), float(t[2]))
                        ang_y = math.atan2(float(t[1]), float(t[2]))

                        logging.info("id=%d dist=%.2f m ang_x=%.3f rad ang_y=%.3f rad",
                                    int(mid), dist, ang_x, ang_y)

                        try:
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, r, t.reshape(3,1), marker_length / 2)
                        except Exception:
                            pass

                        if send_mav and vehicle is not None:
                            send_land_message_mav(vehicle, ang_x, ang_y, dist)

                    failure_count = 0
                except Exception:
                    failure_count += 1
                    logging.exception("ArUco pose estimation failed (ignored)")
                    if failure_count > 10:
                        logging.error("Too many consecutive ArUco errors, reopening camera")
                        release_camera(source)
                        time.sleep(0.5)
                        source = open_camera_try()
                        failure_count = 0
                        if source[0] is None:
                            break

            with frame_lock:
                latest_frame = frame.copy()

            if show_gui:
                try:
                    cv2.imshow("ArUco", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except Exception:
                    logging.info("GUI not available, switching to headless mode")
                    show_gui = False

        except KeyboardInterrupt:
            break
        except Exception:
            logging.exception("Unexpected error in main loop")
            time.sleep(0.1)
            continue

    release_camera(source)
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


def send_land_message_mav(vehicle, x_rad, y_rad, dist_m):
    """
    Send LANDING_TARGET message to MAVLink vehicle if available.
    Non-blocking and tolerant to failures.
    """
    if vehicle is None:
        return
    try:
        # time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y
        msg = vehicle.message_factory.landing_target_encode(
            int(time.time() * 1e6),
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            float(x_rad),
            float(y_rad),
            float(dist_m),
            0.0,
            0.0
        )
        vehicle.send_mavlink(msg)
        try:
            vehicle.flush()
        except Exception:
            pass
    except Exception:
        logging.exception("Failed to send MAVLink LANDING_TARGET")


def try_connect_vehicle(conn_strings, baud=57600, timeout=5):
    if not DRONEKIT_AVAILABLE:
        logging.info("dronekit not available, skipping vehicle connection")
        return None
    for cs in conn_strings:
        try:
            logging.info("Trying vehicle connection: %s", cs)
            v = connect(cs, baud=baud, wait_ready=False, timeout=timeout)
            # small wait for heartbeat optionally (non-blocking long)
            t0 = time.time()
            while time.time() - t0 < 3:
                try:
                    if v.last_heartbeat:
                        break
                except Exception:
                    break
                time.sleep(0.1)
            logging.info("Connected to vehicle via %s", cs)
            return v
        except Exception as e:
            logging.debug("Vehicle connection failed %s : %s", cs, e)
    logging.warning("No vehicle connection established")
    return None


def main():
    parser = argparse.ArgumentParser(description="ArUco detection + optional MJPEG and MAVLink output")
    parser.add_argument("--cam", type=int, default=0, help="camera index for OpenCV fallback")
    parser.add_argument("--marker-size", type=float, default=MARKER_LENGTH_M, help="marker size in meters")
    parser.add_argument("--id", type=int, default=None, help="filter a single marker id")
    parser.add_argument("--mjpg", action="store_true", help="start MJPEG server (headless)")
    parser.add_argument("--host", default="0.0.0.0", help="MJPEG host")
    parser.add_argument("--port", type=int, default=8200, help="MJPEG port")
    parser.add_argument("--no-mav", action="store_true", help="do not try to connect/send MAVLink")
    args = parser.parse_args()

    marker_length = args.marker_size

    # open camera
    source = open_camera_try()
    if source[0] is None:
        logging.error("Cannot open camera. Check libcamera/v4l2 and device permissions.")
        return

    # optional MAVLink connect
    vehicle = None
    send_mav = False
    if not args.no_mav:
        conn_strings = ["COM6", "/dev/serial0", "/dev/ttyAMA0", "/dev/ttyUSB0", "udp:127.0.0.1:14550"]
        vehicle = try_connect_vehicle(conn_strings)
        send_mav = vehicle is not None

    display_ok = bool(os.environ.get("DISPLAY"))
    use_gui = display_ok and not args.mjpg

    if args.mjpg or not use_gui:
        t = threading.Thread(target=mjpeg_server_thread, args=(args.host, args.port), daemon=True)
        t.start()
        logging.info("MJPEG server thread started")

    try:
        detection_loop(source, show_gui=use_gui, marker_length=marker_length,
                       camera_matrix=CAMERA_MATRIX, dist_coeffs=DIST_COEFFS,
                       send_mav=send_mav, vehicle=vehicle)
    finally:
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()