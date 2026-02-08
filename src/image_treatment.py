import cv2
import numpy as np
import math
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class ImageTreatment:
    # ArUco marker detection and pose estimation
    
    def __init__(self, marker_dict=cv2.aruco.DICT_4X4_50, marker_length_m=0.05,
                 camera_matrix=None, dist_coeffs=None):
        self.latest_frame = None
        self.marker_length = marker_length_m
        
        # Default camera calibration (should be replaced with actual calibration)
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=float)
        
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((5,), dtype=float)
        
        # Initialize ArUco detector with tuned parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(marker_dict)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Tune detection parameters for better detection rate
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.03
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 3
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        logging.info("ImageTreatment initialized with marker size: %.2fm", marker_length_m)
    
    def detect_markers(self, frame):
        # Detect ArUco markers in frame
        # Returns: (corners, ids, rejected)
        if frame is None:
            return None, None, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Enhance contrast for better detection
        gray = cv2.equalizeHist(gray)
        
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        return corners, ids, rejected
    
    def estimate_pose_single_markers(self, corners):
        # Estimate 3D pose for each marker
        # Returns: (rvecs, tvecs, success_flags)
        marker_points_3d = np.array([
            [-self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, -self.marker_length/2, 0],
            [-self.marker_length/2, -self.marker_length/2, 0],
        ], dtype=np.float32)
        
        rvecs, tvecs, success = [], [], []
        
        for corner in corners:
            img_points = np.asarray(corner, dtype=np.float32).reshape(-1, 2)
            
            ok, rvec, tvec = cv2.solvePnP(
                marker_points_3d,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            success.append(ok)
            rvecs.append(rvec)
            tvecs.append(tvec)
        
        return rvecs, tvecs, success
    
    def get_corner_position(self, frame):
        # Detect markers and compute their 3D positions
        # Returns dict with: frame, markers list, total_detections, successful_poses
        if frame is None:
            return {'frame': None, 'markers': [], 'total_detections': 0, 'successful_poses': 0}
        
        self.latest_frame = frame.copy()
        
        # Detect markers
        corners, ids, _ = self.detect_markers(frame)
        
        markers_info = []
        annotated_frame = frame.copy()
        total_detections = 0
        successful_poses = 0
        
        if ids is not None and len(ids) > 0:
            total_detections = len(ids)
            
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(annotated_frame, corners, ids)
            
            # Estimate poses
            rvecs, tvecs, success = self.estimate_pose_single_markers(corners)
            ids_flat = ids.flatten()
            
            for rvec, tvec, marker_id, ok in zip(rvecs, tvecs, ids_flat, success):
                if not ok:
                    continue
                
                successful_poses += 1
                
                # Extract translation vector
                t = np.asarray(tvec, dtype=np.float64).reshape(3)
                r = np.asarray(rvec, dtype=np.float64).reshape(3)
                
                # Calculate distance and angles
                distance = float(np.linalg.norm(t))
                angle_x = math.atan2(float(t[0]), float(t[2]))  # Horizontal angle
                angle_y = math.atan2(float(t[1]), float(t[2]))  # Vertical angle
                
                # Store marker info
                marker_data = {
                    'id': int(marker_id),
                    'distance': distance,
                    'angle_x': angle_x,
                    'angle_y': angle_y,
                    'tvec': t,
                    'rvec': r,
                    'corners': corners[len(markers_info)]
                }
                markers_info.append(marker_data)
                
                # Draw 3D axes on marker
                try:
                    cv2.drawFrameAxes(
                        annotated_frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        r,
                        t.reshape(3, 1),
                        self.marker_length / 2
                    )
                except Exception as e:
                    logging.debug(f"Failed to draw axes: {e}")
                
                # Add text annotation
                corner_2d = corners[len(markers_info) - 1][0][0]
                text = f"ID:{marker_id} D:{distance:.2f}m"
                cv2.putText(
                    annotated_frame,
                    text,
                    (int(corner_2d[0]), int(corner_2d[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
                
                logging.info(
                    f"Marker ID={marker_id}: dist={distance:.2f}m, "
                    f"angle_x={angle_x:.3f}rad, angle_y={angle_y:.3f}rad"
                )
        
        return {
            'frame': annotated_frame,
            'markers': markers_info,
            'total_detections': total_detections,
            'successful_poses': successful_poses
        }
    
    def set_camera_calibration(self, camera_matrix, dist_coeffs):
        # Update camera calibration parameters
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        logging.info("Camera calibration updated")