# Mock Camera - generates fake camera frames with ArUco markers for testing
import time
import logging
import numpy as np
import cv2
from camera import Camera

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


class MockCamera(Camera):
    
    def __init__(self, width=640, height=480):
        super().__init__()
        self.width = width
        self.height = height
        self.frame_count = 0
        self.marker_positions = []
        self._init_markers()
        
    def _init_markers(self):
        # Create positions for marker to move in a circle
        self.marker_positions = []
        for i in range(60):
            angle = (i / 60) * 2 * np.pi
            x = int(self.width / 2 + 150 * np.cos(angle))
            y = int(self.height / 2 + 100 * np.sin(angle))
            self.marker_positions.append((x, y))
    
    def connect_to_camera(self, width=640, height=480, test_frames=3, timeout=0.1):
        self.width = width
        self.height = height
        self.is_connected = True
        self.source_type = "mock"
        logging.info("MockCamera connected")
        return True
    
    def get_frame(self):
        if not self.is_connected:
            return None
        
        # Create gray background
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 200
        
        # Add some noise for texture
        noise = np.random.randint(0, 30, (self.height, self.width, 3), dtype=np.uint8)
        frame = cv2.add(frame, noise)
        
        # Draw grid lines
        for i in range(0, self.width, 50):
            cv2.line(frame, (i, 0), (i, self.height), (180, 180, 180), 1)
        for i in range(0, self.height, 50):
            cv2.line(frame, (0, i), (self.width, i), (180, 180, 180), 1)
        
        # Generate ArUco marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker_id = 0
        marker_size = 200
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        
        # Get current position (animated movement)
        pos_index = self.frame_count % len(self.marker_positions)
        marker_x, marker_y = self.marker_positions[pos_index]
        
        # Place marker on frame
        x1 = max(0, marker_x - marker_size // 2)
        y1 = max(0, marker_y - marker_size // 2)
        x2 = min(self.width, x1 + marker_size)
        y2 = min(self.height, y1 + marker_size)
        
        # Resize if marker goes off screen
        actual_width = x2 - x1
        actual_height = y2 - y1
        if actual_width != marker_size or actual_height != marker_size:
            marker_img = cv2.resize(marker_img, (actual_width, actual_height))
        
        # Convert to 3-channel and overlay on frame
        marker_img_3ch = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        frame[y1:y2, x1:x2] = marker_img_3ch
        
        # Add some info text
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, f"DEMO MODE - {timestamp}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1)
        
        self.frame_count += 1
        return frame
    
    def disconnect(self):
        self.is_connected = False
        logging.info("MockCamera disconnected")
