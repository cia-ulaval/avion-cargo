import time
import logging
import cv2

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class Camera:
    # Camera interface - supports Picamera2 (Raspberry Pi) or regular USB webcam
    
    def __init__(self):
        self.source = None
        self.source_type = None
        self.is_connected = False
        
    def connect_to_camera(self, width=640, height=480, test_frames=3, timeout=0.1):
        # Try to connect to camera (Picamera2 first, then USB webcam)
        self.source = self._open_camera_try(width, height, test_frames, timeout)
        self.is_connected = self.source[0] is not None
        
        if self.is_connected:
            self.source_type = self.source[0]
            logging.info(f"Camera connected via {self.source_type}")
        else:
            logging.error("Failed to connect to any camera")
            
        return self.is_connected
    
    def _open_camera_try(self, width, height, test_frames, timeout):
        # Try Picamera2 first, then fall back to OpenCV
        # Try Picamera2
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            cfg = picam2.create_preview_configuration({
                "format": "XRGB8888",
                "size": (width, height)
            })
            picam2.configure(cfg)
            picam2.start()
            time.sleep(0.1)
            arr = picam2.capture_array()
            if arr is None:
                picam2.stop()
                raise RuntimeError("Picamera2 returned no frame")
            logging.info("Camera opened via Picamera2")
            return ("picamera2", picam2)
        except Exception as e:
            logging.debug(f"Picamera2 not usable: {e}")
        
        # Try OpenCV VideoCapture
        device_candidates = ["/dev/video0", "/dev/video1", 0, 1, 2, 3]
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        
        for backend in backends:
            for dev in device_candidates:
                try:
                    cap = cv2.VideoCapture(dev, backend)
                except Exception:
                    continue
                    
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
                
                # Test frame capture
                ok = False
                for _ in range(test_frames):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        ok = True
                        break
                    time.sleep(timeout)
                
                if ok:
                    logging.info(f"Camera opened via OpenCV device={dev} backend={backend}")
                    return ("opencv", cap)
                
                try:
                    cap.release()
                except Exception:
                    pass
        
        return (None, None)
    
    def get_frame(self):
        # Capture and return current frame (BGR format)
        if not self.is_connected or self.source is None:
            logging.warning("Camera not connected")
            return None
        
        kind, obj = self.source
        
        if kind == "picamera2":
            arr = obj.capture_array()
            if arr is None:
                return None
            # Convert XRGB8888 to BGR
            if arr.ndim == 3 and arr.shape[2] == 4:
                return arr[:, :, :3][:, :, ::-1]
            return arr
            
        elif kind == "opencv":
            ret, frame = obj.read()
            return frame if ret else None
        
        return None
    
    def reconnect(self, width=640, height=480):
        # Try to reconnect camera after failure
        logging.info("Attempting camera reconnection...")
        self.disconnect()
        time.sleep(0.5)
        return self.connect_to_camera(width, height)
    
    def disconnect(self):
        # Release camera resources
        if self.source is None:
            return
        
        kind, obj = self.source
        try:
            if kind == "picamera2":
                obj.stop()
            elif kind == "opencv":
                obj.release()
            logging.info("Camera disconnected")
        except Exception as e:
            logging.error(f"Error disconnecting camera: {e}")
        finally:
            self.source = None
            self.is_connected = False
    
    def __del__(self):
        # Cleanup on destruction
        self.disconnect()