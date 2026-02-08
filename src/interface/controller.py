import threading
import time
import logging
from .interface_waiter import Interface
from .movements import Movements

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class Controller:
    # Main controller - coordinates camera, vision processing, and vehicle control
    
    def __init__(self, waiter, camera=None, image_treatment=None, vehicle_interface=None):
        self.movements = Movements()
        self.frame = None
        self.waiter = waiter
        
        # References to components
        self.camera = camera
        self.image_treatment = image_treatment
        self.vehicle_interface = vehicle_interface
        
        # State tracking
        self.running = False
        self.thread = None
        self.update_rate = 30  # Hz
        
        # Latest detection results
        self.latest_markers = []
        self.latest_annotated_frame = None
        self.latest_vehicle_status = None
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.total_detections = 0
        self.successful_poses = 0
        self.start_time = None
        
        logging.info("Controller initialized")
    
    def init(self):
        # Initialize all components
        success = True
        
        # Connect camera
        if self.camera is not None:
            if not self.camera.is_connected:
                if not self.camera.connect_to_camera():
                    logging.error("Failed to initialize camera")
                    success = False
                else:
                    logging.info("Camera initialized")
        
        # Connect vehicle
        if self.vehicle_interface is not None:
            if not self.vehicle_interface.is_connected:
                if not self.vehicle_interface.connect_to_vehicle():
                    logging.warning("Vehicle not connected (continuing anyway)")
                else:
                    logging.info("Vehicle initialized")
        
        if self.image_treatment is not None:
            logging.info("Image treatment ready")
        
        return success
    
    def subscribe_observer(self, observer):
        self.waiter.subscribe_observer(observer)
    
    def unsubscribe_observer(self, observer):
        self.waiter.unsubscribe_observer(observer)
    
    def get_frame(self):
        return self.latest_annotated_frame
    
    def get_movements(self):
        return self.movements.get_movements()
    
    def change_frame(self, frame):
        self.frame = frame
    
    def add_movement(self, movement):
        self.movements.add_movement(movement)
    
    def start(self):
        # Start the main processing loop in a thread
        if self.running:
            logging.warning("Controller already running")
            return
        
        self.running = True
        self.start_time = time.time()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        logging.info("Controller started")
    
    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        logging.info("Controller stopped")
    
    def _run_loop(self):
        # Main loop - captures frames, detects markers, sends commands
        frame_time = 1.0 / self.update_rate
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Get frame from camera
                if self.camera is not None:
                    frame = self.camera.get_frame()
                    
                    if frame is None:
                        logging.warning("Failed to capture frame")
                        time.sleep(0.1)
                        continue
                    
                    self.frame_count += 1
                    
                    # Detect markers
                    if self.image_treatment is not None:
                        result = self.image_treatment.get_corner_position(frame)
                        self.latest_annotated_frame = result['frame']
                        self.latest_markers = result['markers']
                        
                        # Update precision tracking
                        self.total_detections += result.get('total_detections', 0)
                        self.successful_poses += result.get('successful_poses', 0)
                        
                        if len(self.latest_markers) > 0:
                            self.detection_count += 1
                        
                        # Send commands to vehicle if connected
                        if self.vehicle_interface is not None and self.vehicle_interface.is_connected:
                            if len(self.latest_markers) > 0:
                                success = self.vehicle_interface.move_vehicle(self.latest_markers)
                                if success:
                                    closest = min(self.latest_markers, key=lambda m: m['distance'])
                                    movement = {
                                        'type': 'landing_target',
                                        'timestamp': time.time(),
                                        'data': {
                                            'marker_id': closest['id'],
                                            'distance': closest['distance'],
                                            'angle_x': closest['angle_x'],
                                            'angle_y': closest['angle_y']
                                        }
                                    }
                                    self.add_movement(movement)
                        
                        # Get vehicle status
                        if self.vehicle_interface is not None:
                            self.latest_vehicle_status = self.vehicle_interface.get_vehicle_feedback(
                                self.latest_markers
                            )
                    else:
                        self.latest_annotated_frame = frame
                    
                    # Notify observers with latest data
                    data = {
                        'frame': self.latest_annotated_frame,
                        'markers': self.latest_markers,
                        'vehicle_status': self.latest_vehicle_status,
                        'statistics': self.get_statistics()
                    }
                    self.waiter.trigger_observers(data)
                
            except Exception as e:
                logging.exception(f"Error in controller loop: {e}")
                time.sleep(0.1)
            
            # Maintain update rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, frame_time - elapsed)
            time.sleep(sleep_time)
    
    def get_statistics(self):
        runtime = time.time() - self.start_time if self.start_time else 0
        precision_rate = self.successful_poses / self.total_detections if self.total_detections > 0 else 0
        
        return {
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'total_detections': self.total_detections,
            'successful_poses': self.successful_poses,
            'runtime': runtime,
            'fps': self.frame_count / runtime if runtime > 0 else 0,
            'detection_rate': self.detection_count / self.frame_count if self.frame_count > 0 else 0,
            'precision_rate': precision_rate,
            'movement_count': self.movements.get_count()
        }
    
    def cleanup(self):
        self.stop()
        
        if self.camera is not None:
            self.camera.disconnect()
        
        if self.vehicle_interface is not None:
            self.vehicle_interface.disconnect()
        
        logging.info("Controller cleanup complete")