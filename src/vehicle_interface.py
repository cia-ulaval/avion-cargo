import time
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

# Try to import dronekit
try:
    from dronekit import connect
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    logging.warning("DroneKit not available. Install with: pip install dronekit pymavlink")


class VehicleInterface:
    # MAVLink interface for communicating with ArduPilot drone
    
    def __init__(self):
        self.vehicle = None
        self.is_connected = False
        self.connection_string = None
        
    def connect_to_vehicle(self, connection_string=None, baud=57600, timeout=5):
        # Connect to drone via MAVLink
        if not DRONEKIT_AVAILABLE:
            logging.error("DroneKit not available. Cannot connect to vehicle.")
            return False
        
        # Default connection strings to try
        if connection_string is None:
            conn_strings = [
                "/dev/serial0",      # Raspberry Pi UART
                "/dev/ttyAMA0",      # Raspberry Pi UART (older)
                "/dev/ttyUSB0",      # USB serial
                "udp:127.0.0.1:14550",  # SITL
                "COM6"               # Windows COM port
            ]
        else:
            conn_strings = [connection_string]
        
        # Try each connection string
        for cs in conn_strings:
            try:
                logging.info(f"Attempting vehicle connection: {cs}")
                self.vehicle = connect(cs, baud=baud, wait_ready=False, timeout=timeout)
                
                # Wait briefly for heartbeat
                t0 = time.time()
                while time.time() - t0 < 3:
                    try:
                        if self.vehicle.last_heartbeat:
                            self.is_connected = True
                            self.connection_string = cs
                            logging.info(f"✓ Connected to vehicle via {cs}")
                            return True
                    except Exception:
                        break
                    time.sleep(0.1)
                
                # If we get here, connection succeeded but no heartbeat
                self.is_connected = True
                self.connection_string = cs
                logging.info(f"Connected to vehicle via {cs} (no heartbeat yet)")
                return True
                
            except Exception as e:
                logging.debug(f"Connection failed for {cs}: {e}")
                continue
        
        logging.warning("No vehicle connection established")
        return False
    
    def get_vehicle_status(self):
        # Get current vehicle status (mode, battery, GPS, location, etc.)
        if not self.is_connected or self.vehicle is None:
            return {
                'connected': False,
                'mode': 'N/A',
                'armed': False,
                'battery': 0.0,
                'gps': {'satellites': 0, 'fix_type': 0},
                'location': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
                'last_heartbeat': 999
            }
        
        try:
            status = {
                'connected': True,
                'mode': str(self.vehicle.mode.name) if self.vehicle.mode else 'UNKNOWN',
                'armed': bool(self.vehicle.armed) if hasattr(self.vehicle, 'armed') else False,
                'battery': float(self.vehicle.battery.voltage) if self.vehicle.battery else 0.0,
                'gps': {
                    'satellites': int(self.vehicle.gps_0.satellites_visible) if self.vehicle.gps_0 else 0,
                    'fix_type': int(self.vehicle.gps_0.fix_type) if self.vehicle.gps_0 else 0
                },
                'location': {
                    'lat': float(self.vehicle.location.global_frame.lat) if self.vehicle.location.global_frame else 0.0,
                    'lon': float(self.vehicle.location.global_frame.lon) if self.vehicle.location.global_frame else 0.0,
                    'alt': float(self.vehicle.location.global_frame.alt) if self.vehicle.location.global_frame else 0.0
                },
                'last_heartbeat': float(self.vehicle.last_heartbeat) if hasattr(self.vehicle, 'last_heartbeat') else 0.0
            }
            return status
        except Exception as e:
            logging.error(f"Error getting vehicle status: {e}")
            return {
                'connected': self.is_connected,
                'mode': 'ERROR',
                'armed': False,
                'battery': 0.0,
                'gps': {'satellites': 0, 'fix_type': 0},
                'location': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
                'last_heartbeat': 999
            }
    
    def move_vehicle(self, markers):
        # Send landing target command to vehicle using closest marker
        if not self.is_connected or self.vehicle is None:
            return False
        
        if not markers or len(markers) == 0:
            return False
        
        # Use closest marker for landing
        closest_marker = min(markers, key=lambda m: m['distance'])
        
        return self.send_landing_target(
            closest_marker['angle_x'],
            closest_marker['angle_y'],
            closest_marker['distance']
        )
    
    def send_landing_target(self, angle_x_rad, angle_y_rad, distance_m):
        # Send LANDING_TARGET MAVLink message with marker position
        if not self.is_connected or self.vehicle is None:
            return False
        
        try:
            msg = self.vehicle.message_factory.landing_target_encode(
                int(time.time() * 1e6),              # time_usec
                0,                                    # target_num
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                float(angle_x_rad),                   # angle_x
                float(angle_y_rad),                   # angle_y
                float(distance_m),                    # distance
                0.0,                                  # size_x
                0.0                                   # size_y
            )
            self.vehicle.send_mavlink(msg)
            
            try:
                self.vehicle.flush()
            except Exception:
                pass
            
            logging.debug(
                f"Sent LANDING_TARGET: x={angle_x_rad:.3f}rad, "
                f"y={angle_y_rad:.3f}rad, dist={distance_m:.2f}m"
            )
            return True
            
        except Exception as e:
            logging.error(f"Failed to send LANDING_TARGET: {e}")
            return False
    
    def get_vehicle_feedback(self, markers):
        # Combine vehicle status with marker detection info
        status = self.get_vehicle_status()
        status['markers_detected'] = len(markers) if markers else 0
        
        if markers and len(markers) > 0:
            closest = min(markers, key=lambda m: m['distance'])
            status['target_marker'] = {
                'id': closest['id'],
                'distance': closest['distance'],
                'angle_x': closest['angle_x'],
                'angle_y': closest['angle_y']
            }
        else:
            status['target_marker'] = None
        
        return status
    
    def disconnect(self):
        # Close vehicle connection
        if self.vehicle is not None:
            try:
                self.vehicle.close()
                logging.info("Vehicle disconnected")
            except Exception as e:
                logging.error(f"Error disconnecting vehicle: {e}")
            finally:
                self.vehicle = None
                self.is_connected = False
    
    def __del__(self):
        # Cleanup on destruction
        self.disconnect()