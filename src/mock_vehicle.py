# Mock Vehicle - simulates drone telemetry and commands for testing
import time
import logging
import numpy as np
from vehicle_interface import VehicleInterface

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


class MockVehicleInterface(VehicleInterface):
    
    def __init__(self):
        super().__init__()
        self.start_time = time.time()
        self.simulated_mode = "GUIDED"
        self.simulated_armed = False
        self.command_count = 0
        
    def connect_to_vehicle(self, connection_string=None, baud=57600, timeout=5):
        self.is_connected = True
        self.connection_string = "simulated://demo"
        logging.info("MockVehicle connected")
        return True
    
    def get_vehicle_status(self):
        runtime = time.time() - self.start_time
        
        # Simulate battery draining over time
        battery_voltage = 12.6 - (runtime / 3600) * 0.5
        battery_voltage = max(10.5, battery_voltage)
        
        # Simulate GPS acquiring satellites
        gps_sats = 12 if runtime > 5 else int(runtime * 2)
        gps_fix = 3 if gps_sats >= 6 else 0
        
        # Simulate position drifting slightly (like real GPS)
        lat = 46.7812 + np.sin(runtime / 60) * 0.0001
        lon = -71.2826 + np.cos(runtime / 60) * 0.0001
        alt = 100.0 + np.sin(runtime / 30) * 5.0
        
        return {
            'connected': True,
            'mode': self.simulated_mode,
            'armed': self.simulated_armed,
            'battery': battery_voltage,
            'gps': {
                'satellites': gps_sats,
                'fix_type': gps_fix
            },
            'location': {
                'lat': lat,
                'lon': lon,
                'alt': alt
            },
            'last_heartbeat': 0.1
        }
    
    def move_vehicle(self, markers):
        if not markers or len(markers) == 0:
            return False
        
        self.command_count += 1
        closest = min(markers, key=lambda m: m['distance'])
        
        # Switch to landing mode after a few commands
        if self.command_count > 10:
            self.simulated_mode = "LAND"
        
        logging.debug(f"Command #{self.command_count} - Marker {closest['id']} at {closest['distance']:.2f}m")
        return True
    
    def send_landing_target(self, angle_x_rad, angle_y_rad, distance_m):
        logging.debug(f"Landing target: x={angle_x_rad:.3f}, y={angle_y_rad:.3f}, d={distance_m:.2f}m")
        return True
    
    def get_vehicle_feedback(self, markers):
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
        self.is_connected = False
        logging.info("MockVehicle disconnected")
