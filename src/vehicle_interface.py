import time
from pymavlink import mavutil
import math
from gpiozero import DigitalInputDevice

class VehicleInterface:
    def __init__(self):
        self.vehicle = None
        self.gpio_pin = DigitalInputDevice(18, pull_up=False)
        self.gpio_pin.when_activated = self._update_signal_timestamp
        self.stats = {
            "mode": "UNKNOWN",
            "alt": 0.0,
            "groundspeed": 0.0,
            "battery_voltage": 0.0,
            "battery_remaining": 0,
            "gps_fix": 0,
            "armed": False,
            "last_heartbeat": 0,
            "last_signal_gpio": 0,
        }

    def connect_to_vehicle(self, connection_string='/dev/serial0', baud=921600, timeout=5):
        print("\n--- Connexion UART (PyMavlink) ---")
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            print("Attente du Heartbeat...")
            self.master.wait_heartbeat(timeout=timeout)
            self.stats["last_heartbeat"] = time.time()
            print(f"Connecté ! (System ID: {self.master.target_system})")
        except Exception as e:
            print(f"Erreur de connexion : {e}")

    def _update_signal_timestamp(self):
        self.stats["last_signal_gpio"] = time.time()

    def get_vehicle_status(self):
        self.update_metrics()
        return self.stats

    def update_metrics(self):
        if not self.master: return

        msg = self.master.recv_match(blocking=False)
        while msg:
            msg_type = msg.get_type()
            
            if msg_type == 'HEARTBEAT':
                self.stats["mode"] = mavutil.mode_string_v10(msg)
                self.stats["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.stats["last_heartbeat"] = time.time()
            
            elif msg_type == 'VFR_HUD':
                self.stats["alt"] = msg.alt
                self.stats["groundspeed"] = msg.groundspeed
            
            elif msg_type == 'SYS_STATUS':
                self.stats["battery_voltage"] = msg.voltage_battery / 1000.0 
                self.stats["battery_remaining"] = msg.battery_remaining
            
            elif msg_type == 'GPS_RAW_INT':
                self.stats["gps_fix"] = msg.fix_type

            elif msg_type == 'POSITION_TARGET_LOCAL_NED':
                print(f"\n[ACK] Cible validée -> N:{msg.x:.2f}m, E:{msg.y:.2f}m")

            msg = self.master.recv_match(blocking=False)

    def move_target_distance(self,distance_x, distance_y, distance_alt):
        angle_x = math.atan2(distance_y,distance_alt)
        angle_y = math.atan2(distance_x,distance_alt)
        distance = (distance_x**2 + distance_y**2 + distance_alt**2) ** (1/2)

        self.move_target_angle(angle_x, angle_y, distance)

    #angles en radiants!!!
    #Et angle x c'est devant arrière et y c'est droite gauche
    def move_target_angle(self, angle_x, angle_y, distance):
        if not self.master: return
        self.master.mav.landing_target_send(
            0,  
            0,  
            mavutil.mavlink.MAV_FRAME_BODY_NED,  
            angle_x,  
            angle_y,  
            distance,  
            1,  
            1   
        )

    def get_vehicle_should_drop(self):
        return time.time()- self.stats["last_signal_gpio"] <1 
