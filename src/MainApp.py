import time
import threading
import vehicle_interface

class DroneApp:
    def __init__(self):
        self.running = False
        self.vehicle = vehicle_interface()
        self.state = {}

    def control_loop(self):
        print("Boucle démarrée")
        
        while self.running:
            start = time.time()
            #sécurité pour être sur qu'il a un vehicule
            if(self.vehicle.vehicle is not None and self.vehicle.get_vehicle_should_drop()): 
                # Récupérer les stat Caméra ici
                distance_x = 0
                distance_y = 0
                distance_alt = 0            

                self.vehicle.move_target_distance(distance_x, distance_y, distance_alt)
            elapsed = time.time() - start
            sleep_time = max(0, 0.05 - elapsed) #0.05 à changer pour changer le nb de hz
            time.sleep(sleep_time)
        print("Boucle stoppée")

    def start(self):
        self.vehicle.connect_to_vehicle()
        self.running = True

        self.thread = threading.Thread(target=self.control_loop)
        self.thread.daemon = True
        self.thread.start()

        print("App démarrée en arrière-plan")

    def stop(self):
        self.running = False
        self.thread.join()
        print("App arrêtée")


if __name__ == "__main__":
    app = DroneApp()
    app.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        app.stop()