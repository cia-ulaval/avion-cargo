class VehicleInterface:

    def __init__(self):
        self.vehicle = None

    def connect_to_vehicle(self, connection_string, baud=5000, timeout=5):
        raise NotImplementedError()

    def get_vehicle_status(self):
        raise NotImplementedError()

    def move_vehicle(self, corners):
        raise NotImplementedError()

    def get_vehicle_feedback(self, corners):
        raise NotImplementedError()
