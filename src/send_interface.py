class SendInterface:

    def __init__(self):
        self.interface = None

    def connect_interface(self, ip, timeout):
        raise NotImplementedError()

    def send_camera(self, frame, corners):
        raise NotImplementedError()

    def send_vehicle_feedback(self):
        raise NotImplementedError()
