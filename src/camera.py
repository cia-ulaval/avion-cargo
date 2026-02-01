class Camera:
    def __init__(self):
        self.source = None

    def connect_to_camera(self, width, height, test_frame, timeout):
        raise NotImplementedError

    def get_frame(self):
        raise NotImplementedError