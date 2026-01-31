class ImageTreatment:
    def __init__(self):
        self.latest_frame = None

    def get_corner_position(self, frame):
        raise NotImplementedError()