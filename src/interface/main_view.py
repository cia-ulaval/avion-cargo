class MainView:

    def __init__(self, controller):
        self.controller = controller

    def initialize(self):
        raise NotImplementedError()

    def update_interface(self):
        raise NotImplementedError()