from interface_waiter import Interface
from movements import Movements


class Controller:

    def __init__(self, waiter):
        self.movements = Movements()
        self.frame = None
        self.waiter = waiter

    def init(self):
        raise NotImplementedError()

    def subscribe_observer(self, main_view):
        raise NotImplementedError()

    def unsubscribe_observer(self, main_view):
        raise NotImplementedError()

    def get_frame(self):
        raise NotImplementedError()

    def get_movement(self):
        raise NotImplementedError()

    def change_frame(self, frame):
        self.frame = frame

    def add_movement(self, movement):
        self.movements.add_movement(movement)
