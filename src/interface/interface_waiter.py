class Interface:

    def __init__(self):
        self.observers = []

    def subscribe_observer(self, observer):
        self.observers.append(observer)

    def unsubscribe_observer(self, observer):
        self.observers.remove(observer)

    def trigger_observers(self, observer):
        raise NotImplementedError()

    def listen_observers(self):
        raise NotImplementedError()
