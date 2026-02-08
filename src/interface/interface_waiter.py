# Observer pattern implementation for GUI updates
import logging
import threading

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class Interface:
    # Base class for observer pattern - views subscribe to get updates
    
    def __init__(self):
        self.observers = []
        self._lock = threading.Lock()
        
    def subscribe_observer(self, observer):
        # Add observer to receive updates
        with self._lock:
            if observer not in self.observers:
                self.observers.append(observer)
                logging.info(f"Observer {observer.__class__.__name__} subscribed")
    
    def unsubscribe_observer(self, observer):
        with self._lock:
            if observer in self.observers:
                self.observers.remove(observer)
                logging.info(f"Observer {observer.__class__.__name__} unsubscribed")
    
    def trigger_observers(self, data):
        # Notify all observers with new data
        with self._lock:
            observers_copy = self.observers.copy()
        
        for observer in observers_copy:
            try:
                if hasattr(observer, 'update'):
                    observer.update(data)
                else:
                    logging.warning(f"Observer {observer} has no update() method")
            except Exception as e:
                logging.error(f"Error notifying observer {observer}: {e}")
    
    def listen_observers(self):
        # Override this in subclasses if needed
        logging.info("Base Interface.listen_observers() called")
        
    def get_observer_count(self):
        with self._lock:
            return len(self.observers)