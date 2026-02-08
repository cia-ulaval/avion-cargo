# Movement history tracker
import threading
import logging
from collections import deque

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class Movements:
    # Stores history of movement commands sent to vehicle
    
    def __init__(self, max_size=1000):
        self.movements = deque(maxlen=max_size)
        self._lock = threading.Lock()
        self.max_size = max_size
    
    def add_movement(self, movement):
        with self._lock:
            self.movements.append(movement)
            logging.debug(f"Movement added: {movement}")
    
    def get_movements(self, count=None):
        # Get recent movements (all if count=None)
        with self._lock:
            if count is None:
                return list(self.movements)
            else:
                return list(self.movements)[-count:]
    
    def get_latest_movement(self):
        with self._lock:
            return self.movements[-1] if self.movements else None
    
    def clear(self):
        with self._lock:
            self.movements.clear()
            logging.info("Movement queue cleared")
    
    def get_count(self):
        with self._lock:
            return len(self.movements)
    
    def get_statistics(self):
        with self._lock:
            if not self.movements:
                return {'count': 0, 'types': {}}
            
            # Count movement types
            types_count = {}
            for move in self.movements:
                move_type = move.get('type', 'unknown')
                types_count[move_type] = types_count.get(move_type, 0) + 1
            
            return {
                'count': len(self.movements),
                'types': types_count,
                'max_size': self.max_size
            }