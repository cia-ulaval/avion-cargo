from application.tracking_service import TrackingService
from domain.content_diffuser import ContentDiffuser
from domain.drone import Drone


class DroneAutolandingService:
    def __init__(self, drone: Drone, tracker: TrackingService, diffuser: ContentDiffuser):
        self.drone = drone
        self.aruco_tracker = tracker
        self.diffuser = diffuser

    def start(self):
        pass