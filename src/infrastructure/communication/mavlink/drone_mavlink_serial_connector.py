from pymavlink import mavutil

from .mavlink_connection_params import MavlinkConnectionParams
from .drone_mavlink_base import DroneMavlinkBase


class DroneMavlinkSerialConnector(DroneMavlinkBase):
    def __init__(self, params: MavlinkConnectionParams):
        super().__init__(params)

    def _init_mavlink_connection(self) -> None:
        self.connection = mavutil.mavlink_connection(self.parameters.address, baud=self.parameters.baud_rate)
