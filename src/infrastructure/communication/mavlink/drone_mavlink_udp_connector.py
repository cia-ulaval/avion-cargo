from pymavlink import mavutil

from .drone_mavlink_base import DroneMavlinkBase
from .mavlink_connection_params import MavlinkConnectionParams


class DroneMavlinkUDPConnector(DroneMavlinkBase):
    def __init__(self, params: MavlinkConnectionParams):
        super().__init__(params)

    def _init_mavlink_connection(self) -> None:
        self.connection = mavutil.mavlink_connection(f"udp:{self.parameters.address}:{self.parameters.port}")
