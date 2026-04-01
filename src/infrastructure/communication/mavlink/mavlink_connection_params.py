from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MavlinkConnectionParams:
    address: str
    port: int = 0
    timeout: float = 10.0
    baud_rate: int = 921600