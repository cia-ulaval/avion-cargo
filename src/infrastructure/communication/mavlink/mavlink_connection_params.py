import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MavlinkConnectionParams:
    address: str
    port: int = 0
    timeout: float = 30.0
    baud_rate: int = 921600
    fail_on_timeout: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("MAVLink timeout must be finite and greater than zero")
        if not isinstance(self.fail_on_timeout, bool):
            raise ValueError("MAVLink fail_on_timeout must be a boolean")
