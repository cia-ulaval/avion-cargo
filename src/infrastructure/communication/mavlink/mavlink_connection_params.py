from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class MavlinkConnectionParams:
    address: str
    port: int = 0
    timeout: float = 10.0
    baud_rate: int = 921600

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("MAVLink timeout must be finite and greater than zero")
