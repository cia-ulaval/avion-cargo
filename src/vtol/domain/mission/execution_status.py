import time
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MissionExecutionStatus:
    """
    Latest observed ArduPilot mission progress.
    """

    current_item_seq: int | None = None
    last_reached_item_seq: int | None = None
    last_updated_monotonic_s: float | None = None
    mission_id: int | None = None
