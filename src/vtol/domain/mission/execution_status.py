import time
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MissionExecutionStatus:
    """Latest observed ArduPilot mission progress.

    ``current_item_seq`` identifies the item currently reported by the flight
    controller. ``last_reached_item_seq`` is updated only by
    ``MISSION_ITEM_REACHED`` and therefore has different semantics.
    """

    current_item_seq: int | None = None
    last_reached_item_seq: int | None = None
    last_updated_monotonic_s: float | None = None
    mission_id: int | None = None

    def is_fresh(self, max_age_s: float = 3.0, *, now_s: float | None = None) -> bool:
        if max_age_s <= 0:
            raise ValueError("max_age_s must be greater than zero")
        if self.last_updated_monotonic_s is None:
            return False
        now = time.monotonic() if now_s is None else now_s
        age = now - self.last_updated_monotonic_s
        return 0 <= age <= max_age_s
