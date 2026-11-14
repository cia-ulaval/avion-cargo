"""Pure, hardware-free payload mission FSM. Not a physical safety controller."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
import math
import time
from typing import Callable, Mapping


class State(Enum):
    IDLE = auto()
    LOCALIZING = auto()
    CLEARING_LZ = auto()
    READY_FOR_LANDING = auto()
    WAIT_LANDING = auto()
    LOCATE_AIRCRAFT = auto()
    PLAN_APPROACH = auto()
    APPROACH = auto()
    READY_TO_DOCK = auto()
    SAFE_STOP = auto()


class Kind(Enum):
    MISSION_START = auto()
    PAYLOAD_POSE_VALID = auto()
    CLEARANCE_PATH_APPROVED = auto()
    CLEARANCE_EXECUTION_STARTED = auto()
    CLEARANCE_VERIFIED = auto()
    CLEARANCE_ACK = auto()
    LANDING_CONFIRMED = auto()
    AIRCRAFT_POSE_VALID = auto()
    PATH_APPROVED = auto()
    APPROACH_STARTED = auto()
    APPROACH_COMPLETE = auto()
    EMERGENCY_STOP = auto()
    CRITICAL_FAULT = auto()
    RESET_APPROVED = auto()
    SAFETY_INVALIDATED = auto()


class Action(Enum):
    STOP = auto()
    REQUEST_PAYLOAD_POSE = auto()
    REQUEST_CLEARANCE_PATH = auto()
    EXECUTE_CLEARANCE = auto()
    SEND_CLEARANCE_READY = auto()
    REQUEST_AIRCRAFT_POSE = auto()
    REQUEST_APPROACH_PATH = auto()
    EXECUTE_APPROACH = auto()
    REPORT_FAULT = auto()


SAFETY_KINDS = frozenset({Kind.EMERGENCY_STOP, Kind.CRITICAL_FAULT, Kind.SAFETY_INVALIDATED})


@dataclass(frozen=True, slots=True)
class Event:
    kind: Kind
    mission_id: str
    source: str
    sequence: int
    event_id: str
    received_at: float  # receiver-local monotonic time; never remote timestamp
    validated: bool = False  # trusted adapter assertion, NOT verified in this class
    reference: str = ""


@dataclass(frozen=True, slots=True)
class Decision:
    state: State
    actions: tuple[Action, ...]
    accepted: bool
    reason: str = ""
    revision: int = 0  # monotonic transition generation; used to reject stale dispatch


# A transition's second value indicates required trusted external validation.
TRANSITIONS: dict[tuple[State, Kind], tuple[State, bool]] = {
    (State.IDLE, Kind.MISSION_START): (State.LOCALIZING, False),
    (State.LOCALIZING, Kind.PAYLOAD_POSE_VALID): (State.CLEARING_LZ, True),
    (State.CLEARING_LZ, Kind.CLEARANCE_PATH_APPROVED): (State.CLEARING_LZ, True),
    (State.CLEARING_LZ, Kind.CLEARANCE_EXECUTION_STARTED): (State.CLEARING_LZ, True),
    (State.CLEARING_LZ, Kind.CLEARANCE_VERIFIED): (State.READY_FOR_LANDING, True),
    (State.READY_FOR_LANDING, Kind.CLEARANCE_ACK): (State.WAIT_LANDING, True),
    (State.WAIT_LANDING, Kind.LANDING_CONFIRMED): (State.LOCATE_AIRCRAFT, True),
    (State.LOCATE_AIRCRAFT, Kind.AIRCRAFT_POSE_VALID): (State.PLAN_APPROACH, True),
    (State.PLAN_APPROACH, Kind.PATH_APPROVED): (State.APPROACH, True),
    (State.APPROACH, Kind.APPROACH_STARTED): (State.APPROACH, True),
    (State.APPROACH, Kind.APPROACH_COMPLETE): (State.READY_TO_DOCK, True),
    (State.SAFE_STOP, Kind.RESET_APPROVED): (State.IDLE, True),
}

ENTRY_ACTIONS: dict[State, tuple[Action, ...]] = {
    State.IDLE: (Action.STOP,),
    State.LOCALIZING: (Action.STOP, Action.REQUEST_PAYLOAD_POSE),
    State.CLEARING_LZ: (Action.STOP, Action.REQUEST_CLEARANCE_PATH),
    State.READY_FOR_LANDING: (Action.STOP, Action.SEND_CLEARANCE_READY),
    State.WAIT_LANDING: (Action.STOP,),
    State.LOCATE_AIRCRAFT: (Action.STOP, Action.REQUEST_AIRCRAFT_POSE),
    State.PLAN_APPROACH: (Action.STOP, Action.REQUEST_APPROACH_PATH),
    State.APPROACH: (Action.EXECUTE_APPROACH,),
    State.READY_TO_DOCK: (Action.STOP,),
    State.SAFE_STOP: (Action.STOP, Action.REPORT_FAULT),
}


class StateMachine:
    """Single-owner deterministic mission FSM; external adapters own physical proofs.

    Source validation is optional for standalone/demo usage. For any actual integration,
    supply a mapping of trusted sources per event kind; never trust a wire-level bool.
    """

    def __init__(self, *, max_event_age_s: float = 1.0,
                 allowed_sources: Mapping[Kind, frozenset[str]] | None = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if not math.isfinite(max_event_age_s) or max_event_age_s <= 0:
            raise ValueError("max_event_age_s must be finite and positive")
        self.state = State.IDLE
        self.mission_id: str | None = None
        self._last_seq: dict[tuple[str, str], int] = {}
        self._seen_ids: set[tuple[str, str, str]] = set()
        self.history: list[tuple[State, Kind, State]] = []
        self.revision = 0
        self._clearance_execution_requested = False
        self._clearance_execution_started = False
        self._approach_execution_started = False
        self._max_event_age_s = max_event_age_s
        self._allowed_sources = allowed_sources
        self._clock = clock

    def _reject(self, reason: str) -> Decision:
        return Decision(self.state, (), False, reason, self.revision)

    def handle(self, event: Event) -> Decision:
        # Treat safety signals first: stale, malformed, unknown mission must still stop.
        # The receiver's trusted ingress must restrict who can assert a safety signal.
        if isinstance(event.kind, Kind) and event.kind in SAFETY_KINDS:
            return self._transition(event, State.SAFE_STOP)
        if not isinstance(event.kind, Kind):
            return self._reject("unknown event kind")
        if not all(isinstance(s, str) and bool(s.strip()) for s in
                   (event.source, event.event_id, event.mission_id)):
            return self._reject("missing identity")
        if type(event.sequence) is not int or event.sequence < 0:
            return self._reject("invalid sequence")
        if isinstance(event.received_at, bool) or not isinstance(event.received_at, (int, float)) or not math.isfinite(event.received_at):
            return self._reject("invalid receiver timestamp")
        now = self._clock()
        if event.received_at > now or now - event.received_at > self._max_event_age_s:
            return self._reject("stale or future event")
        if self._allowed_sources is not None and event.source not in self._allowed_sources.get(event.kind, frozenset()):
            return self._reject("unauthorized event source")
        dedup_key = (event.mission_id, event.source, event.event_id)
        if dedup_key in self._seen_ids:
            return self._reject("duplicate id")
        seq_key = (event.mission_id, event.source)
        if event.sequence <= self._last_seq.get(seq_key, -1):
            return self._reject("out-of-order sequence")
        if not (self.state is State.IDLE and event.kind is Kind.MISSION_START):
            if self.mission_id is None or event.mission_id != self.mission_id:
                return self._reject("different mission")
        transition = TRANSITIONS.get((self.state, event.kind))
        if transition is None:
            return self._reject("event not applicable in state")
        target, proof_required = transition
        if proof_required and not event.validated:
            return self._reject("external proof missing")
        if event.kind is Kind.CLEARANCE_VERIFIED and not self._clearance_execution_started:
            return self._reject("clearance execution not confirmed")
        if event.kind is Kind.CLEARANCE_EXECUTION_STARTED and not self._clearance_execution_requested:
            return self._reject("clearance was never requested")
        if event.kind is Kind.APPROACH_COMPLETE and not self._approach_execution_started:
            return self._reject("approach execution not confirmed")
        if event.kind is Kind.CLEARANCE_PATH_APPROVED and self._clearance_execution_requested:
            return self._reject("clearance path already accepted")
        if event.kind is Kind.CLEARANCE_EXECUTION_STARTED and self._clearance_execution_started:
            return self._reject("clearance already started")
        if event.kind is Kind.APPROACH_STARTED and self._approach_execution_started:
            return self._reject("approach already started")
        self._seen_ids.add(dedup_key)
        self._last_seq[seq_key] = event.sequence
        if event.kind is Kind.MISSION_START:
            self.mission_id = event.mission_id
        if event.kind is Kind.RESET_APPROVED:
            self.mission_id = None
            self._last_seq.clear()
            self._seen_ids.clear()
            self._clearance_execution_requested = False
            self._clearance_execution_started = False
            self._approach_execution_started = False
        if event.kind is Kind.CLEARANCE_EXECUTION_STARTED:
            self._clearance_execution_started = True
            self.revision += 1
            return Decision(self.state, (), True, "clearance execution acknowledged", self.revision)
        if event.kind is Kind.APPROACH_STARTED:
            self._approach_execution_started = True
            self.revision += 1
            return Decision(self.state, (), True, "approach execution acknowledged", self.revision)
        if event.kind is Kind.CLEARANCE_PATH_APPROVED:
            self._clearance_execution_requested = True
            self.revision += 1
            return Decision(self.state, (Action.EXECUTE_CLEARANCE,), True, "validated clearance path", self.revision)
        return self._transition(event, target)

    def force_safe_stop(self) -> Decision:
        if self.state is State.SAFE_STOP:
            return self._reject("already stopped")
        old = self.state
        self.state = State.SAFE_STOP
        self.revision += 1
        self.history.append((old, Kind.CRITICAL_FAULT, State.SAFE_STOP))
        return Decision(self.state, ENTRY_ACTIONS[self.state], True, "supervisor stop", self.revision)

    def _transition(self, event: Event, target: State) -> Decision:
        if target is State.SAFE_STOP and self.state is State.SAFE_STOP:
            return self._reject("already stopped")
        old = self.state
        self.state = target
        self.revision += 1
        self.history.append((old, event.kind, target))
        if target is State.CLEARING_LZ:
            self._clearance_execution_requested = False
            self._clearance_execution_started = False
        if target is State.APPROACH:
            self._approach_execution_started = False
        return Decision(target, ENTRY_ACTIONS[target], True, "", self.revision)
