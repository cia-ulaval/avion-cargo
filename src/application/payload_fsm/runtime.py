"""Replaceable threaded adapters and supervision for a *software-only* prototype.

No GPIO, Bluetooth, DWM3000, ArduPilot, or motor drivers are imported here.
All motor/landing permissions require external, independently validated proofs.
"""
from __future__ import annotations

from dataclasses import dataclass
from queue import Queue, Empty, Full
from threading import Event as ThreadEvent, Lock, Thread
import time
from typing import Callable

from .core import Action, Decision, Event, Kind, StateMachine
from .concurrency import EmergencyLatch, EventInbox, EventOverflow, FSMWorker


class ActionOverflow(RuntimeError):
    pass


class ActionMailbox:
    """Bounded intent mailbox; overflow TRIPS latch, never silently drops."""
    def __init__(self, latch: EmergencyLatch, capacity: int = 64):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.latch = latch
        self._queue: Queue[Decision] = Queue(maxsize=capacity)
        self._lock = Lock()
        self._latest_revision = -1

    def publish(self, decision: Decision) -> None:
        if self.latch.is_tripped():
            return  # after a fault the only safe effect is inhibit, not new commands
        try:
            self._queue.put_nowait(decision)
            with self._lock:
                self._latest_revision = max(self._latest_revision, decision.revision)
        except Full as exc:
            self.latch.trip()
            raise ActionOverflow("action mailbox full; safety latch tripped") from exc

    def is_current(self, decision: Decision) -> bool:
        # Old queued intents must not be sent after a newer FSM transition.
        with self._lock:
            return decision.revision == self._latest_revision

    def receive(self, timeout: float = .02) -> Decision | None:
        try:
            return self._queue.get(timeout=timeout)
        except Empty:
            return None


class WorkerThread:
    """Single reader/poller behind a replaceable adapter.

    `poll` must be bounded and return None when no update is available.
    It must not hold shared locks during blocking I/O. Failure trips latch.
    """
    def __init__(self, name: str, poll: Callable[[], Event | None],
                 inbox: EventInbox, latch: EmergencyLatch, *, idle_s: float = .005):
        self.name, self.poll, self.inbox, self.latch = name, poll, inbox, latch
        self.idle_s = idle_s
        self.failure: BaseException | None = None
        self._stop = ThreadEvent()
        self._thread: Thread | None = None
        self._lock = Lock()
        self._progress = time.monotonic()
        self._last_data: float | None = None

    @property
    def last_data(self) -> float | None:
        with self._lock:
            return self._last_data

    @property
    def last_progress(self) -> float:
        with self._lock:
            return self._progress

    def _mark_progress(self) -> None:
        with self._lock:
            self._progress = time.monotonic()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("worker already started")
        self._mark_progress()
        self._thread = Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set() and not self.latch.is_tripped():
                event = self.poll()
                self._mark_progress()  # successful poll, even when it returned None
                if event is not None:
                    with self._lock:
                        self._last_data = time.monotonic()
                    # Safety signals bypass a potentially full or backlogged FIFO.
                    if event.kind in (Kind.EMERGENCY_STOP, Kind.CRITICAL_FAULT, Kind.SAFETY_INVALIDATED):
                        self.latch.trip()
                        continue
                    try:
                        self.inbox.publish(event)
                    except EventOverflow:
                        self.latch.trip()
                        raise
                else:
                    self._stop.wait(self.idle_s)
        except BaseException as exc:
            self.failure = exc
            self.latch.trip()

    def stop(self, timeout: float = 1) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise TimeoutError(f"{self.name} did not stop; adapter may be blocked")


class ActionWorker:
    """Executes intents outside FSM thread. Callback must be bounded.

    An independent motor interlock must observe the latch directly. A Python
    callback blocked in I/O cannot be forcibly interrupted by threading.
    """
    def __init__(self, mailbox: ActionMailbox, latch: EmergencyLatch,
                 dispatch: Callable[[Decision], None]):
        self.mailbox, self.latch, self.dispatch = mailbox, latch, dispatch
        self._stop = ThreadEvent()
        self._thread: Thread | None = None
        self.failure: BaseException | None = None
        self._lock = Lock()
        self._progress = time.monotonic()

    @property
    def last_progress(self) -> float:
        with self._lock:
            return self._progress

    def _mark_progress(self) -> None:
        with self._lock:
            self._progress = time.monotonic()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("worker already started")
        self._mark_progress()
        self._thread = Thread(target=self._run, name="payload-actions", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set() and not self.latch.is_tripped():
                decision = self.mailbox.receive()
                self._mark_progress()
                if decision is None:
                    continue
                if self.latch.is_tripped():
                    break
                if not self.mailbox.is_current(decision):
                    continue  # a newer state invalidated this queued intent
                self.dispatch(decision)
                self._mark_progress()
        except BaseException as exc:
            self.failure = exc
            self.latch.trip()

    def stop(self, timeout: float = 1) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise TimeoutError("action worker did not stop; dispatcher may be blocked")


@dataclass(frozen=True)
class RuntimeStatus:
    faulted: bool
    reasons: tuple[str, ...]


class PayloadRuntime:
    """Orchestrates 2 input workers + unique FSM owner + action worker.

    The two inputs are optional callable adapters: one can read ESP32 events,
    the other already-processed localization events. Neither implements UWB.
    No restart in a faulted session; construct a new runtime after review.
    """
    def __init__(self, *, communication_poll: Callable[[], Event | None],
                 localization_poll: Callable[[], Event | None],
                 dispatch: Callable[[Decision], None],
                 event_capacity: int = 128, action_capacity: int = 64,
                 stall_after_s: float = 1.0,
                 max_event_age_s: float = 1.0,
                 allowed_sources=None,
                 communication_silence_s: float | None = None):
        if stall_after_s <= 0:
            raise ValueError("stall_after_s must be > 0")
        if communication_silence_s is not None and communication_silence_s <= 0:
            raise ValueError("communication_silence_s must be > 0")
        self.latch = EmergencyLatch()
        self.inbox = EventInbox(event_capacity)
        self.mailbox = ActionMailbox(self.latch, action_capacity)
        self.fsm = FSMWorker(self.inbox, self.latch, output=self.mailbox.publish,
                             machine=StateMachine(max_event_age_s=max_event_age_s,
                                                  allowed_sources=allowed_sources))
        self.inputs = [WorkerThread("payload-communication", communication_poll, self.inbox, self.latch),
                       WorkerThread("payload-localization", localization_poll, self.inbox, self.latch)]
        self.actions = ActionWorker(self.mailbox, self.latch, dispatch)
        self._stall_after_s = stall_after_s
        self._communication_silence_s = communication_silence_s
        self._started = False
        self._supervisor_stop = ThreadEvent()
        self._supervisor: Thread | None = None
        self.last_status = RuntimeStatus(False, ())
        self._stopping = False

    def start(self) -> None:
        if self._started:
            raise RuntimeError("runtime cannot restart; use a new instance")
        self._started = True
        # Actions start before FSM; no adapter can move hardware without an
        # independently enforced command-expiration / safety contract.
        self.actions.start()
        self.fsm.start()
        for worker in self.inputs:
            worker.start()
        self._supervisor = Thread(target=self._supervise, name="payload-supervisor", daemon=True)
        self._supervisor.start()

    def _supervise(self) -> None:
        while not self._supervisor_stop.wait(.02):
            self.last_status = self.inspect()
            if self.last_status.faulted:
                break

    def inspect(self) -> RuntimeStatus:
        if not self._started or self._stopping:
            return RuntimeStatus(self.latch.is_tripped(), ())
        reasons = []
        now = time.monotonic()
        for w in [*self.inputs, self.actions]:
            if w.failure is not None:
                reasons.append(f"{w.name if hasattr(w, 'name') else 'actions'}: exception {type(w.failure).__name__}")
            elif not w.alive:
                reasons.append(f"{w.name if hasattr(w, 'name') else 'actions'}: not alive")
            elif now - w.last_progress > self._stall_after_s:
                reasons.append(f"{w.name if hasattr(w, 'name') else 'actions'}: stalled")
        if (self._communication_silence_s is not None and
                self.fsm.machine.state.name not in ("IDLE", "SAFE_STOP") and
                self.inputs[0].last_data is not None and
                now - self.inputs[0].last_data > self._communication_silence_s):
            reasons.append("communication: no new messages within configured limit")
        if self.fsm.failure is not None:
            reasons.append(f"fsm: exception {type(self.fsm.failure).__name__}")
        elif not self.fsm.alive:
            reasons.append("fsm: not alive")
        elif now - self.fsm.last_progress > self._stall_after_s:
            reasons.append("fsm: stalled")
        if reasons:
            self.latch.trip()
        return RuntimeStatus(self.latch.is_tripped(), tuple(reasons))

    def stop(self, timeout: float = 1.0) -> None:
        self._stopping = True
        self.latch.trip()  # always inhibit before joining worker threads
        self._supervisor_stop.set()
        errors = []
        if self._supervisor is not None:
            self._supervisor.join(timeout)
            if self._supervisor.is_alive():
                errors.append("supervisor did not stop")
        for w in [*self.inputs, self.actions, self.fsm]:
            try:
                w.stop(timeout)
            except TimeoutError as exc:
                errors.append(str(exc))
        if errors:
            raise TimeoutError("; ".join(errors))
