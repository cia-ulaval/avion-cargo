"""Thread-safe transport primitives; fail-closed demonstration only."""
from __future__ import annotations
from dataclasses import dataclass
from queue import Queue, Empty, Full
from threading import Event as ThreadEvent, Lock, Thread
import time
from typing import Callable
from .core import Action, Decision, Event, Kind, State, StateMachine

class EventOverflow(RuntimeError):
    pass

class EventInbox:
    def __init__(self, capacity: int = 128):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._queue: Queue[Event] = Queue(maxsize=capacity)

    def publish(self, event: Event) -> None:
        try:
            self._queue.put_nowait(event)
        except Full as exc:
            raise EventOverflow("critical event inbox is full") from exc

    def receive(self, timeout: float = .05) -> Event | None:
        try:
            return self._queue.get(timeout=timeout)
        except Empty:
            return None

@dataclass(frozen=True, slots=True)
class PoseSnapshot:
    x: float
    y: float
    heading_rad: float | None
    frame: str
    source: str
    received_at: float
    version: int

class LatestPose:
    """Coherent snapshot; timestamps are RECEIVER monotonic times."""
    def __init__(self) -> None:
        self._lock = Lock()
        self._current: PoseSnapshot | None = None

    def update(self, pose: PoseSnapshot) -> bool:
        with self._lock:
            if self._current is not None and pose.version <= self._current.version:
                return False
            self._current = pose
            return True

    def read(self) -> PoseSnapshot | None:
        with self._lock:
            return self._current

class EmergencyLatch:
    """Out-of-band software latch; NOT a physical motor interlock."""
    def __init__(self) -> None:
        self._triggered = ThreadEvent()

    def trip(self) -> None:
        self._triggered.set()

    def is_tripped(self) -> bool:
        return self._triggered.is_set()

    def clear_for_new_session(self) -> None:
        # Never call this automatically after a fault.
        self._triggered.clear()

class FSMWorker:
    """Only this worker thread mutates StateMachine in normal operation.

    EmergencyLatch can immediately instruct a separate downstream adapter to
    inhibit motion; local StateMachine transition follows on its next poll.
    """
    def __init__(self, inbox: EventInbox, latch: EmergencyLatch,
                 output: Callable[[Decision], None] | None = None,
                 machine: StateMachine | None = None):
        self.inbox = inbox
        self.latch = latch
        self.machine = machine if machine is not None else StateMachine()
        self.output = output or (lambda decision: None)
        self._shutdown = ThreadEvent()
        self._thread: Thread | None = None
        self.failure: BaseException | None = None
        self._progress_lock = Lock()
        self._progress = time.monotonic()

    @property
    def last_progress(self) -> float:
        with self._progress_lock:
            return self._progress

    def _mark_progress(self) -> None:
        with self._progress_lock:
            self._progress = time.monotonic()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("worker already started")
        self._thread = Thread(target=self._run, name="payload-fsm", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._shutdown.is_set():
                self._mark_progress()
                if self.latch.is_tripped():
                    if self.machine.state is not State.SAFE_STOP:
                        self.output(self.machine.force_safe_stop())
                    # Discard regular events while fault latched; no automatic reset.
                    self._shutdown.wait(.01)
                    continue
                event = self.inbox.receive(timeout=.02)
                if event is not None:
                    if event.kind in (Kind.EMERGENCY_STOP, Kind.CRITICAL_FAULT, Kind.SAFETY_INVALIDATED):
                        self.latch.trip()
                        continue
                    decision = self.machine.handle(event)
                    if decision.accepted and decision.actions:
                        self.output(decision)
        except BaseException as exc:
            self.failure = exc
            self.latch.trip()
        finally:
            self.latch.trip()  # no motion after worker exits

    def stop(self, timeout: float = 2.0) -> None:
        self.latch.trip()
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise TimeoutError("FSM worker did not stop")

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
