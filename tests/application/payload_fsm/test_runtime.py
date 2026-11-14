import threading
import time
import unittest
from queue import Empty, Queue

from payload_fsm.core import Action, Event, Kind, State, StateMachine
from payload_fsm.concurrency import EmergencyLatch, EventInbox
from payload_fsm.runtime import ActionMailbox, ActionOverflow, PayloadRuntime, WorkerThread


def event(kind, sequence, *, source="aircraft", valid=True, eid=None):
    return Event(kind, "mission-1", source, sequence, eid or f"{source}-{sequence}", time.monotonic(), valid)


def wait_until(predicate, timeout=.8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.002)
    return predicate()


class TransitionRegression(unittest.TestCase):
    def test_invalid_proof_does_not_consume_sequence_or_event_id(self):
        machine = StateMachine()
        machine.handle(event(Kind.MISSION_START, 0))
        p = event(Kind.PAYLOAD_POSE_VALID, 1)
        rejected = Event(p.kind, p.mission_id, p.source, p.sequence, p.event_id, p.received_at, False)
        self.assertFalse(machine.handle(rejected).accepted)
        self.assertTrue(machine.handle(p).accepted)
        self.assertEqual(machine.state, State.CLEARING_LZ)

    def test_wrong_state_does_not_poison_sequence(self):
        machine = StateMachine()
        machine.handle(event(Kind.MISSION_START, 0))
        self.assertFalse(machine.handle(event(Kind.LANDING_CONFIRMED, 8)).accepted)
        self.assertTrue(machine.handle(event(Kind.PAYLOAD_POSE_VALID, 1)).accepted)

    def test_safety_invalidated_bypasses_sequence(self):
        machine = StateMachine()
        machine.handle(event(Kind.MISSION_START, 10))
        self.assertEqual(machine.handle(event(Kind.SAFETY_INVALIDATED, 0, source="other")).state, State.SAFE_STOP)


class RuntimeTests(unittest.TestCase):
    def test_worker_poll_error_trips_latch(self):
        latch = EmergencyLatch()
        def fail():
            raise OSError("adapter disconnected")
        w = WorkerThread("comms", fail, EventInbox(), latch)
        w.start()
        try:
            self.assertTrue(wait_until(latch.is_tripped))
            self.assertIsInstance(w.failure, OSError)
        finally:
            w.stop()

    def test_event_overflow_trips_latch(self):
        latch = EmergencyLatch()
        inbox = EventInbox(1)
        inbox.publish(event(Kind.MISSION_START, 0))
        w = WorkerThread("producer", lambda: event(Kind.MISSION_START, 1), inbox, latch)
        w.start()
        try:
            self.assertTrue(wait_until(latch.is_tripped))
            self.assertIsNotNone(w.failure)
        finally:
            w.stop()

    def test_action_overflow_trips_latch(self):
        latch = EmergencyLatch()
        mb = ActionMailbox(latch, 1)
        from payload_fsm.core import Decision
        d = Decision(State.IDLE, (Action.STOP,), True)
        mb.publish(d)
        with self.assertRaises(ActionOverflow):
            mb.publish(d)
        self.assertTrue(latch.is_tripped())

    def test_runtime_nominal_and_clean_shutdown(self):
        comms = Queue()
        localization = Queue()
        outputs = []
        def poll(q):
            try:
                return q.get_nowait()
            except Empty:
                return None
        rt = PayloadRuntime(communication_poll=lambda: poll(comms),
                            localization_poll=lambda: poll(localization),
                            dispatch=outputs.append, stall_after_s=.4)
        rt.start()
        try:
            comms.put(event(Kind.MISSION_START, 0, source="aircraft"))
            self.assertTrue(wait_until(lambda: rt.fsm.machine.state is State.LOCALIZING))
            localization.put(event(Kind.PAYLOAD_POSE_VALID, 0, source="localization"))
            self.assertTrue(wait_until(lambda: rt.fsm.machine.state is State.CLEARING_LZ))
            self.assertTrue(wait_until(lambda: len(outputs) >= 2))
            self.assertFalse(rt.inspect().faulted)
        finally:
            rt.stop()
        self.assertTrue(rt.latch.is_tripped())
        self.assertFalse(any(w.alive for w in rt.inputs))
        self.assertFalse(rt.actions.alive)
        self.assertFalse(rt.fsm.alive)

    def test_blocked_dispatch_does_not_block_fsm_and_supervisor_detects(self):
        unblock = threading.Event()
        called = threading.Event()
        comms = Queue()
        def dispatch(_decision):
            called.set()
            unblock.wait(1)
        def poll():
            try:
                return comms.get_nowait()
            except Empty:
                return None
        rt = PayloadRuntime(communication_poll=poll, localization_poll=lambda: None,
                            dispatch=dispatch, stall_after_s=.06)
        rt.start()
        try:
            comms.put(event(Kind.MISSION_START, 0))
            self.assertTrue(called.wait(.5))
            comms.put(event(Kind.PAYLOAD_POSE_VALID, 1))
            self.assertTrue(wait_until(lambda: rt.fsm.machine.state is State.CLEARING_LZ))
            self.assertTrue(wait_until(rt.latch.is_tripped, timeout=.6))
            self.assertTrue(rt.actions.alive or rt.actions.failure is not None)
        finally:
            unblock.set()
            rt.stop()

    def test_safety_signal_bypasses_full_fifo(self):
        latch = EmergencyLatch()
        inbox = EventInbox(1)
        inbox.publish(event(Kind.MISSION_START, 0))
        w = WorkerThread("safety-receiver", lambda: event(Kind.SAFETY_INVALIDATED, 1), inbox, latch)
        w.start()
        try:
            self.assertTrue(wait_until(latch.is_tripped))
            self.assertIsNone(w.failure)
        finally:
            w.stop()

    def test_restart_rejected(self):
        rt = PayloadRuntime(communication_poll=lambda: None, localization_poll=lambda: None,
                            dispatch=lambda _: None)
        rt.start()
        rt.stop()
        with self.assertRaises(RuntimeError):
            rt.start()


if __name__ == "__main__":
    unittest.main()
