"""Regression and adversarial tests identified during audit of V3."""
import math
import threading
import time
import unittest
from dataclasses import replace
from payload_fsm.core import StateMachine, State, Kind, Action, Event, Decision
from payload_fsm.concurrency import EmergencyLatch
from payload_fsm.runtime import ActionMailbox, ActionWorker, PayloadRuntime


def ev(kind, seq, *, src='aircraft', valid=True, mission='m1', timestamp=None):
    return Event(kind, mission, src, seq, f'{src}-{seq}',
                 time.monotonic() if timestamp is None else timestamp, valid)


class AuditFSMTests(unittest.TestCase):
    def test_clearance_cannot_be_verified_without_execution_request(self):
        f = StateMachine()
        f.handle(ev(Kind.MISSION_START, 0))
        f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1))
        self.assertFalse(f.handle(ev(Kind.CLEARANCE_VERIFIED, 2)).accepted)
        result = f.handle(ev(Kind.CLEARANCE_PATH_APPROVED, 3))
        self.assertTrue(result.accepted)
        self.assertEqual(result.actions, (Action.EXECUTE_CLEARANCE,))
        self.assertEqual(f.state, State.CLEARING_LZ)
        self.assertFalse(f.handle(ev(Kind.CLEARANCE_VERIFIED, 4)).accepted)
        self.assertTrue(f.handle(ev(Kind.CLEARANCE_EXECUTION_STARTED, 5)).accepted)
        self.assertTrue(f.handle(ev(Kind.CLEARANCE_VERIFIED, 6)).accepted)
        self.assertEqual(f.state, State.READY_FOR_LANDING)

    def test_no_double_execute_clearance(self):
        f = StateMachine()
        f.handle(ev(Kind.MISSION_START, 0))
        f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1))
        self.assertTrue(f.handle(ev(Kind.CLEARANCE_PATH_APPROVED, 2)).accepted)
        self.assertFalse(f.handle(ev(Kind.CLEARANCE_PATH_APPROVED, 3)).accepted)

    def test_stale_event_rejected_but_stop_accepted(self):
        f = StateMachine(max_event_age_s=.1)
        f.handle(ev(Kind.MISSION_START, 0))
        old = time.monotonic() - 3
        self.assertFalse(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1, timestamp=old)).accepted)
        self.assertTrue(f.handle(ev(Kind.EMERGENCY_STOP, 2, timestamp=old)).accepted)
        self.assertEqual(f.state, State.SAFE_STOP)

    def test_future_time_rejected(self):
        f = StateMachine()
        self.assertFalse(f.handle(ev(Kind.MISSION_START, 0, timestamp=time.monotonic()+10)).accepted)
        self.assertEqual(f.state, State.IDLE)

    def test_invalid_metadata_rejected_without_crashing(self):
        f = StateMachine()
        for value in [None, float('nan'), float('inf'), 'bad', True]:
            self.assertFalse(f.handle(replace(ev(Kind.MISSION_START, 0), received_at=value)).accepted)
        self.assertFalse(f.handle(replace(ev(Kind.MISSION_START, 0), sequence=True)).accepted)
        self.assertTrue(f.handle(ev(Kind.MISSION_START, 0)).accepted)

    def test_source_policy_blocks_unauthorized_events(self):
        f = StateMachine(allowed_sources={Kind.MISSION_START: frozenset({'aircraft'}),
                                                  Kind.PAYLOAD_POSE_VALID: frozenset({'localizer'})})
        self.assertFalse(f.handle(ev(Kind.MISSION_START, 0, src='untrusted')).accepted)
        self.assertTrue(f.handle(ev(Kind.MISSION_START, 0)).accepted)
        self.assertFalse(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1, src='aircraft')).accepted)
        self.assertTrue(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1, src='localizer')).accepted)

    def test_duplicate_event_id_scoped_by_source(self):
        f = StateMachine()
        self.assertTrue(f.handle(ev(Kind.MISSION_START, 0, src='aircraft')).accepted)
        self.assertTrue(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 0, src='localizer')).accepted)

    def test_stale_rejection_does_not_consume_sequence(self):
        f = StateMachine(max_event_age_s=.1)
        f.handle(ev(Kind.MISSION_START, 0))
        self.assertFalse(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1, timestamp=time.monotonic()-10)).accepted)
        self.assertTrue(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1)).accepted)

    def test_fsm_revision_increments_on_every_accepted_transition(self):
        f = StateMachine()
        self.assertEqual(f.handle(ev(Kind.MISSION_START, 0)).revision, 1)
        self.assertEqual(f.handle(ev(Kind.PAYLOAD_POSE_VALID, 1)).revision, 2)
        self.assertEqual(f.handle(ev(Kind.CLEARANCE_PATH_APPROVED, 2)).revision, 3)
        self.assertEqual(f.force_safe_stop().revision, 4)


class AuditConcurrencyTests(unittest.TestCase):
    def test_stale_queued_actions_are_skipped(self):
        latch = EmergencyLatch()
        mailbox = ActionMailbox(latch)
        outputs = []
        mailbox.publish(Decision(State.CLEARING_LZ, (Action.EXECUTE_CLEARANCE,), True, revision=1))
        mailbox.publish(Decision(State.READY_FOR_LANDING, (Action.STOP,), True, revision=2))
        worker = ActionWorker(mailbox, latch, outputs.append)
        worker.start()
        try:
            deadline = time.monotonic()+.5
            while time.monotonic()<deadline and not outputs:
                time.sleep(.001)
            self.assertEqual(len(outputs), 1)
            self.assertEqual(outputs[0].actions, (Action.STOP,))
        finally:
            worker.stop()

    def test_latched_mailbox_never_dispatches_even_if_prequeued(self):
        latch = EmergencyLatch()
        mailbox = ActionMailbox(latch)
        mailbox.publish(Decision(State.APPROACH, (Action.EXECUTE_APPROACH,), True, revision=1))
        latch.trip()
        outputs = []
        worker = ActionWorker(mailbox, latch, outputs.append)
        worker.start()
        worker.stop()
        self.assertEqual(outputs, [])

    def test_optional_communication_silence_watchdog(self):
        from queue import Queue, Empty
        queue = Queue()
        def read():
            try: return queue.get_nowait()
            except Empty: return None
        runtime = PayloadRuntime(communication_poll=read, localization_poll=lambda: None,
                                 dispatch=lambda _ : None, communication_silence_s=.07,
                                 stall_after_s=.5)
        runtime.start()
        try:
            queue.put(ev(Kind.MISSION_START, 0))
            deadline=time.monotonic()+.5
            while time.monotonic()<deadline and runtime.fsm.machine.state is State.IDLE:
                time.sleep(.002)
            self.assertEqual(runtime.fsm.machine.state, State.LOCALIZING)
            deadline=time.monotonic()+.5
            while time.monotonic()<deadline and not runtime.latch.is_tripped():
                time.sleep(.005)
            self.assertTrue(runtime.latch.is_tripped())
        finally:
            runtime.stop()


if __name__ == '__main__':
    unittest.main()
