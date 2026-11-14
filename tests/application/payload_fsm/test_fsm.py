import threading
import time
import unittest
from payload_fsm.core import State, Kind, Action, Event, StateMachine
from payload_fsm.concurrency import EventInbox, EventOverflow, LatestPose, PoseSnapshot, EmergencyLatch, FSMWorker

def ev(kind, seq, *, valid=True, mission="m1", source="avion", eid=None):
    return Event(kind=kind, mission_id=mission, source=source,
                 sequence=seq, event_id=eid or f"{source}-{seq}",
                 received_at=time.monotonic(), validated=valid)

class FSMTests(unittest.TestCase):
    def test_nominal(self):
        m=StateMachine()
        path=[(Kind.MISSION_START,State.LOCALIZING),
              (Kind.PAYLOAD_POSE_VALID,State.CLEARING_LZ),
              (Kind.CLEARANCE_PATH_APPROVED,State.CLEARING_LZ),
              (Kind.CLEARANCE_EXECUTION_STARTED,State.CLEARING_LZ),
              (Kind.CLEARANCE_VERIFIED,State.READY_FOR_LANDING),
              (Kind.CLEARANCE_ACK,State.WAIT_LANDING),
              (Kind.LANDING_CONFIRMED,State.LOCATE_AIRCRAFT),
              (Kind.AIRCRAFT_POSE_VALID,State.PLAN_APPROACH),
              (Kind.PATH_APPROVED,State.APPROACH),
              (Kind.APPROACH_STARTED,State.APPROACH),
              (Kind.APPROACH_COMPLETE,State.READY_TO_DOCK)]
        for i,(kind,expected) in enumerate(path):
            self.assertEqual(m.handle(ev(kind,i)).state,expected)
        self.assertEqual(m.state,State.READY_TO_DOCK)

    def test_no_skipped_clearance(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,0))
        self.assertFalse(m.handle(ev(Kind.LANDING_CONFIRMED,1)).accepted)
        self.assertEqual(m.state,State.LOCALIZING)

    def test_proof_required(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,0))
        self.assertFalse(m.handle(ev(Kind.PAYLOAD_POSE_VALID,1,valid=False)).accepted)
        self.assertEqual(m.state,State.LOCALIZING)

    def test_wrong_mission_and_duplicate(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,0))
        self.assertFalse(m.handle(ev(Kind.PAYLOAD_POSE_VALID,1,mission="other")).accepted)
        p=ev(Kind.PAYLOAD_POSE_VALID,2)
        self.assertTrue(m.handle(p).accepted)
        self.assertFalse(m.handle(p).accepted)

    def test_out_of_order(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,7))
        self.assertFalse(m.handle(ev(Kind.PAYLOAD_POSE_VALID,6)).accepted)
        self.assertEqual(m.state,State.LOCALIZING)

    def test_safe_stop_from_motion(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,0))
        m.handle(ev(Kind.PAYLOAD_POSE_VALID,1))
        self.assertEqual(m.state,State.CLEARING_LZ)
        result=m.handle(ev(Kind.EMERGENCY_STOP,2))
        self.assertEqual(result.state,State.SAFE_STOP)
        self.assertIn(Action.STOP,result.actions)

    def test_stale_stop_is_never_discarded(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,10))
        result=m.handle(ev(Kind.EMERGENCY_STOP,0,mission="unknown"))
        self.assertEqual(result.state,State.SAFE_STOP)

    def test_reset_requires_validation(self):
        m=StateMachine();m.handle(ev(Kind.MISSION_START,0))
        m.handle(ev(Kind.CRITICAL_FAULT,1))
        self.assertFalse(m.handle(ev(Kind.RESET_APPROVED,2,valid=False)).accepted)
        self.assertTrue(m.handle(ev(Kind.RESET_APPROVED,3,valid=True)).accepted)
        self.assertEqual(m.state,State.IDLE)

    def test_inbox_overflow(self):
        q=EventInbox(capacity=1);q.publish(ev(Kind.MISSION_START,0))
        with self.assertRaises(EventOverflow):q.publish(ev(Kind.MISSION_START,1))

    def test_snapshot_coherence(self):
        p=LatestPose()
        self.assertTrue(p.update(PoseSnapshot(.1,.2,None,"DLZ","UWB",1,2)))
        self.assertFalse(p.update(PoseSnapshot(.5,.6,None,"DLZ","UWB",2,1)))
        self.assertEqual(p.read().x,.1)

    def test_multi_producer_thread_run(self):
        q=EventInbox(capacity=1000);latch=EmergencyLatch();decisions=[]
        worker=FSMWorker(q,latch,output=decisions.append)
        worker.start()
        q.publish(ev(Kind.MISSION_START,0,source="start"))
        def produce(source):
            for i in range(100):
                q.publish(ev(Kind.LANDING_CONFIRMED,i,source=source))
        threads=[threading.Thread(target=produce,args=(f"p{x}",)) for x in range(3)]
        for t in threads:t.start()
        for t in threads:t.join()
        deadline=time.monotonic()+1
        while time.monotonic()<deadline and worker.machine.state is State.IDLE:
            time.sleep(.005)
        worker.stop()
        self.assertFalse(worker.alive)
        self.assertIsNone(worker.failure)
        self.assertIn(State.LOCALIZING,[d.state for d in decisions])

    def test_out_of_band_latch(self):
        q=EventInbox(capacity=1);latch=EmergencyLatch()
        worker=FSMWorker(q,latch)
        worker.start();latch.trip()
        deadline=time.monotonic()+1
        while time.monotonic()<deadline and worker.machine.state is not State.SAFE_STOP:
            time.sleep(.005)
        self.assertEqual(worker.machine.state,State.SAFE_STOP)
        worker.stop()

if __name__=="__main__":unittest.main()
