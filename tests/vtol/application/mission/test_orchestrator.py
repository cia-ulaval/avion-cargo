import asyncio

from vtol.application.mission.context import MissionContext
from vtol.application.mission.decisions import MissionProgressStatus
from vtol.application.mission.observation_store import MissionObservationStore
from vtol.application.mission.orchestrator import MissionOrchestrator
from vtol.application.mission.states import (
    IdleState,
    PayloadTransferState,
    PlaneLandingState,
    PlaneTakeoffState,
    VtolLandState,
    VtolTakeoffState,
)
from vtol.domain.mission.flight_section import FlightSection


def test_orchestrator_starts_idle_and_keeps_transition_logic_deferred():
    async def scenario():
        observations = MissionObservationStore()
        orchestrator = MissionOrchestrator(observations.capture_mission_snapshot)

        assert orchestrator.current_state_name == "idle"
        progress = await orchestrator.step()
        assert progress.status is MissionProgressStatus.IDLE
        assert progress.section is None

        # Changes in incoming observations do not cause transitions in this scaffold PR.
        observations.set_flight_observation(None, observations.capture_mission_snapshot().mission_execution_status)
        for _ in range(3):
            progress = await orchestrator.step()
            assert progress.status is MissionProgressStatus.IDLE
            assert orchestrator.current_state_name == "idle"

        await orchestrator._machine.close(observations.capture_mission_snapshot())

    asyncio.run(scenario())


def test_flight_phase_state_skeletons_report_their_phase_and_do_not_transition():
    states = (
        (PlaneTakeoffState(), FlightSection.PLANE_TAKEOFF),
        (VtolLandState(), FlightSection.VTOL_LAND),
        (PayloadTransferState(), FlightSection.PAYLOAD_TRANSFER),
        (VtolTakeoffState(), FlightSection.VTOL_TAKEOFF),
        (PlaneLandingState(), FlightSection.PLANE_LANDING),
    )

    async def scenario():
        context = MissionContext()
        for state, section in states:
            progress = await state.exec(context)
            assert progress.section is section
            assert progress.status is MissionProgressStatus.ACTIVE
            assert state.next(context) is state

        idle = IdleState()
        assert (await idle.exec(context)).status is MissionProgressStatus.IDLE
        assert idle.next(context) is idle

    asyncio.run(scenario())


def test_run_refreshes_context_provider_until_stopped():
    observations = MissionObservationStore()
    orchestrator = MissionOrchestrator(observations.capture_mission_snapshot)

    async def scenario():
        stop_event = asyncio.Event()
        task = asyncio.create_task(orchestrator.run(stop_event, tick_interval=0.01))
        await asyncio.sleep(0.05)
        stop_event.set()
        await asyncio.wait_for(task, timeout=1)

    asyncio.run(scenario())
