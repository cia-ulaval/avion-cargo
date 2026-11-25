import asyncio
from types import SimpleNamespace

from payload.domain.positioning.events import DistanceFailure, DistanceSample
from payload.infrastructure.d3kble.distance_sensor import D3kbleDistanceSensor
from tests.d3kble_fakes import FakeNode, fake_measurement


def test_distance_sensor_bridges_valid_measurements_and_range_failures() -> None:
    async def scenario():
        node = FakeNode()
        sensor = D3kbleDistanceSensor(node, {200: "anchor-a", 201: "anchor-b"})
        events = sensor.events()
        first = asyncio.create_task(anext(events))
        await asyncio.sleep(0.02)
        subscription = node.uwb.measurements.subscriptions[0]
        subscription.publish(fake_measurement())
        sample = await asyncio.wait_for(first, timeout=1)
        assert sample == DistanceSample("anchor-a", 1.25, 7)

        second = asyncio.create_task(anext(events))
        await asyncio.sleep(0.01)
        node.emit("range_failed", SimpleNamespace(target_id=201, status=4))
        failure = await asyncio.wait_for(second, timeout=1)
        assert isinstance(failure, DistanceFailure)
        assert failure.anchor_id == "anchor-b"
        assert failure.reason.code == "uwb_range_failed_4"

        await events.aclose()
        assert subscription.closed
        assert node.callbacks.get("range_failed", []) == []

    asyncio.run(scenario())


def test_distance_sensor_converts_invalid_measurements_to_failures() -> None:
    async def scenario():
        node = FakeNode()
        sensor = D3kbleDistanceSensor(node, {200: "anchor-a"})
        events = sensor.events()
        pending = asyncio.create_task(anext(events))
        await asyncio.sleep(0.02)
        node.uwb.measurements.subscriptions[0].publish(fake_measurement(distance_m=None, status=2, quality_flags=16))
        failure = await asyncio.wait_for(pending, timeout=1)
        assert isinstance(failure, DistanceFailure)
        assert failure.reason.code == "uwb_measurement_status_2"
        await events.aclose()

    asyncio.run(scenario())


def test_distance_sensor_start_stop_are_explicit_device_operations() -> None:
    async def scenario():
        node = FakeNode()
        sensor = D3kbleDistanceSensor(node, {200: "anchor-a"})
        await sensor.start_ranging()
        await sensor.stop_ranging()
        assert node.uwb.started == 1
        assert node.uwb.stopped == 1

    asyncio.run(scenario())
