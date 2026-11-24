import asyncio

import pytest

from payload.main import run_payload_application


class FakeSyncApplication:
    def __init__(self):
        self.closed = False

    async def run(self, stop_event):
        stop_event.set()

    def close(self):
        self.closed = True


class FakeAsyncApplication:
    def __init__(self):
        self.closed = False

    async def run(self, stop_event):
        stop_event.set()

    async def close(self):
        self.closed = True


@pytest.mark.asyncio
async def test_shared_entrypoint_closes_application_with_sync_close():
    application = FakeSyncApplication()

    await run_payload_application(application)

    assert application.closed


@pytest.mark.asyncio
async def test_shared_entrypoint_awaits_async_close():
    application = FakeAsyncApplication()

    await run_payload_application(application)

    assert application.closed
