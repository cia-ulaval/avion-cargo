from __future__ import annotations

import asyncio
import inspect
import signal
from typing import Any


async def run_application(application: Any) -> None:
    """Run an application until SIGINT/SIGTERM, then close its resources."""
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            # Signal handlers are not available on all event-loop platforms.
            pass

    try:
        await application.run(stop_event)
    finally:
        close_result = application.close()
        if inspect.isawaitable(close_result):
            await close_result
