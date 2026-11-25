from __future__ import annotations

import asyncio
import inspect
import signal
from pathlib import Path
from typing import Any

import click
from loguru import logger

from payload.composition import build_payload_application
from payload.infrastructure.configuration import PayloadRuntimeConfiguration


async def run_payload_application(application: Any) -> None:
    """
    Run a payload application until SIGINT/SIGTERM, then close it.
    """
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


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "--config",
    "config_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Path to the payload hardware/runtime JSON configuration",
)
@logger.catch(reraise=True)
def main(config_path: Path) -> None:
    config = PayloadRuntimeConfiguration.from_file(config_path)
    application = build_payload_application(config)
    logger.info(
        "Payload {} started; connecting to plane BLE peer {}",
        config.payload_id,
        config.plane_peer_id,
    )
    try:
        asyncio.run(run_payload_application(application))
    except KeyboardInterrupt:
        logger.info("Payload application interrupted")
    logger.info("Payload application stopped")


if __name__ == "__main__":
    main()
