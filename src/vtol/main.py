from __future__ import annotations

import asyncio
import signal
from pathlib import Path

import click
from loguru import logger

from vtol.composition import VtolApplication, build_vtol_application
from vtol.infrastructure.persistence.vtol_application_configuration import VtolApplicationConfiguration


async def _run_application(application: VtolApplication) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            # Signal handlers are not available on all event-loop platforms.
            pass

    await application.run(stop_event)


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "--config",
    "config_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Path to the combined VTOL configuration JSON (autolanding and d3kble)",
)
@logger.catch(reraise=True)
def main(config_path: Path) -> None:
    config = VtolApplicationConfiguration.from_file(config_path)
    application = build_vtol_application(config)
    try:
        logger.info(
            "VTOL application started with {} D3K device(s), {} payload peer(s), and MAVLink telemetry",
            len(application.nodes),
            len(application.payload_peers),
        )
        asyncio.run(_run_application(application))
    except KeyboardInterrupt:
        logger.info("VTOL application interrupted")
    finally:
        application.close()
        logger.info("VTOL application stopped")


if __name__ == "__main__":
    main()
