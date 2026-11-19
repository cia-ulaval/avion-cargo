from __future__ import annotations

import argparse
import asyncio
import signal
from pathlib import Path

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
            pass

    await application.run(stop_event)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the VTOL application and its flight-controller/D3K devices")
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to the combined VTOL configuration JSON (autolanding and d3kble)",
    )
    args = parser.parse_args()

    config = VtolApplicationConfiguration.from_file(args.config)
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
