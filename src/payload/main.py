from __future__ import annotations

import argparse
import asyncio
import signal
from pathlib import Path

from loguru import logger

from payload.composition import PayloadApplication, build_payload_application
from payload.infrastructure.configuration import PayloadRuntimeConfiguration


async def _run_application(application: PayloadApplication) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            # Signal handlers are not available on all event-loop platforms.
            pass

    await application.run(stop_event)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run payload D3Kble communication and UWB ranging")
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to the payload hardware/runtime JSON configuration",
    )
    args = parser.parse_args()

    config = PayloadRuntimeConfiguration.from_file(args.config)
    application = build_payload_application(config)
    try:
        logger.info("Payload {} started; connecting to plane BLE peer {}", config.payload_id, config.plane_peer_id)
        asyncio.run(_run_application(application))
    except KeyboardInterrupt:
        logger.info("Payload application interrupted")
    finally:
        application.close()
        logger.info("Payload application stopped")


if __name__ == "__main__":
    main()
