from __future__ import annotations

import asyncio
from pathlib import Path

import click
from loguru import logger

from payload.composition import build_payload_application
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from shared.application import run_application


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
        asyncio.run(run_application(application))
    except KeyboardInterrupt:
        logger.info("Payload application interrupted")
    logger.info("Payload application stopped")


if __name__ == "__main__":
    main()
