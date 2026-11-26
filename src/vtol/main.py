from __future__ import annotations

import asyncio
from pathlib import Path

import click
from loguru import logger

from shared.application import run_application
from vtol.composition import build_vtol_application
from vtol.infrastructure.persistence.vtol_application_configuration import VtolApplicationConfiguration


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
        asyncio.run(run_application(application))
    except KeyboardInterrupt:
        logger.info("VTOL application interrupted")

    logger.info("VTOL application stopped")


if __name__ == "__main__":
    main()
