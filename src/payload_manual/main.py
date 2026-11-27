from __future__ import annotations

import asyncio
from pathlib import Path

import click
from loguru import logger

from payload_manual.application.runtime import ManualControlLimits
from payload_manual.composition import build_manual_control_application
from payload_manual.infrastructure.configuration import ManualPayloadRuntimeConfiguration
from shared.application import run_application


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "--config",
    "config_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Path to the payload hardware/runtime JSON configuration",
)
@click.option(
    "--command-timeout-ms",
    type=click.IntRange(100, 5000),
    default=1000,
    show_default=True,
)
@logger.catch(reraise=True)
def main(
    config_path: Path,
    command_timeout_ms: int,
) -> None:
    config = ManualPayloadRuntimeConfiguration.from_file(config_path)
    limits = ManualControlLimits(
        command_timeout_ms=command_timeout_ms,
    )
    application = build_manual_control_application(
        config,
        limits=limits,
    )
    try:
        asyncio.run(run_application(application))
    except KeyboardInterrupt:
        logger.info("Manual-control application interrupted")
    logger.info("Manual-control application stopped")


if __name__ == "__main__":
    main()
