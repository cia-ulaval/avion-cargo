from __future__ import annotations

import asyncio
from pathlib import Path

import click
from loguru import logger

from payload_manual.application.runtime import ManualControlLimits
from payload.main import run_payload_application
from payload.infrastructure.configuration import PayloadRuntimeConfiguration
from payload_manual.composition import build_manual_control_application


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option(
    "--config",
    "config_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Path to the payload hardware/runtime JSON configuration",
)
@click.option(
    "--controller-peer-id",
    type=click.IntRange(1, 65534),
    required=True,
    help="D3K BLE peer ID of the manual controller node",
)
@click.option(
    "--max-wheel-surface-velocity-m-s",
    type=click.FloatRange(min=0.000001),
    required=True,
    help="Conservative configured maximum wheel surface speed in m/s",
)
@click.option(
    "--max-angular-speed-rad-s",
    type=click.FloatRange(min=0.000001),
    default=0.8,
    show_default=True,
)
@click.option(
    "--command-timeout-ms",
    type=click.IntRange(100, 5000),
    default=300,
    show_default=True,
)
@click.option(
    "--control-rate-hz",
    type=click.IntRange(1, 100),
    default=50,
    show_default=True,
)
@logger.catch(reraise=True)
def main(
    config_path: Path,
    controller_peer_id: int,
    max_wheel_surface_velocity_m_s: float,
    max_angular_speed_rad_s: float,
    command_timeout_ms: int,
    control_rate_hz: int,
) -> None:
    config = PayloadRuntimeConfiguration.from_file(config_path)
    limits = ManualControlLimits(
        max_angular_speed_rad_s=max_angular_speed_rad_s,
        command_timeout_ms=command_timeout_ms,
        control_rate_hz=control_rate_hz,
    )
    application = build_manual_control_application(
        config,
        controller_peer_id=controller_peer_id,
        max_wheel_surface_velocity_m_s=max_wheel_surface_velocity_m_s,
        limits=limits,
    )
    try:
        logger.info(
            "Manual control for payload {} listening to D3K peer {}",
            config.payload_id,
            controller_peer_id,
        )
        asyncio.run(run_payload_application(application))
    except KeyboardInterrupt:
        logger.info("Manual-control application interrupted")
    logger.info("Manual-control application stopped")


if __name__ == "__main__":
    main()
