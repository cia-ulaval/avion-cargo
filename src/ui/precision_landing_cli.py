import signal
import sys
from pathlib import Path

import click
from loguru import logger

from composition import build_landing_service
from infrastructure.persistence.autolander_configuration_reader import AutolanderConfigurationReader


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("config_file_path", type=click.Path(exists=True))
@click.option(
    "--gz-simulation", default=False, is_flag=True, help="Run simulation using Gazebo Camera", show_default=True
)
@logger.catch(reraise=True)
def main(config_file_path, gz_simulation):
    config_reader = AutolanderConfigurationReader(Path(config_file_path))
    autolander_config = config_reader.read()

    landing_service = build_landing_service(autolander_config, use_simulated_cam=gz_simulation)

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    try:
        logger.info("Configuration loaded from {}", config_file_path)
        landing_service.drone.connect()
        landing_service.track_target()
        landing_service.stream_video()
        landing_service.publish_target_positions()
    finally:
        already_failing = sys.exc_info()[0] is not None
        try:
            landing_service.stop()
        except Exception:
            logger.exception("Landing service cleanup failed")
            if not already_failing:
                raise
        finally:
            signal.signal(signal.SIGTERM, previous_handler)


if __name__ == "__main__":
    main()
