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
@logger.catch
def main(config_file_path, gz_simulation):
    config_reader = AutolanderConfigurationReader(Path(config_file_path))
    autolander_config = config_reader.read()

    landing_service = build_landing_service(autolander_config, use_simulated_cam=gz_simulation)
    landing_service.drone.connect()
    landing_service.track_target()
    landing_service.stream_video()
    landing_service.perform_precision_landing()
    landing_service.stop()


if __name__ == "__main__":
    main()
