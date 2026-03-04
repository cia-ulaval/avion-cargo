import click
from loguru import logger

@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("cam_calibration_file", type=click.Path(exists=True))
@click.option("-l", "marker_length", required=True, type=float, help="Marker length (meters)")
@click.option(
    "-d", "dictionary_id", required=True, default=16, show_default=True, type=int, help="Dictionary id (0..16)"
)
@click.option("-mid", "marker_id", default=None, show_default=True, type=int, help="Marker id")
@click.option("--inner_marker_id", "inner_marker_id", default=None, show_default=True, type=int, help="Inner marker id")
@click.option("--outer-marker-id", "outer_marker_id", default=None, show_default=True, type=int, help="Outer marker id")
@click.option("--picam", is_flag=True, help="Use PiCamera2 (Raspberry Pi)")
@click.option("--cam-id", default=0, show_default=True, type=int, help="Webcam id. Not necessary if using Picamera2.")
@click.option("--width", default=640, show_default=True, type=int)
@click.option("--height", default=480, show_default=True, type=int)
@click.option("--fps", default=30, show_default=True, type=int)
@click.option("--address", default="/dev/serial0", show_default=True, type=str, help="Mavlink connection address")
@click.option("--port", default=14560, show_default=True, type=int, help="Mavlink connection port, required for UDP")
@logger.catch
def main():
    pass