"""Generate ArUco markers, calibration boards and FArUco markers for printing."""

from contextlib import contextmanager
from pathlib import Path

import click
import cv2

from domain.errors import DomainError
from domain.markers import (
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_FRACTAL_TYPE,
    SUPPORTED_ARUCO_DICTIONARY_IDS,
    SUPPORTED_FRACTAL_TYPES,
)
from infrastructure.marker_generation.aruco_generators import (
    ARUCO_DICTIONARIES,
    ArucoBoardGenerator,
    ArucoMarkerGenerationParams,
    ArucoMarkerGenerator,
    MarkerBoardGenerationParams,
)
from infrastructure.marker_generation.faruco_generator import FArUcoMarkerGenerationParams, FArUcoMarkerGenerator
from infrastructure.marker_generation.settings import (
    DEFAULT_BORDER_BITS,
    DEFAULT_FRACTAL_BIT_SIZE,
    DEFAULT_PRINT_DPI,
    DEFAULT_WHITE_MARGIN_M,
    MIN_BORDER_BITS,
    MIN_FRACTAL_BIT_SIZE,
    MIN_PRINT_DPI,
    SUPPORTED_OUTPUT_SUFFIXES,
)


def _output_path(ctx, param, value):
    if value is not None and value.suffix.lower() not in SUPPORTED_OUTPUT_SUFFIXES:
        raise click.BadParameter("Use a .pdf or .png output file")
    return value


def _print_options(command):
    for option in (
        click.option(
            "-l",
            "--length",
            "marker_len_m",
            required=True,
            type=float,
            help="Physical outer marker side length (in meters), excluding the margin.",
        ),
        click.option(
            "--margin",
            "margin_m",
            default=DEFAULT_WHITE_MARGIN_M,
            show_default=True,
            type=float,
            help="White margin around the marker or board (in meters).",
        ),
        click.option(
            "-o",
            "--output",
            required=True,
            type=click.Path(dir_okay=False, path_type=Path),
            callback=_output_path,
            help="Destination PDF or PNG (parent directories are created).",
        ),
    ):
        command = option(command)
    return command


def _aruco_options(command):
    for option in (
        click.option(
            "-d",
            "--dictionary",
            default=DEFAULT_ARUCO_DICTIONARY,
            show_default=True,
            type=click.IntRange(min(SUPPORTED_ARUCO_DICTIONARY_IDS), max(SUPPORTED_ARUCO_DICTIONARY_IDS)),
            help="OpenCV dictionary ID; see the dictionaries command.",
        ),
        click.option("--dpi", default=DEFAULT_PRINT_DPI, show_default=True, type=click.IntRange(min=MIN_PRINT_DPI)),
        click.option(
            "--border-bits", default=DEFAULT_BORDER_BITS, show_default=True, type=click.IntRange(min=MIN_BORDER_BITS)
        ),
    ):
        command = option(command)
    return command


@contextmanager
def _generation_errors():
    try:
        yield
    except (DomainError, ValueError, RuntimeError, OSError, cv2.error) as error:
        raise click.ClickException(str(error)) from error


@click.group(context_settings={"help_option_names": ["-h", "--help"]})
def main():
    """Generate printable markers. Print PDFs at 100% without fitting to the page."""


@main.command()
@click.option("--id", "marker_id", required=True, type=click.IntRange(min=0), help="ArUco marker ID.")
@_print_options
@_aruco_options
def aruco(output, **parameters):
    """Generate one conventional ArUco marker."""
    with _generation_errors():
        path = ArucoMarkerGenerator(ArucoMarkerGenerationParams(**parameters)).generate(output)
    click.echo(f"Marker saved to {path}")


@main.command()
@click.option("-w", "--columns", "num_columns", required=True, type=click.IntRange(min=1))
@click.option("-hm", "--rows", "num_rows", required=True, type=click.IntRange(min=1))
@click.option(
    "-s", "--separation", "separation_m", required=True, type=float, help="Gap between adjacent markers (in meters)."
)
@_print_options
@_aruco_options
def board(output, **parameters):
    """Generate an ArUco GridBoard with IDs starting at zero, for camera calibration."""
    with _generation_errors():
        path = ArucoBoardGenerator(MarkerBoardGenerationParams(**parameters)).generate(output)
    click.echo(f"Board saved to {path}")


@main.command()
@click.option(
    "-t",
    "--fractal-type",
    default=DEFAULT_FRACTAL_TYPE.value,
    show_default=True,
    type=click.Choice(SUPPORTED_FRACTAL_TYPES),
)
@click.option(
    "--bit-size",
    default=DEFAULT_FRACTAL_BIT_SIZE,
    show_default=True,
    type=click.IntRange(min=MIN_FRACTAL_BIT_SIZE),
    help="Pixels per innermost bit cell; independent of the physical print size.",
)
@_print_options
def faruco(output, **parameters):
    """Generate a fractal marker using the optional bridge (built-in ID only)."""
    with _generation_errors():
        path = FArUcoMarkerGenerator(FArUcoMarkerGenerationParams(**parameters)).generate(output)
    click.echo(f"Marker saved to {path}")


@main.command()
def dictionaries():
    """List the ArUco dictionaries supported by generation, tracking and calibration."""
    click.echo("ID  Dictionary             Marker IDs")
    for dictionary_id, name in sorted(ARUCO_DICTIONARIES.items()):
        capacity = cv2.aruco.getPredefinedDictionary(dictionary_id).bytesList.shape[0]
        click.echo(f"{dictionary_id:2}  {name:22} 0..{capacity - 1}")


if __name__ == "__main__":
    main()
