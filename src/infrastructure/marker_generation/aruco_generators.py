"""ArUco marker and calibration board generators adapted from feat/add-markers-generators."""

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from domain.marker_generators import MarkerGenerator
from domain.models import TargetedMarker
from infrastructure.marker_generation.image_writer import (
    MarkerImageWriter,
    meters_to_pixels,
    validate_integer,
    validate_margin,
)

# Keep the generator aligned with the 0..16 dictionaries accepted by tracking
# and camera calibration, even when OpenCV also exposes AprilTag dictionaries.
ARUCO_DICTIONARIES = {
    value: name
    for name, value in vars(cv2.aruco).items()
    if name.startswith("DICT_") and isinstance(value, int) and 0 <= value <= 16
}


@dataclass(frozen=True, slots=True)
class ArucoMarkerGenerationParams:
    marker_id: int
    marker_len_m: float
    dictionary: int = 16
    border_bits: int = 1
    dpi: int = 300
    margin_m: float = 0.01

    def __post_init__(self) -> None:
        validate_integer("marker_id", self.marker_id, 0)
        TargetedMarker(id=self.marker_id, length=self.marker_len_m, dictionary=self.dictionary)
        validate_integer("border_bits", self.border_bits, 1)
        validate_integer("dpi", self.dpi, 150)
        validate_margin(self.margin_m)


class ArucoMarkerGenerator(MarkerGenerator):
    def __init__(self, generation_params: ArucoMarkerGenerationParams) -> None:
        self._p = generation_params

    def generate(self, output: Path) -> Path:
        p = self._p
        dictionary = cv2.aruco.getPredefinedDictionary(p.dictionary)
        side_px = meters_to_pixels(p.marker_len_m, p.dpi)
        if side_px < dictionary.markerSize + 2 * p.border_bits:
            raise ValueError("Marker is too small at this DPI; increase marker length or DPI")
        image = cv2.aruco.generateImageMarker(dictionary, p.marker_id, side_px, borderBits=p.border_bits)
        return MarkerImageWriter.save(image, output, p.marker_len_m, p.marker_len_m, p.margin_m)


@dataclass(frozen=True, slots=True)
class MarkerBoardGenerationParams:
    num_columns: int
    num_rows: int
    marker_len_m: float
    separation_m: float
    dictionary: int = 16
    margin_m: float = 0.01
    border_bits: int = 1
    dpi: int = 300

    def __post_init__(self) -> None:
        validate_integer("num_columns", self.num_columns, 1)
        validate_integer("num_rows", self.num_rows, 1)
        # GridBoard uses consecutive IDs starting at zero, like the calibrator.
        ArucoMarkerGenerationParams(
            marker_id=self.num_columns * self.num_rows - 1,
            marker_len_m=self.marker_len_m,
            dictionary=self.dictionary,
            margin_m=self.margin_m,
            border_bits=self.border_bits,
            dpi=self.dpi,
        )
        if not math.isfinite(self.separation_m) or self.separation_m <= 0:
            raise ValueError("separation_m must be finite and > 0")


class ArucoBoardGenerator(MarkerGenerator):
    def __init__(self, generation_params: MarkerBoardGenerationParams) -> None:
        self._p = generation_params

    def generate(self, output: Path) -> Path:
        p = self._p
        dictionary = cv2.aruco.getPredefinedDictionary(p.dictionary)
        marker_px = meters_to_pixels(p.marker_len_m, p.dpi)
        separation_px = meters_to_pixels(p.separation_m, p.dpi)
        if marker_px < dictionary.markerSize + 2 * p.border_bits or separation_px < 1:
            raise ValueError("Markers or separation are too small at this DPI; increase dimensions or DPI")
        width_px = p.num_columns * marker_px + (p.num_columns - 1) * separation_px
        height_px = p.num_rows * marker_px + (p.num_rows - 1) * separation_px
        # Assemble explicitly so each marker has the same number of pixels and
        # a nonsquare board is never stretched by OpenCV's image fitting.
        image = np.full((height_px, width_px), 255, dtype=np.uint8)
        for row in range(p.num_rows):
            for column in range(p.num_columns):
                x = column * (marker_px + separation_px)
                y = row * (marker_px + separation_px)
                marker_id = row * p.num_columns + column
                image[y : y + marker_px, x : x + marker_px] = cv2.aruco.generateImageMarker(
                    dictionary,
                    marker_id,
                    marker_px,
                    borderBits=p.border_bits,
                )
        width_m = p.num_columns * p.marker_len_m + (p.num_columns - 1) * p.separation_m
        height_m = p.num_rows * p.marker_len_m + (p.num_rows - 1) * p.separation_m
        return MarkerImageWriter.save(image, output, width_m, height_m, p.margin_m)
