from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from domain.marker_generators import MarkerGenerator
from infrastructure.aruco_marker.aruco_pdf_writer import ArucoPDFWriter, PDFPlacement
from utils.common.aruco_dico_id_to_name import DICT_ID_TO_NAME


def _meters_to_pixels(m: float, dpi: int) -> int:
    # pixels = inches * dpi
    inches = m / 0.0254
    return int(round(inches * dpi))


@dataclass(frozen=True, slots=True)
class ArucoMarkerGenerationParams:
    marker_id: int
    marker_len_m: float
    dictionary: int = 16  # cv2.aruco predefined dictionary id (e.g. cv2.aruco.DICT_6X6_250)
    border_bits: int = 1
    dpi: int = 300
    margin_m: float = 0.0  # outer margin in PDF around the marker

    def __post_init__(self) -> None:
        if self.marker_id < 0:
            raise ValueError("marker_id must be >= 0")
        if self.marker_len_m <= 0:
            raise ValueError("marker_len_m must be > 0 (meters)")
        if self.dictionary < 0:
            raise ValueError("dictionary must be >= 0")
        if self.border_bits <= 0:
            raise ValueError("border_bits must be > 0")
        if self.dpi < 150:
            raise ValueError("dpi must be >= 150")
        if self.margin_m < 0:
            raise ValueError("margin_m must be >= 0")


class ArucoMarkerGenerator(MarkerGenerator):
    """
    Generates a single ArUco marker into a print-accurate PDF.
    """

    def __init__(self, generation_params: ArucoMarkerGenerationParams) -> None:
        self._p = generation_params
        self._pdf_writer = ArucoPDFWriter(
            "markers", f"aruco_marker_{DICT_ID_TO_NAME.get(generation_params.dictionary)}"
        )

    def _get_dictionary(self):
        # OpenCV expects a "predefined dictionary" constant (int)
        return cv2.aruco.getPredefinedDictionary(int(self._p.dictionary))

    def _draw_marker_u8(self, side_px: int) -> np.ndarray:
        if side_px <= 0:
            raise ValueError("side_px must be > 0")

        dictionary = self._get_dictionary()
        img = np.zeros((side_px, side_px), dtype=np.uint8)

        cv2.aruco.generateImageMarker(dictionary, int(self._p.marker_id), side_px, img, int(self._p.border_bits))
        return img

    def generate(self) -> Path:

        side_px = _meters_to_pixels(self._p.marker_len_m, self._p.dpi)
        marker_u8 = self._draw_marker_u8(side_px)

        page_side_m = self._p.marker_len_m + 2.0 * self._p.margin_m
        placement = PDFPlacement(
            page_w_m=page_side_m,
            page_h_m=page_side_m,
            content_w_m=self._p.marker_len_m,
            content_h_m=self._p.marker_len_m,
            margin_left_m=self._p.margin_m,
            margin_bottom_m=self._p.margin_m,
        )

        return ArucoPDFWriter.save(marker_u8, placement)
