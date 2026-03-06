from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from domain.marker_generators import MarkerGenerator
from infrastructure.aruco_marker.aruco_pdf_writer import ArucoPDFWriter, PDFPlacement
from utils.common.aruco_dico_id_to_name import DICT_ID_TO_NAME


def _meters_to_pixels(m: float, dpi: int) -> int:
    inches = m / 0.0254
    return int(round(inches * dpi))


@dataclass(frozen=True, slots=True)
class MarkerBoardGenerationParams:
    num_columns: int
    num_rows: int
    marker_len_m: float
    dictionary: int
    margins_m: float
    border_bits: int = 1
    dpi: int = 300

    def __post_init__(self) -> None:
        if self.num_columns <= 0:
            raise ValueError("num_columns must be > 0")
        if self.num_rows <= 0:
            raise ValueError("num_rows must be > 0")
        if self.marker_len_m <= 0:
            raise ValueError("marker_len_m must be > 0 (meters)")
        if self.margins_m < 0:
            raise ValueError("margins_m must be >= 0 (meters)")
        if self.border_bits <= 0:
            raise ValueError("border_bits must be > 0")
        if self.dictionary < 0:
            raise ValueError("dictionary must be >= 0")
        if self.dpi < 150:
            raise ValueError("dpi must be >= 150")


class ArucoBoardGenerator(MarkerGenerator):
    """
    Generates a GridBoard into a PDF
    """

    def __init__(self, generation_params: MarkerBoardGenerationParams) -> None:
        self._generations_params = generation_params
        self._pdf_writer = ArucoPDFWriter("boards", f"aruco_marker_{DICT_ID_TO_NAME.get(generation_params.dictionary)}")

    def _get_dictionary(self):
        return cv2.aruco.getPredefinedDictionary(int(self._generations_params.dictionary))

    def _build_board_image_px(self, marker_len_px: int, sep_px: int, margins_px: int) -> np.ndarray:
        p = self._generations_params
        img_w = p.num_columns * (marker_len_px + sep_px) - sep_px + 2 * margins_px
        img_h = p.num_rows * (marker_len_px + sep_px) - sep_px + 2 * margins_px
        image_size = (int(img_w), int(img_h))  # (w, h)

        dictionary = self._get_dictionary()

        board = cv2.aruco.GridBoard(
            (int(p.num_columns), int(p.num_rows)),
            float(marker_len_px),
            float(sep_px),
            dictionary,
        )

        board_img = board.generateImage(image_size, marginSize=int(margins_px), borderBits=int(p.border_bits))

        return np.array(board_img, dtype=np.uint8)

    def generate(self) -> Path:
        p = self._generations_params

        marker_len_px = _meters_to_pixels(p.marker_len_m, p.dpi)
        sep_px = _meters_to_pixels(p.margins_m, p.dpi)
        margins_px = _meters_to_pixels(p.margins_m, p.dpi)

        board_u8 = self._build_board_image_px(marker_len_px, sep_px, margins_px)

        content_w_m = p.num_columns * (p.marker_len_m + p.margins_m) - p.margins_m + 2 * p.margins_m
        content_h_m = p.num_rows * (p.marker_len_m + p.margins_m) - p.margins_m + 2 * p.margins_m

        placement = PDFPlacement(
            page_w_m=content_w_m,
            page_h_m=content_h_m,
            content_w_m=content_w_m,
            content_h_m=content_h_m,
            margin_left_m=0.0,
            margin_bottom_m=0.0,
        )

        return ArucoPDFWriter.save(board_u8, placement)
