from dataclasses import dataclass
from datetime import datetime
from io import BytesIO
from pathlib import Path

import numpy as np
from PIL import Image
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas


def _meters_to_points(m: float) -> float:
    return (m / 0.0254) * 72.0


@dataclass(frozen=True, slots=True)
class PDFPlacement:
    page_w_m: float
    page_h_m: float
    content_w_m: float
    content_h_m: float
    margin_left_m: float = 0.0
    margin_bottom_m: float = 0.0

    def __post_init__(self) -> None:
        for name, v in (
            ("page_w_m", self.page_w_m),
            ("page_h_m", self.page_h_m),
            ("content_w_m", self.content_w_m),
            ("content_h_m", self.content_h_m),
            ("margin_left_m", self.margin_left_m),
            ("margin_bottom_m", self.margin_bottom_m),
        ):
            if v < 0:
                raise ValueError(f"{name} must be >= 0, got {v}")
        if self.page_w_m <= 0 or self.page_h_m <= 0:
            raise ValueError("page_w_m and page_h_m must be > 0")
        if self.content_w_m <= 0 or self.content_h_m <= 0:
            raise ValueError("content_w_m and content_h_m must be > 0")
        if self.margin_left_m + self.content_w_m > self.page_w_m + 1e-12:
            raise ValueError("content overflows page width with given margin_left_m")
        if self.margin_bottom_m + self.content_h_m > self.page_h_m + 1e-12:
            raise ValueError("content overflows page height with given margin_bottom_m")


class ArucoPDFWriter:
    """
    Writes a uint8 grayscale image into a PDF with an exact physical size (meters).
    """

    def __init__(self, default_folder_name: str, default_filename: str) -> None:
        self.default_generated_markers_filedir: Path = Path(f"generated_markers/{default_folder_name}")
        self.generated_markers_file_extension = ".pdf"
        self.default_filename = default_filename
        self.saving_datetime_format = "%Y-%m-%d_%H-%M-%S"

    def _get_filepath(self):
        self.default_generated_markers_filedir.mkdir(parents=True, exist_ok=True)
        file_created_datetime = datetime.now().strftime(self.saving_datetime_format)
        file_path = f"{self.default_filename}_{file_created_datetime}{self.generated_markers_file_extension}"
        file_path = self.default_generated_markers_filedir / file_path
        return file_path

    def save(
        self,
        gray_u8: np.ndarray,
        placement: PDFPlacement,
    ) -> Path:

        if gray_u8.ndim != 2:
            raise ValueError("gray_u8 must be a 2D grayscale image (H,W)")
        if gray_u8.dtype != np.uint8:
            gray_u8 = gray_u8.astype(np.uint8, copy=False)

        page_w_pt = _meters_to_points(placement.page_w_m)
        page_h_pt = _meters_to_points(placement.page_h_m)
        content_w_pt = _meters_to_points(placement.content_w_m)
        content_h_pt = _meters_to_points(placement.content_h_m)
        x_pt = _meters_to_points(placement.margin_left_m)
        y_pt = _meters_to_points(placement.margin_bottom_m)

        # TODO : supprimer
        pil = Image.fromarray(gray_u8).convert("RGB")
        buf = BytesIO()
        pil.save(buf, format="PNG")
        buf.seek(0)
        # ----------

        pdf_path = self._get_filepath()

        c = canvas.Canvas(str(pdf_path), pagesize=(page_w_pt, page_h_pt))
        c.drawImage(
            ImageReader(buf),
            x_pt,
            y_pt,
            width=content_w_pt,
            height=content_h_pt,
            mask="auto",
        )
        c.showPage()
        c.save()

        return pdf_path
