"""Write marker rasters with an explicit physical size (in meters)."""

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageOps
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas


def validate_margin(margin_m: float) -> None:
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError("margin_m must be finite and >= 0")


def validate_integer(name: str, value: int, minimum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")


def meters_to_pixels(length_m: float, dpi: int) -> int:
    return round(length_m / 0.0254 * dpi)


class MarkerImageWriter:
    """PDF dimensions are exact; PNG dimensions are stored in its DPI metadata."""

    @staticmethod
    def save(
        gray_u8: np.ndarray,
        output: Path,
        content_w_m: float,
        content_h_m: float,
        margin_m: float = 0.0,
    ) -> Path:
        output = Path(output).expanduser()
        if output.suffix.lower() not in (".pdf", ".png"):
            raise ValueError("Output must have a .pdf or .png extension")
        if gray_u8.ndim != 2 or gray_u8.dtype != np.uint8 or gray_u8.size == 0:
            raise ValueError("Marker image must be a nonempty uint8 grayscale image")
        if any(not math.isfinite(value) or value <= 0 for value in (content_w_m, content_h_m)):
            raise ValueError("Content dimensions must be finite and > 0")
        validate_margin(margin_m)

        output.parent.mkdir(parents=True, exist_ok=True)
        image = Image.fromarray(gray_u8)
        if output.suffix.lower() == ".png":
            pixels_per_m_x = image.width / content_w_m
            pixels_per_m_y = image.height / content_h_m
            margin_x = round(margin_m * pixels_per_m_x)
            margin_y = round(margin_m * pixels_per_m_y)
            image = ImageOps.expand(image, border=(margin_x, margin_y), fill=255)
            image.save(output, format="PNG", dpi=(pixels_per_m_x * 0.0254, pixels_per_m_y * 0.0254))
        else:
            points_per_m = 72 / 0.0254
            pdf = canvas.Canvas(
                str(output),
                pagesize=((content_w_m + 2 * margin_m) * points_per_m, (content_h_m + 2 * margin_m) * points_per_m),
            )
            pdf.drawImage(
                ImageReader(image),
                margin_m * points_per_m,
                margin_m * points_per_m,
                width=content_w_m * points_per_m,
                height=content_h_m * points_per_m,
            )
            pdf.showPage()
            pdf.save()
        return output
