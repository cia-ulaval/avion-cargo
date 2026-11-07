"""Generate the bridge's native fractal raster without resampling its cells."""

from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2

from domain.marker_generators import MarkerGenerator
from domain.markers import DEFAULT_FRACTAL_TYPE, FractalType, MarkerType
from domain.models import TargetedMarker
from infrastructure.marker_generation.image_writer import MarkerImageWriter, validate_integer, validate_margin
from infrastructure.marker_generation.settings import (
    DEFAULT_FRACTAL_BIT_SIZE,
    DEFAULT_WHITE_MARGIN_M,
    MIN_FRACTAL_BIT_SIZE,
)


@dataclass(frozen=True, slots=True)
class FArUcoMarkerGenerationParams:
    marker_len_m: float
    fractal_type: FractalType = DEFAULT_FRACTAL_TYPE
    bit_size: int = DEFAULT_FRACTAL_BIT_SIZE
    margin_m: float = DEFAULT_WHITE_MARGIN_M

    def __post_init__(self) -> None:
        target = TargetedMarker(
            id=None, length=self.marker_len_m, marker_type=MarkerType.FARUCO, fractal_type=self.fractal_type
        )
        object.__setattr__(self, "fractal_type", target.fractal_type)
        validate_integer("bit_size", self.bit_size, MIN_FRACTAL_BIT_SIZE)
        validate_margin(self.margin_m)


class FArUcoMarkerGenerator(MarkerGenerator):
    def __init__(self, generation_params: FArUcoMarkerGenerationParams) -> None:
        self._p = generation_params

    def generate(self, output: Path) -> Path:
        try:
            from faruco_bridge import generate_marker
        except ImportError as error:
            raise RuntimeError(
                "FArUco generation requires 'faruco-bridge' with generate_marker support; "
                "install it with 'poetry install --with faruco'."
            ) from error

        p = self._p
        with TemporaryDirectory(prefix="autolander-faruco-") as directory:
            marker_path = Path(directory) / "marker.png"
            generate_marker(fractal_type=p.fractal_type.value, output=marker_path, bit_size=p.bit_size)
            image = cv2.imread(str(marker_path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise RuntimeError("FArUco Bridge did not produce a readable marker image")
            return MarkerImageWriter.save(image, output, p.marker_len_m, p.marker_len_m, p.margin_m)
