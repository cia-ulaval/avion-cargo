"""Marker identities, built-in layouts and dictionary limits shared by all adapters."""

from enum import StrEnum


class MarkerType(StrEnum):
    ARUCO = "aruco"
    FARUCO = "faruco"


class FractalType(StrEnum):
    FRACTAL_2L_6 = "FRACTAL_2L_6"
    FRACTAL_3L_6 = "FRACTAL_3L_6"
    FRACTAL_4L_6 = "FRACTAL_4L_6"
    FRACTAL_5L_6 = "FRACTAL_5L_6"

    @property
    def side_ratios(self) -> tuple[float, ...]:
        """ArUco2 level sides relative to the outer marker, in the same centred frame."""
        return _FRACTAL_SIDE_RATIOS[self]


_FRACTAL_SIDE_RATIOS = {
    FractalType.FRACTAL_2L_6: (1.0, 1 / 3),
    FractalType.FRACTAL_3L_6: (1.0, 3 / 7, 1 / 7),
    FractalType.FRACTAL_4L_6: (1.0, 7 / 15, 1 / 5, 1 / 15),
    FractalType.FRACTAL_5L_6: (1.0, 5 / 13, 7 / 39, 1 / 13, 1 / 39),
}

DEFAULT_FRACTAL_TYPE = FractalType.FRACTAL_3L_6
SUPPORTED_FRACTAL_TYPES = tuple(preset.value for preset in FractalType)
FRACTAL_ROOT_ID = 0

_ARUCO_DICTIONARY_CAPACITIES = (50, 100, 250, 1000) * 4 + (1024,)
SUPPORTED_ARUCO_DICTIONARY_IDS = range(len(_ARUCO_DICTIONARY_CAPACITIES))
DEFAULT_ARUCO_DICTIONARY = 16  # DICT_ARUCO_ORIGINAL


def aruco_dictionary_capacity(dictionary_id: int) -> int:
    """Return the number of IDs in a supported OpenCV ArUco dictionary."""
    if isinstance(dictionary_id, bool) or not isinstance(dictionary_id, int):
        raise ValueError("dictionary must be an integer")
    if dictionary_id not in SUPPORTED_ARUCO_DICTIONARY_IDS:
        raise ValueError(
            f"dictionary must be between {min(SUPPORTED_ARUCO_DICTIONARY_IDS)} "
            f"and {max(SUPPORTED_ARUCO_DICTIONARY_IDS)}"
        )
    return _ARUCO_DICTIONARY_CAPACITIES[dictionary_id]
