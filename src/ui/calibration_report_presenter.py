"""Terminal presentation of calibration reports."""

from typing import Any

import numpy as np
from tabulate import tabulate

from domain.models import CalibrationReport


def _format_matrix(
    m: np.ndarray,
    float_fmt: str = ".6f",
    table_fmt: str = "rounded_outline",
) -> str:
    arr = np.asarray(m)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)

    rows: list[list[Any]] = []
    for r in arr:
        rows.append([float(x) if np.isfinite(x) else x for x in r])

    return tabulate(
        rows,
        tablefmt=table_fmt,
        floatfmt=float_fmt,
        colalign=("right",) * (arr.shape[1] if arr.ndim == 2 else 1),
    )


def _fmt_float(v: Any, ndigits: int = 6) -> str:
    if v is None:
        return "—"
    try:
        fv = float(v)
    except ValueError:
        return str(v)
    if not np.isfinite(fv):
        return str(v)
    return f"{fv:.{ndigits}f}"


def _fmt_int(v: Any) -> str:
    if v is None:
        return "—"
    try:
        return f"{int(v)}"
    except ValueError:
        return str(v)


def show_calibration_report(report: CalibrationReport) -> None:
    date_str = report.calibration_date.strftime("%Y-%m-%d %H:%M:%S")
    resolution = f"{_fmt_int(report.image_width)}×{_fmt_int(report.image_height)}"
    reproj = _fmt_float(report.avg_reprojection_error, ndigits=6)
    aspect = "—" if report.aspect_ratio is None else _fmt_float(report.aspect_ratio, ndigits=6)

    summary_rows = [
        ("Calibration date", date_str),
        ("Image size", resolution),
        ("Average reprojection error", reproj),
        ("Aspect ratio", aspect),
    ]

    d = np.asarray(report.camera_distortion_matrix)
    d_flat = d.reshape(-1)

    print(
        f"\n{tabulate(summary_rows, tablefmt='rounded_outline')}\n\n"
        f"Camera matrix (K)\n{_format_matrix(report.camera_matrix, float_fmt='.6f')}\n\n"
        f"Distortion coefficients (D)\n{_format_matrix(d_flat, float_fmt='.8f')}\n"
    )
