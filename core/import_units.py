import os
import re
from typing import Optional, Tuple

import numpy as np


ENGINE_INTERNAL_UNIT = "mm"
ENGINE_UNITS_PER_CM = 10.0

SUPPORTED_LENGTH_UNITS = ("mm", "cm", "m")
UNIT_SCALE_TO_MM = {
    "mm": 1.0,
    "cm": 10.0,
    "m": 1000.0,
}

UP_AXIS_OPTIONS = ("preserve", "x", "y", "z")

_STEP_SI_UNIT_RE = re.compile(
    r"SI_UNIT\(\s*(?:\.(?P<prefix>[A-Z]+)\.\s*,\s*)?\.METRE\.\s*\)",
    re.IGNORECASE,
)


def get_engine_units_per_cm() -> float:
    return ENGINE_UNITS_PER_CM


def get_engine_internal_unit() -> str:
    return ENGINE_INTERNAL_UNIT


def unit_scale_to_internal(unit: str) -> float:
    key = (unit or "").strip().lower()
    if key not in UNIT_SCALE_TO_MM:
        raise ValueError(f"Unsupported unit '{unit}'")
    return UNIT_SCALE_TO_MM[key]


def convert_extents(extents, from_unit: str, to_unit: str):
    values = np.array(extents, dtype=float)
    mm = values * unit_scale_to_internal(from_unit)
    return mm / unit_scale_to_internal(to_unit)


def detect_mesh_declared_unit(mesh) -> Optional[str]:
    candidates = []

    direct_units = getattr(mesh, "units", None)
    if direct_units:
        candidates.append(direct_units)

    metadata = getattr(mesh, "metadata", None) or {}
    for key in ("units", "unit", "file_units", "file_unit", "length_unit"):
        value = metadata.get(key)
        if value:
            candidates.append(value)

    for value in candidates:
        normalized = normalize_unit_label(value)
        if normalized:
            return normalized
    return None


def normalize_unit_label(value) -> Optional[str]:
    if value is None:
        return None
    raw = str(value).strip().lower()
    aliases = {
        "millimeter": "mm",
        "millimeters": "mm",
        "millimetre": "mm",
        "millimetres": "mm",
        "mm": "mm",
        "centimeter": "cm",
        "centimeters": "cm",
        "centimetre": "cm",
        "centimetres": "cm",
        "cm": "cm",
        "meter": "m",
        "meters": "m",
        "metre": "m",
        "metres": "m",
        "m": "m",
    }
    return aliases.get(raw)


def detect_step_file_unit(file_path: str) -> Optional[str]:
    suffix = os.path.splitext(file_path)[1].lower()
    if suffix not in (".step", ".stp"):
        return None

    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as fh:
            header = fh.read(200000)
    except OSError:
        return None

    match = _STEP_SI_UNIT_RE.search(header)
    if not match:
        return None

    prefix = (match.group("prefix") or "").strip().upper()
    if prefix == "MILLI":
        return "mm"
    if prefix == "CENTI":
        return "cm"
    if prefix in ("", "NONE"):
        return "m"
    return None


def detect_file_unit(file_path: str, mesh=None) -> Tuple[Optional[str], str]:
    suffix = os.path.splitext(file_path)[1].lower()

    if mesh is not None:
        mesh_unit = detect_mesh_declared_unit(mesh)
        if mesh_unit:
            return mesh_unit, "mesh metadata"

    if suffix in (".step", ".stp"):
        step_unit = detect_step_file_unit(file_path)
        if step_unit:
            return step_unit, "STEP header"
        return None, "STEP default unresolved"

    if suffix == ".stl":
        return None, "STL has no reliable unit metadata"

    return None, "no unit metadata"


def rotation_matrix_for_up_axis(source_up_axis: str) -> np.ndarray:
    axis = (source_up_axis or "preserve").strip().lower()
    if axis == "preserve" or axis == "z":
        return np.eye(4)

    angle = np.pi / 2.0
    if axis == "y":
        return _rotation_x(angle)
    if axis == "x":
        return _rotation_y(-angle)
    raise ValueError(f"Unsupported up-axis '{source_up_axis}'")


def _rotation_x(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    mat = np.eye(4)
    mat[1, 1] = c
    mat[1, 2] = -s
    mat[2, 1] = s
    mat[2, 2] = c
    return mat


def _rotation_y(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    mat = np.eye(4)
    mat[0, 0] = c
    mat[0, 2] = s
    mat[2, 0] = -s
    mat[2, 2] = c
    return mat

