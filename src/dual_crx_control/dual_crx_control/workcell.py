"""Read-only workcell metadata and five-point verification target generation."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any

import yaml


POINT_ORDER = (
    "CENTER",
    "X_PLUS_Y_PLUS",
    "X_MINUS_Y_PLUS",
    "X_MINUS_Y_MINUS",
    "X_PLUS_Y_MINUS",
)
MIN_CLEARANCE_M = 0.015
DEFAULT_CLEARANCE_M = 0.020
DEFAULT_TRANSIT_HEIGHT_M = 0.080


class WorkcellError(RuntimeError):
    pass


def _finite_vector(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise WorkcellError(f"{label} must contain {length} values")
    if not all(isinstance(item, (int, float)) and math.isfinite(item) for item in value):
        raise WorkcellError(f"{label} must contain finite numbers")
    return [float(item) for item in value]


def load_workcell(path: str | Path) -> tuple[dict[str, Any], dict[str, Any]]:
    resolved = Path(path).expanduser().resolve()
    try:
        raw = resolved.read_bytes()
    except OSError as exc:
        raise WorkcellError(f"cannot read placement file: {exc}") from exc
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise WorkcellError(f"invalid placement YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise WorkcellError("placement YAML must contain a mapping")
    for arm in ("left_arm", "right_arm"):
        placement = data.get(arm)
        if not isinstance(placement, dict):
            raise WorkcellError(f"placement is missing {arm}")
        _finite_vector(placement.get("xyz"), 3, f"{arm}.xyz")
        _finite_vector(placement.get("rpy"), 3, f"{arm}.rpy")
    contract = data.get("coordinate_contract", {})
    if not isinstance(contract, dict):
        contract = {}
    offsets = contract.get("eef_z_offset_m", {})
    if not isinstance(offsets, dict):
        offsets = {}
    metadata = {
        "loaded": True,
        "valid": data.get("valid") is True,
        "world_frame": "world",
        "table_frame": str(contract.get("world_frame", "table_frame")),
        "placement_file": str(resolved),
        "profile_name": str(data.get("profile_name", "")),
        "generated_at": str(data.get("generated_at", "")),
        "source_calibration": str(
            data.get("source_profile", data.get("source_candidate", ""))
        ),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "left_eef_z_offset_m": float(offsets.get("left_arm", 0.0)),
        "right_eef_z_offset_m": float(offsets.get("right_arm", 0.0)),
        "reason": "loaded",
    }
    return data, metadata


def _rpy_matrix(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def verification_targets(
    data: dict[str, Any], arm: str, clearance_m: float, transit_height_m: float
) -> list[dict[str, Any]]:
    if data.get("valid") is not True:
        raise WorkcellError("five-point verification requires valid: true")
    if arm not in ("left_arm", "right_arm"):
        raise WorkcellError("arm must be left_arm or right_arm")
    if not math.isfinite(clearance_m) or clearance_m < MIN_CLEARANCE_M:
        raise WorkcellError(
            f"clearance must be at least {MIN_CLEARANCE_M * 1000:.0f} mm"
        )
    if not math.isfinite(transit_height_m) or transit_height_m < clearance_m + 0.030:
        raise WorkcellError("transit height must be at least 30 mm above clearance")

    square = data.get("inferred_square", {})
    points = square.get("point_coordinates_m", {}) if isinstance(square, dict) else {}
    missing = [point for point in POINT_ORDER if point not in points]
    if missing:
        raise WorkcellError("placement profile is missing five-point coordinates")

    contract = data.get("coordinate_contract", {})
    offsets = contract.get("eef_z_offset_m", {}) if isinstance(contract, dict) else {}
    try:
        eef_offset = float(offsets[arm])
    except (KeyError, TypeError, ValueError) as exc:
        raise WorkcellError(f"placement profile is missing {arm} EEF Z offset") from exc
    if not math.isfinite(eef_offset):
        raise WorkcellError(f"{arm} EEF Z offset is not finite")

    placement = data.get(arm, {})
    rpy = _finite_vector(placement.get("rpy"), 3, f"{arm}.rpy")
    rotation = _rpy_matrix(rpy)
    offset_in_table = [rotation[row][2] * eef_offset for row in range(3)]

    targets = []
    for point_name in POINT_ORDER:
        point = _finite_vector(points[point_name], 3, point_name)
        def flange_target(height: float) -> list[float]:
            contact = [point[0], point[1], point[2] + height]
            return [contact[index] - offset_in_table[index] for index in range(3)]
        targets.append(
            {
                "point": point_name,
                "transit": flange_target(transit_height_m),
                "checkpoint": flange_target(clearance_m),
            }
        )
    return targets
