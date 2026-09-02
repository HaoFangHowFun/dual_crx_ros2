#!/usr/bin/env python3
"""Small, operator-driven five-point table calibration tool.

The tool never connects to or moves a robot.  The operator records the same five
labelled table points with each robot and enters the TCP XYZ shown on the teach
pendant.  A proper rigid transform is fitted independently for each arm.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any

import numpy as np
import yaml


SCHEMA_VERSION = 1
POINT_IDS = (
    "CENTER",
    "X_PLUS_Y_PLUS",
    "X_PLUS_Y_MINUS",
    "X_MINUS_Y_PLUS",
    "X_MINUS_Y_MINUS",
)
LEGACY_POINT_IDS = ("P1", "P2", "P3", "P4", "P5")
ARMS = ("left_arm", "right_arm")
DEFAULT_BASE_TO_FANUC_WORLD_Z_M = 0.185
DEFAULT_RMS_LIMIT_M = 0.003
DEFAULT_MAX_LIMIT_M = 0.006
DEFAULT_CROSS_LIMIT_M = 0.006


class CalibrationError(RuntimeError):
    pass


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_plain(item) for item in value.tolist()]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def atomic_yaml_write(path: Path, data: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(
                _plain(data), handle, sort_keys=False, default_flow_style=False
            )
        os.replace(temp_name, path)
    except BaseException:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.expanduser().open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError as exc:
        raise CalibrationError(f"file not found: {path}") from exc
    if not isinstance(data, dict):
        raise CalibrationError(f"expected a YAML mapping in {path}")
    return data


def parse_xyz(text: str, unit: str) -> list[float]:
    fields = text.replace(",", " ").split()
    if len(fields) != 3:
        raise CalibrationError("enter exactly three numbers: X Y Z")
    try:
        values = np.asarray([float(field) for field in fields], dtype=float)
    except ValueError as exc:
        raise CalibrationError("X, Y, and Z must be numbers") from exc
    if not np.all(np.isfinite(values)):
        raise CalibrationError("coordinates must be finite")
    if unit == "mm":
        values *= 0.001
    return values.tolist()


def validate_points(points: np.ndarray, label: str) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.shape != (5, 3):
        raise CalibrationError(f"{label} must contain exactly five XYZ points")
    if not np.all(np.isfinite(points)):
        raise CalibrationError(f"{label} contains NaN or infinity")
    if len({tuple(row) for row in points}) != 5:
        raise CalibrationError(f"{label} contains duplicate points")
    singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    if singular[0] < 0.05:
        raise CalibrationError(f"{label} spans less than 50 mm")
    if singular[1] / singular[0] < 0.08:
        raise CalibrationError(
            f"{label} is nearly collinear; spread points in both table axes"
        )
    return points


def fit_rigid_transform(source: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    """Fit target = R * source + t using a proper Kabsch rotation."""
    source = validate_points(source, "measured TCP points")
    target = validate_points(target, "table points")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    if np.linalg.det(rotation) < 0.999999:
        raise CalibrationError("solver produced an improper/reflected rotation")
    translation = target_center - rotation @ source_center
    predicted = (rotation @ source.T).T + translation
    residuals = np.linalg.norm(predicted - target, axis=1)
    return {
        "rotation": rotation,
        "translation": translation,
        "predicted": predicted,
        "residuals": residuals,
        "rms": float(np.sqrt(np.mean(residuals**2))),
        "max": float(np.max(residuals)),
        "singular_values": singular,
    }


def matrix_to_rpy(rotation: np.ndarray) -> list[float]:
    """Return fixed-axis URDF roll, pitch, yaw for Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return [roll, pitch, yaw]


def table_to_base_placement(
    table_from_measurement: dict[str, Any],
    measurement_frame: str,
    base_to_fanuc_world_z_m: float,
) -> tuple[list[float], list[float]]:
    rotation = table_from_measurement["rotation"]
    translation = table_from_measurement["translation"].copy()
    if measurement_frame == "fanuc_world":
        # CRX-5iA URDF: base_link -> wbase is +0.185 m along base Z.
        # Therefore table -> base_link is table -> wbase followed by -0.185 m.
        translation = translation - rotation @ np.array(
            [0.0, 0.0, base_to_fanuc_world_z_m]
        )
    return translation.tolist(), matrix_to_rpy(rotation)


def _session_points(
    session: dict[str, Any], section: str, point_ids: tuple[str, ...] = POINT_IDS
) -> np.ndarray:
    values = session.get(section, {})
    try:
        return np.asarray([values[point_id] for point_id in point_ids], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError(
            f"{section} does not contain: {', '.join(point_ids)}"
        ) from exc


def _solve_known_points_session(
    session: dict[str, Any],
    *,
    rms_limit_m: float = DEFAULT_RMS_LIMIT_M,
    max_limit_m: float = DEFAULT_MAX_LIMIT_M,
    cross_limit_m: float = DEFAULT_CROSS_LIMIT_M,
) -> dict[str, Any]:
    if session.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationError("unsupported session schema_version")
    table = _session_points(session, "table_points_m", LEGACY_POINT_IDS)
    validate_points(table, "table points")
    measurement_frame = session.get("measurement_frame")
    if measurement_frame not in ("fanuc_world", "base_link"):
        raise CalibrationError("measurement_frame must be fanuc_world or base_link")
    offset = float(
        session.get("base_to_fanuc_world_z_m", DEFAULT_BASE_TO_FANUC_WORLD_Z_M)
    )
    if not math.isfinite(offset) or offset < 0.0:
        raise CalibrationError("invalid base-to-FANUC-world Z offset")

    fits: dict[str, dict[str, Any]] = {}
    placements: dict[str, dict[str, Any]] = {}
    eef_z_offsets = session.get("eef_z_offset_m", {})
    applied_offsets: dict[str, float] = {}
    for arm in ARMS:
        measured = _session_points(
            session, f"{arm}_tcp_points_m", LEGACY_POINT_IDS
        )
        try:
            z_offset = float(eef_z_offsets.get(arm, 0.0))
        except (AttributeError, TypeError, ValueError) as exc:
            raise CalibrationError(f"invalid {arm} EEF Z offset") from exc
        if not math.isfinite(z_offset):
            raise CalibrationError(f"invalid {arm} EEF Z offset")
        measured = measured.copy()
        measured[:, 2] += z_offset
        applied_offsets[arm] = z_offset
        fit = fit_rigid_transform(measured, table)
        xyz, rpy = table_to_base_placement(fit, measurement_frame, offset)
        fits[arm] = fit
        placements[arm] = {"xyz": xyz, "rpy": rpy}

    cross = np.linalg.norm(
        fits["left_arm"]["predicted"] - fits["right_arm"]["predicted"], axis=1
    )
    valid = all(
        fit["rms"] <= rms_limit_m and fit["max"] <= max_limit_m
        for fit in fits.values()
    ) and float(np.max(cross)) <= cross_limit_m

    residual_report = {}
    for arm, fit in fits.items():
        residual_report[arm] = {
            "rms_m": fit["rms"],
            "max_m": fit["max"],
            "per_point_m": {
                point_id: float(error)
                for point_id, error in zip(LEGACY_POINT_IDS, fit["residuals"])
            },
            "rotation_determinant": float(np.linalg.det(fit["rotation"])),
            "conditioning_singular_values": fit["singular_values"],
        }

    return {
        "schema_version": SCHEMA_VERSION,
        "calibration_method": "known_points",
        "valid": bool(valid),
        "generated_at": utc_now(),
        "coordinate_contract": {
            "world_frame": "table_frame",
            "placement_meaning": "table_frame_from_ros_base_link",
            "translation_unit": "m",
            "rotation": "URDF fixed-axis roll pitch yaw in radians",
            "measurement_frame": measurement_frame,
            "base_to_fanuc_world_z_m": offset,
            "eef_z_offset_convention": (
                "contact_Z_in_measurement_frame = entered_Z + eef_z_offset_m"
            ),
            "eef_z_offset_m": applied_offsets,
        },
        "left_arm": placements["left_arm"],
        "right_arm": placements["right_arm"],
        "quality": {
            "thresholds": {
                "rms_m": rms_limit_m,
                "max_point_m": max_limit_m,
                "cross_arm_max_m": cross_limit_m,
            },
            "arms": residual_report,
            "cross_arm_per_point_m": {
                point_id: float(error)
                for point_id, error in zip(LEGACY_POINT_IDS, cross)
            },
            "cross_arm_max_m": float(np.max(cross)),
        },
        "samples": {
            "table_points_m": session["table_points_m"],
            "left_arm_tcp_points_m": session["left_arm_tcp_points_m"],
            "right_arm_tcp_points_m": session["right_arm_tcp_points_m"],
            "eef_z_offset_m": applied_offsets,
            "tcp_setup_note": session.get("tcp_setup_note", ""),
            "session_created_at": session.get("created_at"),
        },
    }


def derive_center_square_frame(points: np.ndarray) -> dict[str, Any]:
    """Build table axes from CENTER and four semantically labelled square corners."""
    points = validate_points(points, "center/square TCP points")
    center, pp, pn, np_corner, nn = points
    x_vector = 0.5 * ((pp + pn) - (np_corner + nn))
    y_vector = 0.5 * ((pp + np_corner) - (pn + nn))
    x_span = float(np.linalg.norm(x_vector))
    y_span = float(np.linalg.norm(y_vector))
    if min(x_span, y_span) < 0.05:
        raise CalibrationError("square points span less than 50 mm in X or Y")
    cosine = float(np.dot(x_vector, y_vector) / (x_span * y_span))
    cosine = float(np.clip(cosine, -1.0, 1.0))
    raw_angle_deg = math.degrees(math.acos(cosine))
    if not 60.0 <= raw_angle_deg <= 120.0:
        raise CalibrationError("labelled square X/Y directions are nearly collinear")

    # The polar factor is the closest orthonormal pair to the two measured axes.
    raw_axes = np.column_stack((x_vector / x_span, y_vector / y_span))
    u, _, vt = np.linalg.svd(raw_axes, full_matrices=False)
    xy_axes = u @ vt
    x_axis = xy_axes[:, 0]
    y_axis = xy_axes[:, 1]
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    basis_measurement_from_table = np.column_stack((x_axis, y_axis, z_axis))
    rotation = basis_measurement_from_table.T
    translation = -rotation @ center
    transformed = (rotation @ points.T).T + translation

    half_extent = float(np.mean(np.abs(transformed[1:, :2])))
    ideal = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [half_extent, half_extent, 0.0],
            [half_extent, -half_extent, 0.0],
            [-half_extent, half_extent, 0.0],
            [-half_extent, -half_extent, 0.0],
        ]
    )
    residuals = np.linalg.norm(transformed - ideal, axis=1)
    return {
        "rotation": rotation,
        "translation": translation,
        "predicted": transformed,
        "ideal": ideal,
        "residuals": residuals,
        "rms": float(np.sqrt(np.mean(residuals**2))),
        "max": float(np.max(residuals)),
        "raw_axis_angle_deg": raw_angle_deg,
        "half_extent_m": half_extent,
        "rotation_determinant": float(np.linalg.det(rotation)),
        "spans_m": [x_span, y_span],
    }


def _solve_center_square_session(
    session: dict[str, Any],
    *,
    rms_limit_m: float,
    max_limit_m: float,
    cross_limit_m: float,
) -> dict[str, Any]:
    measurement_frame = session.get("measurement_frame")
    if measurement_frame not in ("fanuc_world", "base_link"):
        raise CalibrationError("measurement_frame must be fanuc_world or base_link")
    base_offset = float(
        session.get("base_to_fanuc_world_z_m", DEFAULT_BASE_TO_FANUC_WORLD_Z_M)
    )
    if not math.isfinite(base_offset) or base_offset < 0.0:
        raise CalibrationError("invalid base-to-FANUC-world Z offset")
    eef_z_offsets = session.get("eef_z_offset_m", {})

    fits: dict[str, dict[str, Any]] = {}
    placements: dict[str, dict[str, Any]] = {}
    applied_offsets: dict[str, float] = {}
    for arm in ARMS:
        measured = _session_points(session, f"{arm}_tcp_points_m")
        try:
            z_offset = float(eef_z_offsets.get(arm, 0.0))
        except (AttributeError, TypeError, ValueError) as exc:
            raise CalibrationError(f"invalid {arm} EEF Z offset") from exc
        if not math.isfinite(z_offset):
            raise CalibrationError(f"invalid {arm} EEF Z offset")
        corrected = measured.copy()
        corrected[:, 2] += z_offset
        fit = derive_center_square_frame(corrected)
        xyz, rpy = table_to_base_placement(fit, measurement_frame, base_offset)
        fits[arm] = fit
        placements[arm] = {"xyz": xyz, "rpy": rpy}
        applied_offsets[arm] = z_offset

    cross = np.linalg.norm(
        fits["left_arm"]["predicted"] - fits["right_arm"]["predicted"], axis=1
    )
    valid = all(
        fit["rms"] <= rms_limit_m and fit["max"] <= max_limit_m
        for fit in fits.values()
    ) and float(np.max(cross)) <= cross_limit_m

    arm_quality = {}
    for arm, fit in fits.items():
        arm_quality[arm] = {
            "rms_m": fit["rms"],
            "max_m": fit["max"],
            "per_point_m": {
                point_id: float(error)
                for point_id, error in zip(POINT_IDS, fit["residuals"])
            },
            "rotation_determinant": fit["rotation_determinant"],
            "raw_axis_angle_deg": fit["raw_axis_angle_deg"],
            "inferred_square_half_extent_m": fit["half_extent_m"],
            "measured_axis_spans_m": fit["spans_m"],
        }

    inferred_half_extent = float(
        np.mean([fit["half_extent_m"] for fit in fits.values()])
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "calibration_method": "center_square",
        "valid": bool(valid),
        "generated_at": utc_now(),
        "coordinate_contract": {
            "world_frame": "table_frame",
            "origin": "CENTER",
            "axis_definition": (
                "+X and +Y follow the four signed corner labels; +Z = +X cross +Y"
            ),
            "placement_meaning": "table_frame_from_ros_base_link",
            "translation_unit": "m",
            "rotation": "URDF fixed-axis roll pitch yaw in radians",
            "measurement_frame": measurement_frame,
            "base_to_fanuc_world_z_m": base_offset,
            "eef_z_offset_convention": (
                "contact_Z_in_measurement_frame = entered_Z + eef_z_offset_m"
            ),
            "eef_z_offset_m": applied_offsets,
        },
        "inferred_square": {
            "half_extent_m": inferred_half_extent,
            "side_length_m": 2.0 * inferred_half_extent,
            "point_coordinates_m": {
                point_id: coordinates
                for point_id, coordinates in zip(
                    POINT_IDS,
                    [
                        [0.0, 0.0, 0.0],
                        [inferred_half_extent, inferred_half_extent, 0.0],
                        [inferred_half_extent, -inferred_half_extent, 0.0],
                        [-inferred_half_extent, inferred_half_extent, 0.0],
                        [-inferred_half_extent, -inferred_half_extent, 0.0],
                    ],
                )
            },
        },
        "left_arm": placements["left_arm"],
        "right_arm": placements["right_arm"],
        "quality": {
            "thresholds": {
                "rms_m": rms_limit_m,
                "max_point_m": max_limit_m,
                "cross_arm_max_m": cross_limit_m,
            },
            "arms": arm_quality,
            "cross_arm_per_point_m": {
                point_id: float(error) for point_id, error in zip(POINT_IDS, cross)
            },
            "cross_arm_max_m": float(np.max(cross)),
        },
        "samples": {
            "left_arm_tcp_points_m": session["left_arm_tcp_points_m"],
            "right_arm_tcp_points_m": session["right_arm_tcp_points_m"],
            "eef_z_offset_m": applied_offsets,
            "tcp_setup_note": session.get("tcp_setup_note", ""),
            "session_created_at": session.get("created_at"),
        },
    }


def solve_session(
    session: dict[str, Any],
    *,
    rms_limit_m: float = DEFAULT_RMS_LIMIT_M,
    max_limit_m: float = DEFAULT_MAX_LIMIT_M,
    cross_limit_m: float = DEFAULT_CROSS_LIMIT_M,
) -> dict[str, Any]:
    if session.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationError("unsupported session schema_version")
    if session.get("calibration_method") == "center_square":
        return _solve_center_square_session(
            session,
            rms_limit_m=rms_limit_m,
            max_limit_m=max_limit_m,
            cross_limit_m=cross_limit_m,
        )
    if "table_points_m" in session:
        return _solve_known_points_session(
            session,
            rms_limit_m=rms_limit_m,
            max_limit_m=max_limit_m,
            cross_limit_m=cross_limit_m,
        )
    raise CalibrationError("unknown calibration method")


def default_session_path() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("calibration_sessions") / f"table_5point_{stamp}.yaml"


def default_candidate_path(session_path: Path) -> Path:
    return Path("calibration_candidates") / f"{session_path.stem}_candidate.yaml"


def _ask_xyz(prompt: str, unit: str) -> list[float] | None:
    while True:
        text = input(prompt).strip()
        if text.lower() in ("q", "quit", "exit"):
            return None
        try:
            value = parse_xyz(text, unit)
        except CalibrationError as exc:
            print(f"  {exc}")
            continue
        shown = ", ".join(f"{item:.3f}" for item in value)
        if input(f"  stored as metres [{shown}] — accept? [Y/n] ").strip().lower() in (
            "",
            "y",
            "yes",
        ):
            return value


def new_session(
    unit: str,
    measurement_frame: str,
    note: str,
    left_eef_z_offset_m: float = 0.0,
    right_eef_z_offset_m: float = 0.0,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "calibration_method": "center_square",
        "created_at": utc_now(),
        "input_unit": unit,
        "measurement_frame": measurement_frame,
        "base_to_fanuc_world_z_m": DEFAULT_BASE_TO_FANUC_WORLD_Z_M,
        "tcp_setup_note": note,
        "eef_z_offset_m": {
            "left_arm": left_eef_z_offset_m,
            "right_arm": right_eef_z_offset_m,
        },
        "left_arm_tcp_points_m": {},
        "right_arm_tcp_points_m": {},
    }


def collect(args: argparse.Namespace) -> int:
    session_path = args.session or default_session_path()
    session_path = session_path.expanduser()
    if session_path.exists():
        session = load_yaml(session_path)
        unit = session.get("input_unit", args.unit)
        print(f"Resuming {session_path}")
    else:
        print("This tool does not connect to or move either robot.")
        print("Use the SAME calibrated TCP/UTOOL and UFRAME 0 for all ten readings.")
        print("Enter q at any prompt to save and quit; rerun the same command to resume.\n")
        session = new_session(
            args.unit,
            args.measurement_frame,
            args.tcp_note,
            args.left_eef_z_offset_mm * 0.001,
            args.right_eef_z_offset_mm * 0.001,
        )
        unit = args.unit
        atomic_yaml_write(session_path, session)

    sections = (
        ("left_arm_tcp_points_m", "LEFT teach-pendant TCP"),
        ("right_arm_tcp_points_m", "RIGHT teach-pendant TCP"),
    )
    for section, title in sections:
        values = session.setdefault(section, {})
        print(f"\n{title} coordinates ({unit})")
        for point_id in POINT_IDS:
            if point_id in values:
                print(f"  {point_id}: already recorded {values[point_id]} m")
                continue
            value = _ask_xyz(f"  {point_id} X Y Z (or q): ", unit)
            if value is None:
                atomic_yaml_write(session_path, session)
                print(f"Saved partial session: {session_path}")
                return 0
            values[point_id] = value
            atomic_yaml_write(session_path, session)

    candidate = solve_session(
        session,
        rms_limit_m=args.rms_limit_mm * 0.001,
        max_limit_m=args.max_limit_mm * 0.001,
        cross_limit_m=args.cross_limit_mm * 0.001,
    )
    candidate_path = args.candidate or default_candidate_path(session_path)
    atomic_yaml_write(candidate_path, candidate)
    print_report(candidate, candidate_path)
    return 0 if candidate["valid"] else 1


def record(args: argparse.Namespace) -> int:
    session = load_yaml(args.session)
    section = f"{args.arm}_arm_tcp_points_m"
    if args.point not in POINT_IDS:
        raise CalibrationError(f"point must be one of {', '.join(POINT_IDS)}")
    session.setdefault(section, {})[args.point] = parse_xyz(" ".join(args.xyz), args.unit)
    session["updated_at"] = utc_now()
    atomic_yaml_write(args.session, session)
    print(f"Updated {args.arm} {args.point} in {args.session}")
    return 0


def solve_command(args: argparse.Namespace) -> int:
    session = load_yaml(args.session)
    candidate = solve_session(
        session,
        rms_limit_m=args.rms_limit_mm * 0.001,
        max_limit_m=args.max_limit_mm * 0.001,
        cross_limit_m=args.cross_limit_mm * 0.001,
    )
    candidate_path = args.candidate or default_candidate_path(args.session)
    atomic_yaml_write(candidate_path, candidate)
    print_report(candidate, candidate_path)
    return 0 if candidate["valid"] else 1


def print_report(candidate: dict[str, Any], path: Path | None = None) -> None:
    quality = candidate["quality"]
    print("\nCalibration result:", "VALID" if candidate["valid"] else "INVALID")
    for arm in ARMS:
        placement = candidate[arm]
        metrics = quality["arms"][arm]
        xyz = ", ".join(f"{value:.6f}" for value in placement["xyz"])
        rpy = ", ".join(f"{value:.6f}" for value in placement["rpy"])
        print(f"  {arm}: xyz=[{xyz}] m  rpy=[{rpy}] rad")
        print(
            f"    RMS={metrics['rms_m'] * 1000:.2f} mm, "
            f"max={metrics['max_m'] * 1000:.2f} mm"
        )
    print(f"  cross-arm max={quality['cross_arm_max_m'] * 1000:.2f} mm")
    if path:
        print(f"Candidate written to: {path}")
    if not candidate["valid"]:
        print("Candidate is not activatable; redo the largest-error point(s).")


def activate_candidate_file(candidate_path: Path, output: Path) -> Path | None:
    candidate = load_yaml(candidate_path)
    if candidate.get("valid") is not True:
        raise CalibrationError("refusing to activate a candidate that is not valid")
    output = output.expanduser().resolve()
    backup = None
    if output.exists():
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = output.with_name(f"{output.name}.backup_{stamp}")
        shutil.copy2(output, backup)
    active = dict(candidate)
    active["activated_at"] = utc_now()
    active["source_candidate"] = str(candidate_path.expanduser().resolve())
    atomic_yaml_write(output, active)
    return backup


def activate(args: argparse.Namespace) -> int:
    output = args.output.expanduser().resolve()
    candidate = load_yaml(args.candidate)
    if candidate.get("valid") is not True:
        raise CalibrationError("refusing to activate a candidate that is not valid")
    if not args.yes:
        response = input(
            f"Activate {args.candidate} as {output}? Type ACTIVATE to continue: "
        )
        if response != "ACTIVATE":
            print("Activation cancelled.")
            return 1
    backup = activate_candidate_file(args.candidate, output)
    if backup:
        print(f"Previous active calibration backed up to: {backup}")
    print(f"Activated physical placement: {output}")
    return 0


def show(args: argparse.Namespace) -> int:
    data = load_yaml(args.path)
    if "quality" in data and "left_arm" in data:
        print_report(data, args.path)
    else:
        print(yaml.safe_dump(data, sort_keys=False))
    return 0


def add_quality_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--rms-limit-mm", type=float, default=3.0)
    parser.add_argument("--max-limit-mm", type=float, default=6.0)
    parser.add_argument("--cross-limit-mm", type=float, default=6.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Operator-driven five-point dual-CRX table calibration"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    collect_parser = subparsers.add_parser("collect", help="create or resume a session")
    collect_parser.add_argument("--session", type=Path)
    collect_parser.add_argument("--candidate", type=Path)
    collect_parser.add_argument("--unit", choices=("mm", "m"), default="mm")
    collect_parser.add_argument(
        "--measurement-frame",
        choices=("fanuc_world", "base_link"),
        default="fanuc_world",
        help="fanuc_world means teach-pendant UFRAME 0 (default)",
    )
    collect_parser.add_argument(
        "--tcp-note",
        default="Record the UTOOL number and verify its TCP calibration before use.",
    )
    collect_parser.add_argument("--left-eef-z-offset-mm", type=float, default=0.0)
    collect_parser.add_argument("--right-eef-z-offset-mm", type=float, default=0.0)
    add_quality_arguments(collect_parser)
    collect_parser.set_defaults(func=collect)

    record_parser = subparsers.add_parser("record", help="replace one arm measurement")
    record_parser.add_argument("session", type=Path)
    record_parser.add_argument("--arm", choices=("left", "right"), required=True)
    record_parser.add_argument("--point", required=True)
    record_parser.add_argument("--xyz", nargs=3, required=True)
    record_parser.add_argument("--unit", choices=("mm", "m"), default="mm")
    record_parser.set_defaults(func=record)

    solve_parser = subparsers.add_parser("solve", help="solve an existing session")
    solve_parser.add_argument("session", type=Path)
    solve_parser.add_argument("--candidate", type=Path)
    add_quality_arguments(solve_parser)
    solve_parser.set_defaults(func=solve_command)

    activate_parser = subparsers.add_parser(
        "activate", help="explicitly activate a valid candidate for physical launch"
    )
    activate_parser.add_argument("candidate", type=Path)
    activate_parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "src/dual_crx_description/config/robot_placement_physical.yaml"
        ),
    )
    activate_parser.add_argument("--yes", action="store_true")
    activate_parser.set_defaults(func=activate)

    show_parser = subparsers.add_parser("show", help="show a session or result")
    show_parser.add_argument("path", type=Path)
    show_parser.set_defaults(func=show)
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        return int(args.func(args))
    except (CalibrationError, yaml.YAMLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nCancelled by operator.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
