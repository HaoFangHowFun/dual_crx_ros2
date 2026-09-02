import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "table_calibration_tool.py"
SPEC = importlib.util.spec_from_file_location("table_calibration_tool", SCRIPT)
tool = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(tool)


TABLE = np.array(
    [
        [-0.25, -0.15, 0.0],
        [0.25, -0.15, 0.0],
        [0.25, 0.15, 0.0],
        [-0.25, 0.15, 0.0],
        [0.0, 0.0, 0.0],
    ]
)


def rpy_matrix(roll, pitch, yaw):
    rx = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )
    ry = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
    )
    rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    return rz @ ry @ rx


def measured_in_fanuc_world(table_from_base_rotation, table_from_base_translation):
    offset = np.array([0.0, 0.0, tool.DEFAULT_BASE_TO_FANUC_WORLD_Z_M])
    table_from_wbase_translation = (
        table_from_base_translation + table_from_base_rotation @ offset
    )
    return (
        table_from_base_rotation.T
        @ (TABLE - table_from_wbase_translation).T
    ).T


def session(left, right):
    return {
        "schema_version": 1,
        "calibration_method": "known_points",
        "created_at": "test",
        "measurement_frame": "fanuc_world",
        "base_to_fanuc_world_z_m": 0.185,
        "table_points_m": dict(zip(tool.LEGACY_POINT_IDS, TABLE.tolist())),
        "left_arm_tcp_points_m": dict(zip(tool.LEGACY_POINT_IDS, left.tolist())),
        "right_arm_tcp_points_m": dict(zip(tool.LEGACY_POINT_IDS, right.tolist())),
    }


SQUARE = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.2, 0.2, 0.0],
        [0.2, -0.2, 0.0],
        [-0.2, 0.2, 0.0],
        [-0.2, -0.2, 0.0],
    ]
)


def center_square_session(left, right):
    return {
        "schema_version": 1,
        "calibration_method": "center_square",
        "created_at": "test",
        "measurement_frame": "fanuc_world",
        "base_to_fanuc_world_z_m": 0.185,
        "eef_z_offset_m": {"left_arm": 0.0, "right_arm": 0.0},
        "left_arm_tcp_points_m": dict(zip(tool.POINT_IDS, left.tolist())),
        "right_arm_tcp_points_m": dict(zip(tool.POINT_IDS, right.tolist())),
    }


def square_measured_in_fanuc_world(rotation, translation):
    offset = np.array([0.0, 0.0, tool.DEFAULT_BASE_TO_FANUC_WORLD_Z_M])
    table_from_wbase_translation = translation + rotation @ offset
    return (rotation.T @ (SQUARE - table_from_wbase_translation).T).T


def test_exact_recovery_outputs_table_to_ros_base_link():
    left_r = rpy_matrix(0.02, -0.01, 0.45)
    right_r = rpy_matrix(-0.015, 0.025, -1.1)
    left_t = np.array([-0.35, 0.42, -0.16])
    right_t = np.array([0.38, -0.41, -0.17])

    result = tool.solve_session(
        session(
            measured_in_fanuc_world(left_r, left_t),
            measured_in_fanuc_world(right_r, right_t),
        )
    )

    assert result["valid"] is True
    assert np.allclose(result["left_arm"]["xyz"], left_t, atol=1e-10)
    assert np.allclose(result["right_arm"]["xyz"], right_t, atol=1e-10)
    assert result["quality"]["cross_arm_max_m"] < 1e-10


def test_small_seeded_touch_noise_passes():
    rng = np.random.default_rng(42)
    left_r = rpy_matrix(0.0, 0.0, 0.4)
    right_r = rpy_matrix(0.0, 0.0, -1.2)
    left = measured_in_fanuc_world(left_r, np.array([-0.3, 0.4, -0.15]))
    right = measured_in_fanuc_world(right_r, np.array([0.3, -0.4, -0.15]))
    left += rng.normal(0.0, 0.0004, left.shape)
    right += rng.normal(0.0, 0.0004, right.shape)

    result = tool.solve_session(session(left, right))

    assert result["valid"] is True
    assert result["quality"]["arms"]["left_arm"]["rms_m"] < 0.003


def test_center_and_four_square_labels_define_table_frame_without_table_xyz():
    left_r = rpy_matrix(0.015, -0.02, 0.48)
    right_r = rpy_matrix(-0.01, 0.018, -1.05)
    left_t = np.array([-0.31, 0.43, -0.16])
    right_t = np.array([0.37, -0.4, -0.17])
    data = center_square_session(
        square_measured_in_fanuc_world(left_r, left_t),
        square_measured_in_fanuc_world(right_r, right_t),
    )

    result = tool.solve_session(data)

    assert result["valid"] is True
    assert np.allclose(result["left_arm"]["xyz"], left_t, atol=1e-10)
    assert np.allclose(result["right_arm"]["xyz"], right_t, atol=1e-10)
    assert result["inferred_square"]["side_length_m"] == pytest.approx(0.4)
    assert result["coordinate_contract"]["origin"] == "CENTER"


def test_per_arm_measurement_frame_z_offsets_are_applied():
    left_r = rpy_matrix(0.0, 0.0, 0.35)
    right_r = rpy_matrix(0.0, 0.0, -1.0)
    left_t = np.array([-0.32, 0.41, -0.16])
    right_t = np.array([0.36, -0.39, -0.17])
    left_offset = 0.075
    right_offset = -0.012
    left_raw = square_measured_in_fanuc_world(left_r, left_t)
    right_raw = square_measured_in_fanuc_world(right_r, right_t)
    left_raw[:, 2] -= left_offset
    right_raw[:, 2] -= right_offset
    data = center_square_session(left_raw, right_raw)
    data["eef_z_offset_m"] = {
        "left_arm": left_offset,
        "right_arm": right_offset,
    }

    result = tool.solve_session(data)

    assert result["valid"] is True
    assert np.allclose(result["left_arm"]["xyz"], left_t, atol=1e-10)
    assert np.allclose(result["right_arm"]["xyz"], right_t, atol=1e-10)
    assert result["coordinate_contract"]["eef_z_offset_m"]["left_arm"] == left_offset


def test_bad_point_is_not_activatable():
    rotation = rpy_matrix(0.0, 0.0, 0.5)
    left = measured_in_fanuc_world(rotation, np.array([-0.3, 0.4, -0.15]))
    right = measured_in_fanuc_world(rotation, np.array([0.3, -0.4, -0.15]))
    right[2] += np.array([0.025, 0.0, 0.0])

    result = tool.solve_session(session(left, right))

    assert result["valid"] is False
    assert result["quality"]["arms"]["right_arm"]["max_m"] > 0.006


def test_collinear_table_points_are_rejected():
    points = np.array([[index * 0.1, 0.0, 0.0] for index in range(5)])
    data = session(points.copy(), points.copy())
    data["table_points_m"] = dict(zip(tool.LEGACY_POINT_IDS, points.tolist()))

    with pytest.raises(tool.CalibrationError, match="collinear"):
        tool.solve_session(data)
