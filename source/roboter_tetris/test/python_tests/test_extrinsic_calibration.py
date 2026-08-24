"""Unit tests for extrinsic calibration mathematics and JSON generation."""

import json
import math
import os
import tempfile
import numpy as np
import pytest

from roboter_tetris.Calibration.extrinsic_calibration import (
    CalibrationResult, CalibrationSample, matrix_to_pose, pose_to_matrix,
    rotation_matrix_to_rpy, rpy_to_rotation_matrix, save_calibration_json,
    solve_eye_in_hand,
)


def test_rpy_rotation_matrix_roundtrip():
    """Test conversion between RPY angles and 3x3 rotation matrices."""
    roll = math.radians(10.0)
    pitch = math.radians(-15.0)
    yaw = math.radians(45.0)

    R = rpy_to_rotation_matrix(roll, pitch, yaw)

    # Check orthogonality
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-6)
    assert np.abs(np.linalg.det(R) - 1.0) < 1e-6

    # Convert back to RPY
    r_calc, p_calc, y_calc = rotation_matrix_to_rpy(R)

    assert math.degrees(r_calc) == pytest.approx(10.0, abs=1e-3)
    assert math.degrees(p_calc) == pytest.approx(-15.0, abs=1e-3)
    assert math.degrees(y_calc) == pytest.approx(45.0, abs=1e-3)


def test_rotation_matrix_to_quaternion():
    """Test rotation matrix to quaternion conversion."""
    from roboter_tetris.Calibration.extrinsic_calibration import rotation_matrix_to_quaternion
    R_identity = np.eye(3, dtype=np.float64)
    q_id = rotation_matrix_to_quaternion(R_identity)
    np.testing.assert_allclose(q_id, [1.0, 0.0, 0.0, 0.0], atol=1e-6)

    # 90 deg rotation around Z
    R_z90 = rpy_to_rotation_matrix(0, 0, math.pi / 2.0)
    q_z90 = rotation_matrix_to_quaternion(R_z90)
    # expected w = cos(45 deg) = sqrt(0.5), z = sin(45 deg) = sqrt(0.5)
    np.testing.assert_allclose(q_z90, [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)], atol=1e-5)


def test_pose_matrix_conversions():
    """Test conversion between (tvec, rvec) and 4x4 homogeneous matrices."""
    tvec = np.array([0.5, -0.2, 0.8], dtype=np.float64)
    rvec = np.array([0.1, 0.2, -0.3], dtype=np.float64)

    T = pose_to_matrix(tvec, rvec)
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1])

    t_out, r_out = matrix_to_pose(T)
    np.testing.assert_allclose(t_out, tvec, atol=1e-6)
    np.testing.assert_allclose(r_out, rvec, atol=1e-6)


def test_solve_eye_in_hand_synthetic():
    """Test solve_eye_in_hand with synthetic ground-truth poses."""
    # Define ground truth Eye-in-Hand offset (T_ee_cam)
    T_ee_cam_gt = np.eye(4, dtype=np.float64)
    T_ee_cam_gt[:3, 3] = [0.05, -0.03, 0.10]
    T_ee_cam_gt[:3, :3] = rpy_to_rotation_matrix(0.1, -0.05, 0.2)

    # Ground truth Board in Robot Base (T_base_board)
    T_base_board_gt = np.eye(4, dtype=np.float64)
    T_base_board_gt[:3, 3] = [0.6, -0.4, 0.0]

    # Create synthetic samples at different robot EE poses
    samples = []
    ee_angles = [(0, 0, 0), (0.1, -0.1, 0.2), (-0.15, 0.05, -0.1), (0.05, 0.1, 0.15)]
    ee_positions = [[0.4, -0.3, 0.5], [0.45, -0.35, 0.52], [0.38, -0.25, 0.48], [0.42, -0.32, 0.55]]

    for pos, (r, p, y) in zip(ee_positions, ee_angles):
        T_base_ee = np.eye(4, dtype=np.float64)
        T_base_ee[:3, 3] = pos
        T_base_ee[:3, :3] = rpy_to_rotation_matrix(r, p, y)

        # Compute synthetic camera reading: T_cam_board = inv(T_ee_cam) @ inv(T_base_ee) @ T_base_board
        T_cam_board = np.linalg.inv(T_ee_cam_gt) @ np.linalg.inv(T_base_ee) @ T_base_board_gt
        tvec, rvec = matrix_to_pose(T_cam_board)
        robot_cam_board_pose = [*tvec, *rvec]

        sample = CalibrationSample(
            T_robot_ee=T_base_ee,
            robot_cam_board_pose=robot_cam_board_pose
        )
        samples.append(sample)

    res: CalibrationResult = solve_eye_in_hand(samples)

    assert res.sample_count == 4
    np.testing.assert_allclose(res.T_ee_robot_cam, T_ee_cam_gt, atol=1e-3)
    np.testing.assert_allclose(res.T_robot_board[:3, 3], T_base_board_gt[:3, 3], atol=1e-3)


def test_save_calibration_json():
    """Test JSON export formatting and schema compliance."""
    T_dummy = np.eye(4, dtype=np.float64)
    T_dummy[0, 3] = 0.6118
    T_dummy[1, 3] = -0.7820
    T_dummy[2, 3] = 0.8902

    res = CalibrationResult(
        T_robot_base_cam=T_dummy,
        position_rmse_mm=1.23,
        rotation_rmse_deg=0.45,
        sample_count=5
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        json_path = os.path.join(tmpdir, "calibration_test.json")
        save_calibration_json(json_path, res, operator="pytest_unit_test")

        assert os.path.exists(json_path)
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert data["schema_version"] == 2
        assert data["status"] == "validated"
        assert "last_calibrated_at" in data
        assert data["translation_m"]["x"] == 0.6118
        assert data["translation_m"]["y"] == -0.7820
        assert data["translation_m"]["z"] == 0.8902
        assert "transformations" in data
        assert "T_robot_base_cam" in data["transformations"]
        assert data["validation"]["sample_count"] == 5
        assert data["validation"]["position_rmse_mm"] == 1.23
        assert data["validation"]["operator"] == "pytest_unit_test"
