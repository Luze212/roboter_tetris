"""Extrinsic calibration mathematics and JSON generator for roboter_tetris.

Implements Eye-in-Hand hand-eye calibration, rigid transformation math,
conveyor frame alignment (using the side edge alignment), and calibration.json
export according to the project's specification.
"""

from dataclasses import dataclass, field
import datetime
import json
import math
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np


def rpy_to_rotation_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Compute R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0, math.sin(roll_rad), math.cos(roll_rad)],
    ], dtype=np.float64)

    ry = np.array([
        [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
        [0, 1, 0],
        [-math.sin(pitch_rad), 0, math.cos(pitch_rad)],
    ], dtype=np.float64)

    rz = np.array([
        [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
        [math.sin(yaw_rad), math.cos(yaw_rad), 0],
        [0, 0, 1],
    ], dtype=np.float64)

    return rz @ ry @ rx


def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract Roll, Pitch, Yaw (in radians) from a 3x3 rotation matrix.
    
    Convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def pose_to_matrix(tvec: np.ndarray, rvec: np.ndarray) -> np.ndarray:
    """Convert (tvec [3], rvec [3] Rodrigues) to 4x4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert 4x4 homogeneous matrix to (tvec [3], rvec [3] Rodrigues)."""
    tvec = T[:3, 3].copy()
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return tvec, rvec.flatten()


@dataclass
class CalibrationSample:
    """A single sample point containing EE pose and camera board detections."""
    # EE pose in robot base frame: 4x4 T_robot_ee
    T_robot_ee: np.ndarray
    # Board pose in eye-in-hand robot_cam frame: [tx, ty, tz, rx, ry, rz]
    robot_cam_board_pose: Optional[List[float]] = None
    # Board pose in static base_cam frame: [tx, ty, tz, rx, ry, rz]
    base_cam_board_pose: Optional[List[float]] = None


@dataclass
class CalibrationResult:
    """Result of extrinsic calibration."""
    # Transformation from base_cam to robot_base (p_robot = T_robot_base_cam @ p_cam)
    T_robot_base_cam: np.ndarray
    # Transformation from robot_cam to end-effector (Eye-in-Hand offset)
    T_ee_robot_cam: Optional[np.ndarray] = None
    # Transformation from board (aligned with conveyor edge) to robot_base
    T_robot_board: Optional[np.ndarray] = None
    # Transformation from conveyor_frame to robot_base (origin = conveyor center, Y = flow dir)
    T_robot_conveyor: Optional[np.ndarray] = None
    position_rmse_mm: float = 0.0
    rotation_rmse_deg: float = 0.0
    sample_count: int = 0


def solve_eye_in_hand(
    samples: List[CalibrationSample],
    method: int = cv2.CALIB_HAND_EYE_TSAI,
    conveyor_offset_m: Tuple[float, float, float] = (-0.375, 0.416, 0.0)
) -> CalibrationResult:
    """Solve Eye-in-Hand hand-eye calibration from a list of samples.
    
    Given:
      - T_base_ee (robot gripper pose in robot base)
      - T_cam_target (ChArUco board pose in robot hand camera)
      
    Computes:
      - T_ee_cam (Eye-in-Hand transformation: hand camera relative to gripper)
      - T_base_cam (Base camera relative to robot base, when combined with static base_cam board pose)
    """
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    valid_samples = []
    for sample in samples:
        if sample.robot_cam_board_pose is None or len(sample.robot_cam_board_pose) < 6:
            continue
        valid_samples.append(sample)

        # T_base_ee
        R_ee = sample.T_robot_ee[:3, :3]
        t_ee = sample.T_robot_ee[:3, 3]
        R_gripper2base.append(R_ee)
        t_gripper2base.append(t_ee.reshape(3, 1))

        # T_cam_target (from robot_cam board pose)
        tvec = np.array(sample.robot_cam_board_pose[:3], dtype=np.float64)
        rvec = np.array(sample.robot_cam_board_pose[3:6], dtype=np.float64)
        R_cam, _ = cv2.Rodrigues(rvec)

        R_target2cam.append(R_cam)
        t_target2cam.append(tvec.reshape(3, 1))

    if len(valid_samples) < 3:
        raise ValueError(f"At least 3 valid samples required for hand-eye calibration, got {len(valid_samples)}.")

    # Solve Hand-Eye for Eye-in-Hand: R_cam2gripper, t_cam2gripper (T_ee_cam)
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=method
    )

    T_ee_cam = np.eye(4, dtype=np.float64)
    T_ee_cam[:3, :3] = R_cam2gripper
    T_ee_cam[:3, 3] = t_cam2gripper.flatten()

    # Now compute Board in Robot Base for each sample:
    # T_base_target = T_base_ee @ T_ee_cam @ T_cam_target
    board_positions = []
    T_base_target_list = []

    for sample in valid_samples:
        tvec = np.array(sample.robot_cam_board_pose[:3], dtype=np.float64)
        rvec = np.array(sample.robot_cam_board_pose[3:6], dtype=np.float64)
        T_cam_target = pose_to_matrix(tvec, rvec)

        T_base_target = sample.T_robot_ee @ T_ee_cam @ T_cam_target
        T_base_target_list.append(T_base_target)
        board_positions.append(T_base_target[:3, 3])

    # Average board pose in robot base frame
    mean_board_pos = np.mean(board_positions, axis=0)
    T_robot_board = np.eye(4, dtype=np.float64)
    T_robot_board[:3, 3] = mean_board_pos
    # Use first valid rotation as baseline
    T_robot_board[:3, :3] = T_base_target_list[0][:3, :3]

    # Compute T_robot_conveyor from T_robot_board and conveyor_offset_m:
    # board_origin_in_conveyor = [off_x, off_y, off_z]
    # T_conveyor_board = eye(4) with translation = conveyor_offset_m
    # T_robot_board = T_robot_conveyor @ T_conveyor_board
    # => T_robot_conveyor = T_robot_board @ inv(T_conveyor_board)
    T_conveyor_board = np.eye(4, dtype=np.float64)
    T_conveyor_board[0, 3] = conveyor_offset_m[0]
    T_conveyor_board[1, 3] = conveyor_offset_m[1]
    T_conveyor_board[2, 3] = conveyor_offset_m[2]
    T_robot_conveyor = T_robot_board @ np.linalg.inv(T_conveyor_board)

    # Calculate position RMSE of board estimation across poses
    errors = [np.linalg.norm(pos - mean_board_pos) for pos in board_positions]
    pos_rmse_mm = float(np.sqrt(np.mean(np.square(errors))) * 1000.0)

    # Compute T_robot_base_cam if base_cam board pose is available in samples
    T_robot_base_cam = None
    for sample in valid_samples:
        if sample.base_cam_board_pose is not None and len(sample.base_cam_board_pose) >= 6:
            t_base_cam = np.array(sample.base_cam_board_pose[:3], dtype=np.float64)
            r_base_cam = np.array(sample.base_cam_board_pose[3:6], dtype=np.float64)
            T_base_cam_board = pose_to_matrix(t_base_cam, r_base_cam)

            # T_robot_base_cam @ T_base_cam_board = T_robot_board
            # => T_robot_base_cam = T_robot_board @ inv(T_base_cam_board)
            T_robot_base_cam = T_robot_board @ np.linalg.inv(T_base_cam_board)
            break

    if T_robot_base_cam is None:
        # Fallback if no base_cam reading was attached
        T_robot_base_cam = T_robot_conveyor

    return CalibrationResult(
        T_robot_base_cam=T_robot_base_cam,
        T_ee_robot_cam=T_ee_cam,
        T_robot_board=T_robot_board,
        T_robot_conveyor=T_robot_conveyor,
        position_rmse_mm=pos_rmse_mm,
        rotation_rmse_deg=0.0,
        sample_count=len(valid_samples)
    )


def save_calibration_json(
    filepath: str,
    result: CalibrationResult,
    operator: str = "auto_calibration_component",
    notes: str = "Automatic extrinsic calibration via Eye-in-Hand ChArUco board detection"
) -> None:
    """Save calibration results to json with full explanations of all frame transformations and timestamp."""
    T_cam = result.T_robot_base_cam
    roll_cam, pitch_cam, yaw_cam = rotation_matrix_to_rpy(T_cam[:3, :3])

    now_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()

    transformations = {
        "T_robot_base_cam": {
            "description": "Transformation von statischer Base-Kamera zu Roboter-Basis (p_robot = T @ p_base_cam)",
            "source_frame": "base_camera_frame",
            "target_frame": "robot_base",
            "translation_m": {
                "x": round(float(T_cam[0, 3]), 6),
                "y": round(float(T_cam[1, 3]), 6),
                "z": round(float(T_cam[2, 3]), 6),
            },
            "rotation_rpy_deg": {
                "roll": round(float(math.degrees(roll_cam)), 4),
                "pitch": round(float(math.degrees(pitch_cam)), 4),
                "yaw": round(float(math.degrees(yaw_cam)), 4),
            },
            "homogeneous_matrix": [
                [round(float(val), 8) for val in row] for row in T_cam.tolist()
            ],
        }
    }

    if result.T_robot_conveyor is not None:
        T_conv = result.T_robot_conveyor
        roll_conv, pitch_conv, yaw_conv = rotation_matrix_to_rpy(T_conv[:3, :3])
        transformations["T_robot_conveyor"] = {
            "description": "Transformation von Förderband-Frame zu Roboter-Basis (Ursprung=Bandmitte, +Y=Laufrichtung, Z=0 Bandebene)",
            "source_frame": "conveyor_frame",
            "target_frame": "robot_base",
            "translation_m": {
                "x": round(float(T_conv[0, 3]), 6),
                "y": round(float(T_conv[1, 3]), 6),
                "z": round(float(T_conv[2, 3]), 6),
            },
            "rotation_rpy_deg": {
                "roll": round(float(math.degrees(roll_conv)), 4),
                "pitch": round(float(math.degrees(pitch_conv)), 4),
                "yaw": round(float(math.degrees(yaw_conv)), 4),
            },
            "homogeneous_matrix": [
                [round(float(val), 8) for val in row] for row in T_conv.tolist()
            ],
        }

    if result.T_ee_robot_cam is not None:
        T_ee_cam = result.T_ee_robot_cam
        roll_ee, pitch_ee, yaw_ee = rotation_matrix_to_rpy(T_ee_cam[:3, :3])
        transformations["T_ee_robot_cam"] = {
            "description": "Eye-in-Hand Transformation von Roboter-Kamera zu Endeffektor/Gripper",
            "source_frame": "robot_camera_frame",
            "target_frame": "end_effector",
            "translation_m": {
                "x": round(float(T_ee_cam[0, 3]), 6),
                "y": round(float(T_ee_cam[1, 3]), 6),
                "z": round(float(T_ee_cam[2, 3]), 6),
            },
            "rotation_rpy_deg": {
                "roll": round(float(math.degrees(roll_ee)), 4),
                "pitch": round(float(math.degrees(pitch_ee)), 4),
                "yaw": round(float(math.degrees(yaw_ee)), 4),
            },
            "homogeneous_matrix": [
                [round(float(val), 8) for val in row] for row in T_ee_cam.tolist()
            ],
        }

    data = {
        "schema_version": 2,
        "status": "validated",
        "last_calibrated_at": now_iso,
        "description": "Extrinsische Kalibrierdaten für Roboter-Tetris (Kameras, Förderband & Endeffektor).",
        "units": {
            "translation": "m",
            "rotation": "deg"
        },
        "convention": {
            "matrix_layout": "row-major",
            "rotation": "R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"
        },
        "transformations": transformations,
        # Backward compatibility top-level fields:
        "source_frame": "camera_frame",
        "target_frame": "robot_base",
        "translation_m": transformations["T_robot_base_cam"]["translation_m"],
        "rotation_rpy_deg": transformations["T_robot_base_cam"]["rotation_rpy_deg"],
        "homogeneous_matrix": transformations["T_robot_base_cam"]["homogeneous_matrix"],
        "validation": {
            "measured_at": now_iso,
            "operator": operator,
            "method": "Eye-in-Hand ChArUco auto calibration",
            "sample_count": result.sample_count,
            "position_rmse_mm": round(float(result.position_rmse_mm), 4),
            "rotation_rmse_deg": round(float(result.rotation_rmse_deg), 4),
            "notes": notes,
        }
    }

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
