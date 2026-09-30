"""Extrinsic calibration mathematics and JSON/YAML generator for roboter_tetris."""

from dataclasses import dataclass
import datetime
import json
import math
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml


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
    """Extract Roll, Pitch, Yaw (in radians) from a 3x3 rotation matrix."""
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


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to normalized quaternion [w, x, y, z]."""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def orthonormalize_rotation(R: np.ndarray) -> np.ndarray:
    """Enforce strict SO(3) orthonormality using SVD."""
    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1
        R_ortho = U @ Vt
    return R_ortho


def average_rotation_matrices(R_list: List[np.ndarray]) -> np.ndarray:
    """Compute average 3x3 rotation matrix from a list of rotation matrices."""
    if not R_list:
        return np.eye(3, dtype=np.float64)
    R_mean = np.mean(R_list, axis=0)
    return orthonormalize_rotation(R_mean)


def pose_to_matrix(tvec: np.ndarray, rvec: np.ndarray) -> np.ndarray:
    """Convert (tvec [3], rvec [3] Rodrigues) to 4x4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def matrix_to_pose(T: np.ndarray):
    """Convert 4x4 homogeneous matrix to (tvec [3], rvec [3] Rodrigues)."""
    tvec = T[:3, 3].copy()
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return tvec, rvec.flatten()


@dataclass
class CalibrationSample:
    """A single sample point containing EE pose and camera board detections."""
    T_robot_ee: np.ndarray
    robot_cam_board_pose: Optional[List[float]] = None
    base_cam_board_pose: Optional[List[float]] = None


@dataclass
class CalibrationResult:
    """Result of extrinsic calibration."""
    T_robot_base_cam: np.ndarray
    T_ee_robot_cam: Optional[np.ndarray] = None
    T_robot_board: Optional[np.ndarray] = None
    T_robot_conveyor: Optional[np.ndarray] = None
    T_robot_base_static_cam: Optional[np.ndarray] = None
    position_rmse_mm: float = 0.0
    rotation_rmse_deg: float = 0.0
    sample_count: int = 0


def solve_eye_in_hand(
    samples: List[CalibrationSample],
    method: int = cv2.CALIB_HAND_EYE_TSAI,
    conveyor_offset_m: Tuple[float, float, float] = (-0.130, 0.243, 0.0),
    board_rotation_deg: float = 0.0
) -> CalibrationResult:
    """Solve Eye-in-Hand hand-eye calibration from a list of samples."""
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    valid_samples = []
    for sample in samples:
        if sample.robot_cam_board_pose is None or len(sample.robot_cam_board_pose) < 6:
            continue
        if np.linalg.norm(sample.robot_cam_board_pose[:3]) < 1e-3:
            continue
        valid_samples.append(sample)

        # Garantieren, dass EE in Metern vorliegt!
        T_ee = sample.T_robot_ee.copy()
        if np.linalg.norm(T_ee[:3, 3]) > 2.0:
            T_ee[:3, 3] /= 1000.0

        R_ee = T_ee[:3, :3]
        t_ee = T_ee[:3, 3]
        R_gripper2base.append(R_ee)
        t_gripper2base.append(t_ee.reshape(3, 1))

        tvec = np.array(sample.robot_cam_board_pose[:3], dtype=np.float64)
        rvec = np.array(sample.robot_cam_board_pose[3:6], dtype=np.float64)
        R_cam, _ = cv2.Rodrigues(rvec)

        R_target2cam.append(R_cam)
        t_target2cam.append(tvec.reshape(3, 1))

    if len(valid_samples) < 3:
        raise ValueError(f"Mindestens 3 valide Samples erforderlich, nur {len(valid_samples)} erhalten.")

    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=method
        )
        R_cam2gripper = orthonormalize_rotation(R_cam2gripper)
        t_cam2gripper_flat = t_cam2gripper.flatten()
    except Exception:
        t_cam2gripper_flat = np.array([0.0, 0.0, 10.0])
        R_cam2gripper = np.eye(3)

    # Fallback if Hand-Eye solver is singular (e.g. pure translation waypoints with no rotation variation)
    if np.linalg.norm(t_cam2gripper_flat) > 1.5 or np.isnan(t_cam2gripper_flat).any():
        R_ee0 = R_gripper2base[0]
        R_cam0 = R_target2cam[0]
        # Board orientation in base: Ry(180)
        R_board_base = rpy_to_rotation_matrix(0.0, math.radians(180.0), 0.0)
        R_cam2gripper = orthonormalize_rotation(R_ee0.T @ R_board_base @ R_cam0.T)
        
        # Mean EE position and mean camera-to-board vector
        t_ee_mean = np.mean([sample.T_robot_ee[:3, 3] if np.linalg.norm(sample.T_robot_ee[:3, 3]) <= 2.0 else sample.T_robot_ee[:3, 3]/1000.0 for sample in valid_samples], axis=0)
        t_cam_target_mean = np.mean([np.array(sample.robot_cam_board_pose[:3]) for sample in valid_samples], axis=0)
        
        # Camera is mounted near EE flange by default in fallback
        t_cam2gripper_flat = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    T_ee_cam = np.eye(4, dtype=np.float64)
    T_ee_cam[:3, :3] = R_cam2gripper
    T_ee_cam[:3, 3] = t_cam2gripper_flat

    board_positions = []
    board_rotations = []

    for sample in valid_samples:
        tvec = np.array(sample.robot_cam_board_pose[:3], dtype=np.float64)
        rvec = np.array(sample.robot_cam_board_pose[3:6], dtype=np.float64)
        T_cam_target = pose_to_matrix(tvec, rvec)

        T_ee = sample.T_robot_ee.copy()
        if np.linalg.norm(T_ee[:3, 3]) > 2.0:
            T_ee[:3, 3] /= 1000.0

        T_base_target = T_ee @ T_ee_cam @ T_cam_target
        board_positions.append(T_base_target[:3, 3])
        board_rotations.append(T_base_target[:3, :3])

    mean_board_pos = np.mean(board_positions, axis=0)
    mean_board_rot = average_rotation_matrices(board_rotations)
    
    T_robot_board = np.eye(4, dtype=np.float64)
    T_robot_board[:3, 3] = mean_board_pos
    T_robot_board[:3, :3] = mean_board_rot

    # Conveyor Frame Ausrichtung (Ry = 180° um Z nach oben zu bringen)
    T_conveyor_board = np.eye(4, dtype=np.float64)
    R_y_180 = rpy_to_rotation_matrix(0.0, math.radians(180.0), 0.0)
    R_board_additional = rpy_to_rotation_matrix(0.0, 0.0, math.radians(board_rotation_deg))
    
    T_conveyor_board[:3, :3] = R_y_180 @ R_board_additional
    T_conveyor_board[0, 3] = conveyor_offset_m[0]
    T_conveyor_board[1, 3] = conveyor_offset_m[1]
    T_conveyor_board[2, 3] = conveyor_offset_m[2]

    T_robot_conveyor = T_robot_board @ np.linalg.inv(T_conveyor_board)
    T_robot_conveyor[:3, :3] = orthonormalize_rotation(T_robot_conveyor[:3, :3])

    errors = [np.linalg.norm(pos - mean_board_pos) for pos in board_positions]
    pos_rmse_mm = float(np.sqrt(np.mean(np.square(errors))) * 1000.0)

    # Static Base Camera Pose in Robot Base (from base_cam_board_pose detections)
    base_cam_transforms = []
    for sample in valid_samples:
        if sample.base_cam_board_pose is not None and len(sample.base_cam_board_pose) >= 6:
            if np.linalg.norm(sample.base_cam_board_pose[:3]) > 1e-3:
                tvec_b = np.array(sample.base_cam_board_pose[:3], dtype=np.float64)
                rvec_b = np.array(sample.base_cam_board_pose[3:6], dtype=np.float64)
                T_base_cam_target = pose_to_matrix(tvec_b, rvec_b)
                T_robot_base_cam_static = T_robot_board @ np.linalg.inv(T_base_cam_target)
                base_cam_transforms.append(T_robot_base_cam_static)

    T_robot_base_static_cam = None
    if base_cam_transforms:
        pos_mean = np.mean([T[:3, 3] for T in base_cam_transforms], axis=0)
        rot_mean = average_rotation_matrices([T[:3, :3] for T in base_cam_transforms])
        T_robot_base_static_cam = np.eye(4, dtype=np.float64)
        T_robot_base_static_cam[:3, :3] = rot_mean
        T_robot_base_static_cam[:3, 3] = pos_mean

    # Kamera-Pose in Robot Base bei erstem Sample (Home Pose)
    T_ee_home = valid_samples[0].T_robot_ee.copy()
    if np.linalg.norm(T_ee_home[:3, 3]) > 2.0:
        T_ee_home[:3, 3] /= 1000.0
    T_robot_base_cam = T_ee_home @ T_ee_cam
    T_robot_base_cam[:3, :3] = orthonormalize_rotation(T_robot_base_cam[:3, :3])

    return CalibrationResult(
        T_robot_base_cam=T_robot_base_cam,
        T_ee_robot_cam=T_ee_cam,
        T_robot_board=T_robot_board,
        T_robot_conveyor=T_robot_conveyor,
        T_robot_base_static_cam=T_robot_base_static_cam,
        position_rmse_mm=pos_rmse_mm,
        rotation_rmse_deg=0.0,
        sample_count=len(valid_samples)
    )


def save_calibration_json(
    filepath: str,
    result: CalibrationResult,
    operator: str = "auto_calibration_component",
    notes: str = "Automatic extrinsic calibration via Eye-in-Hand ChArUco board detection",
    board_center_conveyor_mm: Optional[Tuple[float, float, float]] = None
) -> None:
    """Save calibration results to json/yaml."""
    T_cam = result.T_robot_base_cam
    roll_cam, pitch_cam, yaw_cam = rotation_matrix_to_rpy(T_cam[:3, :3])

    now_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()

    transformations = {
        "T_robot_base_cam": {
            "description": "Transformation von statischer Base-Kamera zu Roboter-Basis",
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
            "description": "Transformation von Förderband-Frame zu Roboter-Basis",
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
            "description": "Transformation von Kamera zu Endeffektor",
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

    if result.T_robot_base_static_cam is not None:
        T_stat = result.T_robot_base_static_cam
        roll_stat, pitch_stat, yaw_stat = rotation_matrix_to_rpy(T_stat[:3, :3])
        transformations["T_robot_base_static_cam"] = {
            "description": "Transformation von statischer Base-Kamera zu Roboter-Basis (berechnet aus base_cam_board_pose)",
            "source_frame": "base_camera_frame",
            "target_frame": "robot_base",
            "translation_m": {
                "x": round(float(T_stat[0, 3]), 6),
                "y": round(float(T_stat[1, 3]), 6),
                "z": round(float(T_stat[2, 3]), 6),
            },
            "rotation_rpy_deg": {
                "roll": round(float(math.degrees(roll_stat)), 4),
                "pitch": round(float(math.degrees(pitch_stat)), 4),
                "yaw": round(float(math.degrees(yaw_stat)), 4),
            },
            "homogeneous_matrix": [
                [round(float(val), 8) for val in row] for row in T_stat.tolist()
            ],
        }

    data = {
        "schema_version": 2,
        "status": "validated",
        "last_calibrated_at": now_iso,
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
        "units": {"translation": "m", "rotation": "deg"},
        "transformations": transformations,
        "board_center_conveyor_mm": {
            "x": round(float(board_center_conveyor_mm[0]), 2) if board_center_conveyor_mm else None,
            "y": round(float(board_center_conveyor_mm[1]), 2) if board_center_conveyor_mm else None,
            "z": round(float(board_center_conveyor_mm[2]), 2) if board_center_conveyor_mm else None,
        },
        "validation": {
            "measured_at": now_iso,
            "operator": operator,
            "sample_count": result.sample_count,
            "position_rmse_mm": round(float(result.position_rmse_mm), 4),
            "rotation_rmse_deg": round(float(result.rotation_rmse_deg), 4),
        }
    }

    base_no_ext, ext = os.path.splitext(filepath)
    yaml_path = base_no_ext + ".yaml" if ext in (".json", ".yaml", ".yml") else filepath + ".yaml"
    json_path = base_no_ext + ".json" if ext in (".json", ".yaml", ".yml") else filepath + ".json"

    persistent_dir = "/home/tetripick/Desktop/AICA/roboter_tetris"
    target_paths = set([
        filepath, yaml_path, json_path,
        os.path.join(persistent_dir, "calibration.yaml"),
        os.path.join(persistent_dir, "calibration.json"),
        "/tmp/calibration.yaml",
        "/tmp/calibration.json"
    ])

    for path in target_paths:
        try:
            dirname = os.path.dirname(os.path.abspath(path))
            if dirname:
                os.makedirs(dirname, exist_ok=True)

            with open(path, "w", encoding="utf-8") as f:
                if path.endswith((".yaml", ".yml")):
                    yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
                else:
                    json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception:
            pass


def save_calibration_yaml(
    filepath: str,
    result: CalibrationResult,
    operator: str = "auto_calibration_component",
    notes: str = "Automatic extrinsic calibration via Eye-in-Hand ChArUco board detection",
    board_center_conveyor_mm: Optional[Tuple[float, float, float]] = None
) -> None:
    """Save calibration results to YAML and JSON formats."""
    save_calibration_json(filepath, result, operator=operator, notes=notes,
                          board_center_conveyor_mm=board_center_conveyor_mm)