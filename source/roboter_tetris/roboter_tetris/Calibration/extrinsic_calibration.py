"""Extrinsic calibration mathematics and JSON export for roboter_tetris."""

from dataclasses import dataclass
import datetime
import json
import math
import os
import tempfile
from typing import List, Optional, Tuple

import cv2
import numpy as np


MIN_HAND_EYE_SAMPLES = 4
MIN_HAND_EYE_ROTATION_DEG = 5.0
MIN_HAND_EYE_AXIS_SEPARATION_DEG = 15.0
MAX_FLANGE_CAMERA_DISTANCE_M = 0.5


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
    flange_rotation_span_deg: float = 0.0
    sample_count: int = 0


def _rotation_angle_deg(R: np.ndarray) -> float:
    """Return the principal rotation angle of a rotation matrix in degrees."""
    cosine = (float(np.trace(R)) - 1.0) / 2.0
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def _validate_hand_eye_motion(rotations: List[np.ndarray]) -> float:
    """Reject hand-eye sample sets without enough rotational excitation."""
    relative_axes = []
    largest_angle_deg = 0.0

    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            R_relative = rotations[i].T @ rotations[j]
            angle_deg = _rotation_angle_deg(R_relative)
            largest_angle_deg = max(largest_angle_deg, angle_deg)
            if angle_deg >= MIN_HAND_EYE_ROTATION_DEG:
                rvec, _ = cv2.Rodrigues(R_relative)
                axis = rvec.flatten()
                axis_norm = np.linalg.norm(axis)
                if axis_norm > 1e-12:
                    relative_axes.append(axis / axis_norm)

    if largest_angle_deg < MIN_HAND_EYE_ROTATION_DEG:
        raise ValueError(
            "Hand-Eye-Kalibrierung unbestimmt: Die Flanschposen enthalten weniger als "
            f"{MIN_HAND_EYE_ROTATION_DEG:.1f} Grad Rotationsänderung."
        )

    largest_axis_separation_deg = 0.0
    for i in range(len(relative_axes)):
        for j in range(i + 1, len(relative_axes)):
            # An axis and its inverse describe the same physical rotation axis.
            cosine = abs(float(np.dot(relative_axes[i], relative_axes[j])))
            separation_deg = math.degrees(math.acos(min(1.0, max(-1.0, cosine))))
            largest_axis_separation_deg = max(largest_axis_separation_deg, separation_deg)

    if largest_axis_separation_deg < MIN_HAND_EYE_AXIS_SEPARATION_DEG:
        raise ValueError(
            "Hand-Eye-Kalibrierung unbestimmt: Die Flanschrotationen erfolgen nur um "
            "eine nahezu gemeinsame Achse."
        )

    return largest_angle_deg


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

    if len(valid_samples) < MIN_HAND_EYE_SAMPLES:
        raise ValueError(
            f"Mindestens {MIN_HAND_EYE_SAMPLES} valide Samples erforderlich, "
            f"nur {len(valid_samples)} erhalten."
        )

    flange_rotation_span_deg = _validate_hand_eye_motion(R_gripper2base)

    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=method
        )
    except cv2.error as exc:
        raise ValueError(f"OpenCV Hand-Eye-Solver fehlgeschlagen: {exc}") from exc

    if not np.isfinite(R_cam2gripper).all() or not np.isfinite(t_cam2gripper).all():
        raise ValueError("Hand-Eye-Solver lieferte nicht-endliche Werte.")

    R_cam2gripper = orthonormalize_rotation(R_cam2gripper)
    t_cam2gripper_flat = t_cam2gripper.flatten()
    if np.linalg.norm(t_cam2gripper_flat) > MAX_FLANGE_CAMERA_DISTANCE_M:
        raise ValueError(
            "Hand-Eye-Solver lieferte einen unplausiblen Flansch-Kamera-Abstand von "
            f"{np.linalg.norm(t_cam2gripper_flat) * 1000.0:.1f} mm."
        )

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
    rotation_errors_deg = [
        _rotation_angle_deg(mean_board_rot.T @ rotation) for rotation in board_rotations
    ]
    rotation_rmse_deg = float(np.sqrt(np.mean(np.square(rotation_errors_deg))))

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
        rotation_rmse_deg=rotation_rmse_deg,
        flange_rotation_span_deg=flange_rotation_span_deg,
        sample_count=len(valid_samples)
    )


def save_calibration_json(
    filepath: str,
    result: CalibrationResult,
    operator: str = "auto_calibration_component",
    notes: str = "Automatic extrinsic calibration via Eye-in-Hand ChArUco board detection",
    board_center_conveyor_mm: Optional[Tuple[float, float, float]] = None
) -> None:
    """Atomically replace only the configured JSON file; propagate write errors."""
    if not os.path.isabs(filepath) or not filepath.lower().endswith(".json"):
        raise ValueError("calibration_file_path muss ein absoluter Pfad zu einer .json-Datei sein.")
    # This is the moving robot camera pose at the first sample, expressed in
    # world. It is diagnostic only; the reusable hand-eye result is below.
    T_cam = result.T_robot_base_cam
    roll_cam, pitch_cam, yaw_cam = rotation_matrix_to_rpy(T_cam[:3, :3])

    now_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()

    transformations = {
        "T_world_robot_cam_at_first_sample": {
            "description": "Pose der bewegten Roboterkamera beim ersten Sample in world; nur Diagnose, nicht für den Pick-Betrieb",
            "source_frame": "robot_camera_frame",
            "target_frame": "world",
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
        transformations["T_world_conveyor"] = {
            "description": "Transformation vom Förderband-Frame in den globalen Roboter-Frame world",
            "source_frame": "conveyor_frame",
            "target_frame": "world",
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
        transformations["T_flange_robot_cam"] = {
            "description": "Transformation von der Roboterkamera zum Roboterflansch ur_tool0",
            "source_frame": "robot_camera_frame",
            "target_frame": "ur_tool0",
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
        transformations["T_world_base_static_cam"] = {
            "description": "Transformation von der statischen Basiskamera in den globalen Roboter-Frame world (aus base_cam_board_pose)",
            "source_frame": "base_camera_frame",
            "target_frame": "world",
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
        "schema_version": 3,
        "status": "validated",
        "last_calibrated_at": now_iso,
        "units": {"translation": "m", "rotation": "deg"},
        "frame_convention": {
            "global_frame": "world",
            "global_frame_definition": "AICA ur_base_link am Roboterfuß. Gegenüber dem UR-Frame base ist world um 180 Grad um z gedreht; innerhalb von AICA wird keine Umrechnung vorgenommen.",
            "flange_frame": "ur_tool0",
            "robot_ee_pose_requirement": "world_T_ur_tool0: Flanschpose vom Hardware-State, keine TCP- oder Greifpunktpose.",
            "matrix_notation": "T_target_source transformiert Punkte von source_frame nach target_frame.",
        },
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
            "flange_rotation_span_deg": round(float(result.flange_rotation_span_deg), 4),
        }
    }

    # Resolve host-side symlinks as well, so replacement preserves the link.
    target = os.path.realpath(filepath)
    dirname = os.path.dirname(target)
    os.makedirs(dirname, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=dirname,
            prefix=".calibration-", suffix=".tmp", delete=False
        ) as f:
            temporary_path = f.name
            json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
            # Calibration results must also be readable from the host project.
            os.fchmod(f.fileno(), 0o644)
        os.replace(temporary_path, target)
        temporary_path = None
    finally:
        if temporary_path is not None:
            os.unlink(temporary_path)
