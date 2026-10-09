"""Combine three independent fixed-board hand-eye results.

The ChArUco board may move *between* groups, never within one group. Solving
all samples as one board would violate the fixed-target hand-eye assumption.
"""

import math
from typing import Sequence, Tuple

import cv2
import numpy as np

from .handeye_solver import HandEyeCalibrationResult


def rotation_angle_deg(rotation: np.ndarray) -> float:
    vector, _ = cv2.Rodrigues(rotation)
    return math.degrees(float(np.linalg.norm(vector)))


def mean_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Equal-weight translation mean and intrinsic SO(3) rotation mean."""
    if not transforms:
        raise ValueError("Keine Transformationen zum Mitteln vorhanden")
    matrices = [np.asarray(transform, dtype=np.float64) for transform in transforms]
    for matrix in matrices:
        if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
                or abs(np.linalg.det(matrix[:3, :3]) - 1.0) > 1e-5):
            raise ValueError("Ungültige starre 4x4-Transformation in einem Abschnitt")
    rotation = matrices[0][:3, :3].copy()
    for _ in range(30):
        increments = [cv2.Rodrigues(rotation.T @ matrix[:3, :3])[0].reshape(3)
                      for matrix in matrices]
        step = np.mean(increments, axis=0)
        if float(np.linalg.norm(step)) < 1e-12:
            break
        rotation = rotation @ cv2.Rodrigues(step)[0]
    else:
        raise ValueError("Rotationsmittel konvergiert nicht")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.mean([matrix[:3, 3] for matrix in matrices], axis=0)
    return result


def _largest_pairwise_gap(transforms: Sequence[np.ndarray]) -> Tuple[float, float]:
    translation_mm = 0.0
    rotation_deg = 0.0
    for index, first in enumerate(transforms):
        for second in transforms[index + 1:]:
            translation_mm = max(translation_mm,
                                 float(np.linalg.norm(first[:3, 3] - second[:3, 3]) * 1000.0))
            rotation_deg = max(rotation_deg,
                               rotation_angle_deg(first[:3, :3].T @ second[:3, :3]))
    return translation_mm, rotation_deg


def combine_board_results(results: Sequence[HandEyeCalibrationResult],
                          first_world_T_flange: np.ndarray,
                          max_camera_spread_mm: float,
                          max_camera_spread_deg: float):
    """Return one result plus per-method spread report, or reject inconsistent groups.

    Every board position contributes once, regardless of its detection count.
    Both the flange-mounted robot camera and static base camera must agree
    across positions. A large disagreement is not hidden by their mean.
    """
    if not results:
        raise ValueError("Mindestens eine abgeschlossene Board-Lage ist erforderlich")
    limits = (float(max_camera_spread_mm), float(max_camera_spread_deg))
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("Streuungsgrenzen müssen endlich und positiv sein")
    flange_cam, static_cam = [], []
    for index, result in enumerate(results, 1):
        if result.T_ee_robot_cam is None or result.T_robot_base_static_cam is None:
            raise ValueError(f"Board-Lage {index}: Hand-Auge- oder Basiskamera-Pose fehlt")
        flange_cam.append(np.asarray(result.T_ee_robot_cam, dtype=np.float64))
        static_cam.append(np.asarray(result.T_robot_base_static_cam, dtype=np.float64))
    # Validate every source matrix before calculating spreads or an average.
    mean_flange_cam = mean_transforms(flange_cam)
    mean_static_cam = mean_transforms(static_cam)
    report = {}
    for label, matrices in (("flange_robot_cam", flange_cam),
                            ("world_base_static_cam", static_cam)):
        position_mm, rotation_deg = _largest_pairwise_gap(matrices)
        report[label] = {"max_pairwise_translation_mm": round(position_mm, 4),
                         "max_pairwise_rotation_deg": round(rotation_deg, 5)}
        if position_mm > limits[0] or rotation_deg > limits[1]:
            raise ValueError(
                f"{label}: {len(results)} Board-Lagen widersprechen sich um bis zu "
                f"{position_mm:.2f} mm / {rotation_deg:.3f} Grad; erlaubt sind "
                f"{limits[0]:.2f} mm / {limits[1]:.3f} Grad"
            )
    first_flange = np.asarray(first_world_T_flange, dtype=np.float64)
    mean_transforms([first_flange])  # validate the reference pose as well
    first_world_T_robot_cam = first_flange @ mean_flange_cam
    result = HandEyeCalibrationResult(
        T_robot_base_cam=first_world_T_robot_cam,
        T_ee_robot_cam=mean_flange_cam,
        T_robot_board=None,
        T_robot_conveyor=None,
        T_robot_base_static_cam=mean_static_cam,
        # The JSON's legacy RMSE fields remain conservative: the worst group,
        # not a fictitious RMSE from pooling three different board poses.
        position_rmse_mm=max(item.position_rmse_mm for item in results),
        rotation_rmse_deg=max(item.rotation_rmse_deg for item in results),
        flange_rotation_span_deg=min(item.flange_rotation_span_deg for item in results),
        sample_count=sum(item.sample_count for item in results),
    )
    report["group_count"] = len(results)
    report["samples_per_group"] = [item.sample_count for item in results]
    report["aggregation"] = "equal weight per board position; translation arithmetic, rotation geodesic mean"
    return result, report
