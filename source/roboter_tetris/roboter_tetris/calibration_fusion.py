"""Combine two independently measured static base-camera poses without ROS.

The output uses the existing BaseCam extrinsics schema. The weight is an
operator choice, not a statistical estimate of either method's accuracy.
"""

import datetime
import hashlib
import json
import math
import os

import cv2
import numpy as np

from .basecam_extrinsics import (
    ExtrinsicsRecord, record_from_dict, replace_calibration,
    resolve_calibration_path, rotation_angle_deg,
)


DEFAULT_BASE_CAM_FILE = "Extrinsics/base_cam_extrinsics.json"
DEFAULT_ROBOT_CAM_FILE = "/data/robot_cam_handeye_calibration.json"
DEFAULT_OUTPUT_FILE = "/data/base_cam_fused_extrinsics.json"
EXPECTED_FRAME = {"parent": "world", "child": "base camera, color optical frame"}
EXPECTED_CONVENTION = "R = Rz(yaw) @ Ry(pitch) @ Rx(roll); m, deg"


def _read_source(path: str, label: str):
    try:
        with open(path, "rb") as source:
            raw = source.read()
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError("JSON-Wurzel muss ein Objekt sein")
        record = record_from_dict(data)
    except (OSError, UnicodeError, json.JSONDecodeError, AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} ({path}) kann nicht gelesen werden: {exc}") from exc
    if data.get("frame") != EXPECTED_FRAME or data.get("convention") != EXPECTED_CONVENTION:
        raise ValueError(f"{label}: Frame oder Einheit/Winkelkonvention stimmt nicht überein")
    return data, record, hashlib.sha256(raw).hexdigest()


def interpolate_poses(base_pose: np.ndarray, robot_pose: np.ndarray, weight_percent: float) -> np.ndarray:
    """Linear translation and shortest-arc rotation interpolation, 0=base, 100=robot."""
    weight = float(weight_percent)
    if not math.isfinite(weight) or not 0.0 <= weight <= 100.0:
        raise ValueError("robot_cam_weight_percent muss zwischen 0 und 100 liegen")
    if weight == 0.0:
        return base_pose.copy()
    if weight == 100.0:
        return robot_pose.copy()
    fraction = weight / 100.0
    relative = base_pose[:3, :3].T @ robot_pose[:3, :3]
    rotation_vector, _ = cv2.Rodrigues(relative)
    step, _ = cv2.Rodrigues(rotation_vector * fraction)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = base_pose[:3, :3] @ step
    result[:3, 3] = (1.0 - fraction) * base_pose[:3, 3] + fraction * robot_pose[:3, 3]
    return result


def fuse_calibration_files(base_file: str, robot_file: str, output_file: str,
                           robot_cam_weight_percent: float) -> dict:
    """Validate inputs, write a separate BaseCam-readable file, return a summary.

    A failed validation leaves the output untouched. The previous successful
    output is kept as ``*_vorher.json`` when a new result is written.
    """
    weight = float(robot_cam_weight_percent)
    if not math.isfinite(weight) or not 0.0 <= weight <= 100.0:
        raise ValueError("robot_cam_weight_percent muss zwischen 0 und 100 liegen")
    if not all(isinstance(value, str) and value.strip()
               for value in (base_file, robot_file, output_file)):
        raise ValueError("Alle drei Dateipfade müssen gesetzt sein")
    base_path, robot_path, output_path = (
        resolve_calibration_path(value) for value in (base_file, robot_file, output_file)
    )
    base_path, robot_path, output_path = map(os.path.abspath, (base_path, robot_path, output_path))
    backup_path = os.path.splitext(output_path)[0] + "_vorher.json"
    if os.path.islink(output_path) or os.path.islink(backup_path):
        raise ValueError("Ergebnisdatei und Sicherung dürfen keine symbolischen Links sein")
    real_paths = [os.path.realpath(path) for path in (base_path, robot_path, output_path)]
    if len(set(real_paths)) != 3 or os.path.realpath(backup_path) in real_paths[:2]:
        raise ValueError("Quell- und Ergebnisdateien einschließlich Sicherung müssen verschieden sein")

    base_data, base_record, base_hash = _read_source(base_path, "BaseCam-Kalibrierung")
    robot_data, robot_record, robot_hash = _read_source(robot_path, "Robot-Kamera-Kalibrierung")
    if base_record.method not in ("stufe1_basecam", "stufe2_basecam"):
        raise ValueError(
            f"BaseCam-Datei enthält keine abgeschlossene Messung (method={base_record.method!r}); "
            "zuerst die BaseCam-Kalibrierung erfolgreich beenden"
        )
    if (robot_data.get("schema_version") != 3
            or robot_data.get("status") != "validated"
            or robot_record.method != "robot_cam_handeye_charuco"):
        raise ValueError("Robot-Kamera-Datei ist kein validiertes Hand-Auge-Ergebnis im Schema 3")
    transformations = robot_data.get("transformations")
    if not isinstance(transformations, dict):
        raise ValueError("Robot-Kamera-Datei enthält keine Transformationen")
    static_cam = transformations.get("T_world_base_static_cam")
    if not isinstance(static_cam, dict):
        raise ValueError("Robot-Kamera-Datei enthält keine statische Basiskamera-Pose")
    if static_cam.get("source_frame") != "base_camera_frame" or static_cam.get("target_frame") != "world":
        raise ValueError("Robot-Kamera-Datei enthält keine eindeutig benannte statische Basiskamera-Pose")
    try:
        original_matrix = np.asarray(static_cam["homogeneous_matrix"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("T_world_base_static_cam fehlt oder ist ungültig") from exc
    if original_matrix.shape != (4, 4) or not np.allclose(
            original_matrix, robot_record.world_T_cam, atol=1e-6, rtol=0):
        raise ValueError("BaseCam-Matrix und T_world_base_static_cam in Robot-Kamera-Datei widersprechen sich")

    base_pose, robot_pose = base_record.world_T_cam, robot_record.world_T_cam
    combined = interpolate_poses(base_pose, robot_pose, weight)
    position_gap_mm = float(np.linalg.norm(robot_pose[:3, 3] - base_pose[:3, 3]) * 1000.0)
    angle_gap_deg = rotation_angle_deg(base_pose[:3, :3].T @ robot_pose[:3, :3])
    record = ExtrinsicsRecord(
        world_T_cam=combined,
        method="fusion_basecam_robotcam",
        created=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        quality={
            "robot_cam_weight_percent": weight,
            "base_cam_weight_percent": 100.0 - weight,
            "translation_difference_mm": round(position_gap_mm, 3),
            "rotation_difference_deg": round(angle_gap_deg, 4),
            "calculation": "Translation linear in m; Rotation entlang kuerzestem Weg in SO(3)",
            "note": "Gewichtung manuell; keine aus den Quell-Qualitaetswerten abgeleitete Genauigkeit. Nur fuer BaseCam/Pick, keine Referenzmarken-Pruefung.",
            "sources": {
                "base_cam": {"path": base_path, "sha256": base_hash,
                             "method": base_record.method, "created": base_data.get("created")},
                "robot_cam": {"path": robot_path, "sha256": robot_hash,
                              "method": robot_record.method, "created": robot_data.get("created")},
            },
        },
    )
    backup = replace_calibration(output_path, record)
    return {"output_path": output_path, "backup_path": backup,
            "robot_cam_weight_percent": weight,
            "translation_difference_mm": round(position_gap_mm, 3),
            "rotation_difference_deg": round(angle_gap_deg, 4)}
