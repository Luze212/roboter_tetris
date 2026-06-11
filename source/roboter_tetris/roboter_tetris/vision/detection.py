"""Conveyor-object detection from an aligned color/depth frame pair.

Port of ``FuE_Greifen-main/cameras/camera_static.cpp::processFrames`` (read-only
reference), vectorized: the C++ per-pixel deprojection loops collapse to numpy
mask operations, since for color-aligned depth the depth value *is* the camera-z.

Pure numpy/cv2 — no ROS imports. Inputs: BGR color image, depth image in mm
(float32), camera intrinsics. Output: detections in the robot frame (mm), as
:class:`~roboter_tetris.vision.tracker.TrackedObject` plus pixel-space drawing
info for the debug image.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .color_estimation import estimate_object_color_id
from .tracker import TrackedObject

# Constants from the C++ original
BORDER_MARGIN_PX = 2
HEIGHT_BIAS_MM = 20.0
NEAR_SQUARE_ASPECT = 0.92


@dataclass
class DetectionParams:
    """All tunables; defaults reproduce the behaviour of the C++ binary.

    Note: ``z_offset``, ``x_scale``/``x_offset_mm`` (affine correction) and the
    ``search_area`` filter existed in the old config file but were *not* applied
    by the C++ code — they default to neutral here and only act when set.
    """

    roi: Tuple[int, int, int, int] = (0, 0, 0, 0)  # x, y, w, h; zero area = full frame
    conveyor_z_dist: float = 865.0       # mm, camera to conveyor plane
    min_obj_height: float = 15.0         # mm
    max_obj_height_mm: float = 150.0     # mm
    z_offset: float = 0.0                # mm, conveyor reflection margin
    min_contour_area: float = 500.0      # px
    cam_to_robot: Optional[np.ndarray] = None  # 4x4 homogeneous, meters
    x_scale: float = 1.0
    y_scale: float = 1.0
    x_offset_mm: float = 0.0
    y_offset_mm: float = 0.0
    search_area_y_min: float = -1.0e9    # mm, robot frame; wide open = off
    search_area_y_max: float = 1.0e9


@dataclass
class DebugInfo:
    """Pixel-space drawing data for one detection (full-frame coordinates)."""

    box_px: np.ndarray = field(default_factory=lambda: np.zeros((4, 2), dtype=np.int32))
    center_px: Tuple[int, int] = (0, 0)


def build_cam_to_robot(x_m: float, y_m: float, z_m: float,
                       roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """4x4 homogeneous camera→robot transform; R = Rz(yaw)·Ry(pitch)·Rx(roll),
    angles in degrees, translation in meters (port of genTransformCamToRobot)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    yv = math.radians(yaw_deg)
    rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)],
                   [0, 1, 0],
                   [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(yv), -math.sin(yv), 0],
                   [math.sin(yv), math.cos(yv), 0],
                   [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = (x_m, y_m, z_m)
    return t


def normalize_orientation_half_turn(angle_rad: float) -> float:
    """Objects are 180°-symmetric; keep the angle in [0, pi)."""
    if not math.isfinite(angle_rad):
        return float("nan")
    while angle_rad >= math.pi:
        angle_rad -= math.pi
    while angle_rad < 0.0:
        angle_rad += math.pi
    return angle_rad


def compute_robust_orientation_2d(box_px: np.ndarray, corners_xy: np.ndarray) -> float:
    """Long-axis orientation from the midpoints of the two short edges.

    ``box_px``: 4x2 minAreaRect corner pixels (decides which edges are short);
    ``corners_xy``: 4x2 corresponding robot-frame x/y coordinates.
    Port of computeRobustOrientation2D (camera_static.hpp).
    """
    edge01 = float(np.sum((box_px[1] - box_px[0]) ** 2))
    edge12 = float(np.sum((box_px[2] - box_px[1]) ** 2))

    if edge01 >= edge12:
        short_a, short_b, long_edge = (1, 2), (3, 0), (0, 1)
    else:
        short_a, short_b, long_edge = (0, 1), (2, 3), (1, 2)

    mid_a = 0.5 * (corners_xy[short_a[0]] + corners_xy[short_a[1]])
    mid_b = 0.5 * (corners_xy[short_b[0]] + corners_xy[short_b[1]])
    axis = mid_b - mid_a
    if np.all(np.isfinite(mid_a)) and np.all(np.isfinite(mid_b)) \
            and float(axis[0] ** 2 + axis[1] ** 2) > 1e-12:
        return normalize_orientation_half_turn(math.atan2(float(axis[1]), float(axis[0])))

    delta = corners_xy[long_edge[1]] - corners_xy[long_edge[0]]
    if np.all(np.isfinite(delta)) and float(delta[0] ** 2 + delta[1] ** 2) > 1e-12:
        return normalize_orientation_half_turn(math.atan2(float(delta[1]), float(delta[0])))
    return float("nan")


def detect_objects(color_bgr: np.ndarray, depth_mm: np.ndarray,
                   fx: float, fy: float, cx: float, cy: float,
                   params: DetectionParams):
    """Detect conveyor objects in one frame.

    Returns ``(detections, debug_infos)`` — parallel lists of
    :class:`TrackedObject` (robot frame, mm) and :class:`DebugInfo`.
    """
    full_h, full_w = depth_mm.shape[:2]
    rx0, ry0, rw, rh = params.roi
    if rw <= 0 or rh <= 0:
        rx0, ry0, rw, rh = 0, 0, full_w, full_h
    rx0 = max(0, min(rx0, full_w - 1))
    ry0 = max(0, min(ry0, full_h - 1))
    rw = min(rw, full_w - rx0)
    rh = min(rh, full_h - ry0)

    depth_roi = depth_mm[ry0:ry0 + rh, rx0:rx0 + rw]
    color_roi = color_bgr[ry0:ry0 + rh, rx0:rx0 + rw]
    hsv_roi = cv2.cvtColor(color_roi, cv2.COLOR_BGR2HSV)

    # Vectorized conveyor-height mask (the C++ per-pixel deprojection loop:
    # for aligned depth, point[2] == depth value, so depth thresholds suffice).
    upper = params.conveyor_z_dist - params.min_obj_height - params.z_offset
    lower = params.conveyor_z_dist - params.max_obj_height_mm
    mask = ((depth_roi > 0) & (depth_roi < upper) & (depth_roi > lower)).astype(np.uint8) * 255

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    transform = params.cam_to_robot if params.cam_to_robot is not None else np.eye(4)

    detections: List[TrackedObject] = []
    debug_infos: List[DebugInfo] = []

    for cnt in contours:
        if cv2.contourArea(cnt) < params.min_contour_area:
            continue

        # Partial objects touching the ROI border have unreliable size estimates.
        bx, by, bw, bh = cv2.boundingRect(cnt)
        if (bx <= BORDER_MARGIN_PX or by <= BORDER_MARGIN_PX
                or bx + bw >= rw - BORDER_MARGIN_PX or by + bh >= rh - BORDER_MARGIN_PX):
            continue

        rrect = cv2.minAreaRect(cnt)
        box_px_roi = cv2.boxPoints(rrect)  # 4x2 float, ROI coords

        # Deproject the 4 box corners using the depth at each corner pixel.
        corners_cam_m = np.full((4, 3), np.nan)
        for k in range(4):
            px = rx0 + int(round(box_px_roi[k][0]))
            py = ry0 + int(round(box_px_roi[k][1]))
            if px < 0 or py < 0 or px >= full_w or py >= full_h:
                continue
            d_mm = float(depth_mm[py, px])
            if d_mm <= 0.0:
                continue
            d_m = d_mm / 1000.0
            corners_cam_m[k] = ((px - cx) * d_m / fx, (py - cy) * d_m / fy, d_m)
        if not np.all(np.isfinite(corners_cam_m)):
            # C++ propagates NaN corners into a NaN center, which the final
            # validation drops — requiring all four corners is equivalent.
            continue

        # Camera → robot frame (homogeneous, meters).
        ones = np.ones((4, 1))
        corners_robot_m = (transform @ np.hstack([corners_cam_m, ones]).T).T[:, :3]

        center_m = corners_robot_m.mean(axis=0)

        # Mean object-top depth over the contour (bbox crop, not full frame).
        c_mask = np.zeros((bh, bw), dtype=np.uint8)
        cv2.drawContours(c_mask, [cnt - (bx, by)], -1, 255, cv2.FILLED)
        depth_patch = depth_roi[by:by + bh, bx:bx + bw]
        valid = depth_patch[(c_mask == 255) & (depth_patch > 0)]
        if valid.size == 0:
            continue
        mean_z_mm = float(valid.mean())
        height_mm = params.conveyor_z_dist - mean_z_mm + HEIGHT_BIAS_MM

        # 3D edge lengths between robot-frame corners (mm), long side first.
        length_mm = float(np.linalg.norm(corners_robot_m[0] - corners_robot_m[1])) * 1000.0
        width_mm = float(np.linalg.norm(corners_robot_m[1] - corners_robot_m[2])) * 1000.0
        if length_mm < width_mm:
            length_mm, width_mm = width_mm, length_mm

        orientation = compute_robust_orientation_2d(
            np.asarray(box_px_roi), corners_robot_m[:, :2])

        # Color: patch vote around the rect center (fallback: contour centroid).
        center_px_roi = (int(round(rrect[0][0])), int(round(rrect[0][1])))
        if cv2.pointPolygonTest(cnt, (float(center_px_roi[0]), float(center_px_roi[1])), False) < 0.0:
            mu = cv2.moments(cnt)
            if abs(mu["m00"]) > 1e-6:
                center_px_roi = (int(round(mu["m10"] / mu["m00"])),
                                 int(round(mu["m01"] / mu["m00"])))
        color_id = estimate_object_color_id(hsv_roi, cnt, center_px_roi)

        # Near-square objects get their orientation frozen by the tracker.
        w_r, h_r = rrect[1]
        square = False
        if w_r > 1e-6 and h_r > 1e-6:
            square = (min(w_r, h_r) / max(w_r, h_r)) >= NEAR_SQUARE_ASPECT

        x_mm = center_m[0] * 1000.0 * params.x_scale + params.x_offset_mm
        y_mm = center_m[1] * 1000.0 * params.y_scale + params.y_offset_mm
        z_mm = center_m[2] * 1000.0 + height_mm / 2.0

        if not (math.isfinite(x_mm) and math.isfinite(y_mm)
                and math.isfinite(z_mm) and math.isfinite(orientation)):
            continue
        if not (params.search_area_y_min <= y_mm <= params.search_area_y_max):
            continue

        detections.append(TrackedObject(
            id=0, color=color_id, x=x_mm, y=y_mm, z=z_mm,
            orientation=orientation, vy=0.0,
            length=length_mm, width=width_mm, height=height_mm, square=square,
        ))
        debug_infos.append(DebugInfo(
            box_px=np.round(box_px_roi + (rx0, ry0)).astype(np.int32),
            center_px=(center_px_roi[0] + rx0, center_px_roi[1] + ry0),
        ))

    return detections, debug_infos
