"""Endeffector-camera fine-localization of the object under the gripper.

Port of ``FuE_Greifen-main/cameras/camera_robot.cpp::processFrames`` (read-only
reference). Pure numpy/cv2 — no ROS imports, unit-testable with synthetic depth.

The depth sensor cannot measure the object itself (too close -> reads 0). The
object therefore appears as a *hole* (depth == 0) which is segmented; the actual
distance is sampled on the conveyor belt just below the object's bottom edge and
used to back-project the reference pixel to metric x/y.

Geometry caveat (handled downstream, not here): in ``centroid`` mode the
reference pixel lies *on* the object (closer than the belt) but is back-projected
with the belt distance, so x/y carry a scale error ``(z_band - h) / z_band``.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

BORDER_TOP_MARGIN_PX = 2


@dataclass
class RobotDetectionParams:
    """Tunables; see the component JSON for operator-facing descriptions."""

    border_filter_mode: str = "none"      # "none" | "top"
    reference_point_mode: str = "centroid"  # "centroid" | "bottom_edge"
    min_contour_area: float = 500.0       # px
    morph_kernel_size: int = 15           # px, ellipse, morphology open
    depth_search_radius_px: int = 2       # search window for a valid belt depth
    depth_average_frames: int = 5         # moving average window (1 = off)


@dataclass
class RobotDetection:
    """Result for one frame. ``z_band_mm`` is the camera->belt distance, not the
    object distance. Pixel fields are full-frame; for the debug overlay."""

    x_mm: float
    y_mm: float
    z_band_mm: float
    ref_px: Tuple[int, int]
    belt_px: Tuple[int, int]
    contour: np.ndarray
    mask: np.ndarray


class BeltDistanceFilter:
    """Moving average of the belt distance to fight the noisy depth sensor."""

    def __init__(self, window: int = 5) -> None:
        self._window = max(1, int(window))
        self._values: deque = deque(maxlen=self._window)

    def reset(self) -> None:
        self._values.clear()

    def update(self, window: int, value: float) -> float:
        window = max(1, int(window))
        if window != self._window:
            self._window = window
            self._values = deque(self._values, maxlen=window)
        self._values.append(value)
        return float(sum(self._values) / len(self._values))


def _find_valid_depth_m(depth_mm: np.ndarray, px: int, py: int, radius: int) -> Optional[float]:
    """Search a (2r+1) window around (px, py) for the first non-zero depth (mm),
    returning meters. Mirrors the C++ try_get_depth_m clamp-and-scan."""
    h, w = depth_mm.shape[:2]
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            sx = min(max(px + dx, 0), w - 1)
            sy = min(max(py + dy, 0), h - 1)
            d = float(depth_mm[sy, sx])
            if d > 0.0:
                return d / 1000.0
    return None


def detect_object(depth_mm: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                  params: RobotDetectionParams,
                  belt_filter: Optional[BeltDistanceFilter] = None) -> Optional[RobotDetection]:
    """Locate the object hole and return its back-projected reference point.

    Returns ``None`` when no qualifying object is visible or no valid belt depth
    is found in the search window.
    """
    # Object = where depth is invalid (too close). Belt/background has valid depth.
    mask = (depth_mm == 0).astype(np.uint8) * 255

    ksize = max(1, int(params.morph_kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    if params.border_filter_mode == "top":
        contours = [c for c in contours if cv2.boundingRect(c)[1] <= BORDER_TOP_MARGIN_PX]
        if not contours:
            return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < params.min_contour_area:
        return None

    # Object pixels of the largest contour only (filled).
    obj_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(obj_mask, [largest], 0, 255, cv2.FILLED)
    ys, xs = np.nonzero(obj_mask)
    if xs.size == 0:
        return None

    max_y = int(ys.max())
    mean_x = int(round(float(xs.mean())))

    # Reference pixel: stable centroid (default) or the C++ bottom-edge point.
    if params.reference_point_mode == "bottom_edge":
        ref_px = (mean_x, max_y)
    else:  # centroid
        ref_px = (int(round(float(xs.mean()))), int(round(float(ys.mean()))))

    # Belt distance: sample just *below* the object's bottom edge (on the belt),
    # where depth is valid; optionally averaged over recent frames.
    h, w = depth_mm.shape[:2]
    belt_px = (min(max(mean_x, 0), w - 1), min(max(max_y + 1, 0), h - 1))
    depth_m = _find_valid_depth_m(depth_mm, belt_px[0], belt_px[1], params.depth_search_radius_px)
    if depth_m is None:
        return None
    z_band_mm = depth_m * 1000.0
    if belt_filter is not None:
        z_band_mm = belt_filter.update(params.depth_average_frames, z_band_mm)
        depth_m = z_band_mm / 1000.0

    # Back-project the reference pixel with the belt distance -> metric x/y (mm).
    x_mm = (ref_px[0] - cx) * depth_m / fx * 1000.0
    y_mm = (ref_px[1] - cy) * depth_m / fy * 1000.0

    return RobotDetection(
        x_mm=x_mm, y_mm=y_mm, z_band_mm=z_band_mm,
        ref_px=ref_px, belt_px=belt_px, contour=largest, mask=obj_mask,
    )
