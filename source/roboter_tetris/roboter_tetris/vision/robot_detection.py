"""Endeffector-camera fine-localization of the object under the gripper.

Color-region approach (replaces the previous depth-hole port, which was not
reproducible on our setup). The conveyor belt is green; the 3D-printed blocks are
red / blue / white with a **matte top face** (only the sides are glossy). From a
near-overhead view the camera sees essentially the matte top, which gives a clean
color signal — unaffected by the specular LiDAR failure on the glossy sides.

Pipeline (pure numpy/cv2, no ROS imports, unit-testable with synthetic images):

1. Mask out the green belt in HSV -> invert -> block candidates.
2. Morphological open to remove speckle.
3. Optional depth near-gate: keep only blobs that overlap a "near" region of the
   aligned depth (depth == 0 from the too-close / glossy object, or depth clearly
   above the belt). A specular reflection on the belt reads valid belt depth and
   is therefore rejected — this is what disambiguates a white block from a belt
   glare spot.
4. Largest qualifying blob -> ``minAreaRect`` -> center + orientation.
5. Sample the belt distance just below the blob (valid depth) -> ``z``.
6. Back-project the blob center with ``z`` -> metric x/y in the camera frame.

The color image and the depth image must share the same pixel grid (the aligned
depth lives in the color frame); the component resizes the depth to the color
resolution before calling in.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

# A qualifying blob must overlap the depth near-region by at least this fraction
# of its own area (guards against a single stray zero pixel passing the gate).
GATE_MIN_OVERLAP = 0.10
# How far above the belt (mm) a pixel must sit to count as "near" (object), in
# addition to the depth == 0 case. Loose, because the sensor is noisy.
GATE_HEIGHT_MARGIN_MM = 20.0


@dataclass
class RobotDetectionParams:
    """Tunables; see the component JSON for operator-facing descriptions."""

    # Green belt HSV bounds (OpenCV ranges: H 0-179, S/V 0-255). Everything that
    # is NOT belt-green (and bright enough) is a block candidate.
    belt_h_min: int = 35
    belt_h_max: int = 85
    belt_s_min: int = 60
    belt_v_min: int = 40

    min_contour_area: float = 500.0       # px
    morph_kernel_size: int = 15           # px, ellipse, morphology open
    use_depth_gate: bool = True           # reject belt reflections via depth
    depth_search_radius_px: int = 2       # search window for a valid belt depth
    depth_average_frames: int = 5         # moving average window (1 = off)

    # -- Blob selection (see localize_largest_blob) -------------------------
    # Upper area bound: a blob far larger than any block means the candidate
    # mask failed and the whole frame is rejected. 0 = off.
    max_contour_area: float = 50000.0     # px
    # Hard cutoff around the expected image position. 0 = off (whole image).
    roi_radius_px: float = 0.0            # px
    # Expected lateral offset of the block from the optical axis, in mm at the
    # belt distance. Kept in mm so it survives a change of observation height.
    expect_offset_x_mm: float = 0.0
    expect_offset_y_mm: float = 0.0
    # Rank candidates by distance to the expected point instead of by area.
    select_nearest_to_expect: bool = False


@dataclass
class RobotDetection:
    """Result for one frame. ``z_band_mm`` is the camera->belt distance, not the
    object distance. Pixel fields are full-frame; for the debug overlay."""

    x_mm: float
    y_mm: float
    z_band_mm: float
    orientation_rad: float                # [0, pi), from the top-face minAreaRect
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
    returning meters."""
    h, w = depth_mm.shape[:2]
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            sx = min(max(px + dx, 0), w - 1)
            sy = min(max(py + dy, 0), h - 1)
            d = float(depth_mm[sy, sx])
            if d > 0.0:
                return d / 1000.0
    return None


def belt_candidate_mask(color_bgr: np.ndarray, params: RobotDetectionParams) -> np.ndarray:
    """Block-candidate mask: everything that is NOT belt-green."""
    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    belt = cv2.inRange(
        hsv,
        np.array([params.belt_h_min, params.belt_s_min, params.belt_v_min], dtype=np.uint8),
        np.array([params.belt_h_max, 255, 255], dtype=np.uint8),
    )
    candidate = cv2.bitwise_not(belt)
    ksize = max(1, int(params.morph_kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    return cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel)


def belt_reference_mm(depth_mm: np.ndarray) -> Optional[float]:
    """Median of the valid depth, i.e. a robust pre-estimate of the camera->belt
    distance (the belt dominates the frame).

    Available *before* any blob has been chosen, which is what the expected-point
    calculation in :func:`localize_largest_blob` needs -- the precise belt
    distance is only sampled next to the selected blob, later in the pipeline.
    """
    valid = depth_mm[depth_mm > 0]
    if not valid.size:
        return None
    return float(np.median(valid))


def near_mask(depth_mm: np.ndarray) -> np.ndarray:
    """Pixels that read as an object: invalid depth (too close / glossy) or
    clearly elevated above the belt. Belt and belt reflections read valid belt
    depth and are excluded."""
    near = depth_mm == 0
    belt_ref = belt_reference_mm(depth_mm)
    if belt_ref is not None:
        near = near | (depth_mm < belt_ref - GATE_HEIGHT_MARGIN_MM)
    return near


def _normalize_orientation(angle_deg: float) -> float:
    """minAreaRect angle (deg) -> radians wrapped into [0, pi)."""
    return math.radians(angle_deg) % math.pi


def localize_largest_blob(candidate: np.ndarray, depth_mm: np.ndarray,
                          fx: float, fy: float, cx: float, cy: float, *,
                          min_contour_area: float, use_depth_gate: bool,
                          depth_search_radius_px: int, depth_average_frames: int,
                          belt_filter: Optional[BeltDistanceFilter] = None,
                          max_contour_area: float = 0.0,
                          roi_radius_px: float = 0.0,
                          expect_offset_x_mm: float = 0.0,
                          expect_offset_y_mm: float = 0.0,
                          select_nearest_to_expect: bool = False,
                          ) -> Optional[RobotDetection]:
    """Shared core for every endeffector detector: pick the largest qualifying
    blob of a binary ``candidate`` mask, gate it against the depth near-region,
    then measure the belt distance and back-project the center to metric x/y.

    Both detection approaches (color region, edge detection) differ ONLY in how
    they build ``candidate``; the selection, gate, orientation and back-projection
    are identical here so the two components stay directly comparable.

    The four selection guards default to "off" (except ``max_contour_area``,
    which the callers set from their parameters), so an unconfigured call behaves
    exactly as before:

    * ``max_contour_area`` -- an oversized blob rejects the **whole frame**. It
      means the candidate mask failed (belt, glare and block merged), and then no
      contour is trustworthy; taking the next largest would just swap one wrong
      answer for another.
    * ``roi_radius_px`` and ``expect_offset_*`` -- hard cutoff around the
      expected image position of the block, which excludes machine structure at
      the frame border by construction.
    * ``select_nearest_to_expect`` -- rank by distance to that point instead of
      by area. Both guards share the same distance, so they cost one hypot per
      contour.

    Filtering happens on the ``minAreaRect`` centre, never by cropping the
    candidate mask: a blob clipped at the ROI edge would have its rectangle --
    and therefore the reported position and orientation -- shifted.
    """
    contours, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Areas are needed twice (ceiling check and ranking); compute them once --
    # the image processing already runs close to the CPU budget.
    areas = [cv2.contourArea(c) for c in contours]

    # An oversized blob means the candidate mask failed -- reject the frame.
    if max_contour_area > 0.0 and max(areas) > max_contour_area:
        return None

    near = near_mask(depth_mm) if use_depth_gate else None

    # Expected image position of the block. The mm offset is converted with the
    # current belt distance, so the same parameter holds at any camera height.
    belt_ref_mm = belt_reference_mm(depth_mm)
    if belt_ref_mm and (expect_offset_x_mm or expect_offset_y_mm):
        u_expect = cx + expect_offset_x_mm * fx / belt_ref_mm
        v_expect = cy + expect_offset_y_mm * fy / belt_ref_mm
    else:
        u_expect, v_expect = cx, cy

    # Best qualifying contour: above min area, inside the ROI and (optionally)
    # overlapping the depth near-region. Ranked by distance to the expected
    # point, or by area when no expectation is configured.
    best = None
    best_rect = None
    best_area = 0.0
    best_dist = float("inf")
    for c, area in zip(contours, areas):
        if area < min_contour_area:
            continue
        rect = cv2.minAreaRect(c)
        (rect_cx, rect_cy), _, _ = rect
        dist = math.hypot(rect_cx - u_expect, rect_cy - v_expect)
        if roi_radius_px > 0.0 and dist > roi_radius_px:
            continue
        if select_nearest_to_expect:
            if dist >= best_dist:
                continue
        elif area <= best_area:
            continue
        if near is not None:
            blob = np.zeros(candidate.shape, dtype=np.uint8)
            cv2.drawContours(blob, [c], 0, 255, cv2.FILLED)
            blob_px = int(np.count_nonzero(blob))
            overlap = int(np.count_nonzero(near & (blob > 0)))
            if blob_px == 0 or overlap / blob_px < GATE_MIN_OVERLAP:
                continue
        best = c
        best_rect = rect
        best_area = area
        best_dist = dist
    if best is None:
        return None

    # Filled blob pixels for centroid / bottom-edge belt sampling.
    obj_mask = np.zeros(candidate.shape, dtype=np.uint8)
    cv2.drawContours(obj_mask, [best], 0, 255, cv2.FILLED)
    ys, xs = np.nonzero(obj_mask)
    if xs.size == 0:
        return None
    max_y = int(ys.max())
    mean_x = int(round(float(xs.mean())))

    # Center + orientation from the oriented bounding box of the top face
    # (captured during selection, so minAreaRect runs once per contour).
    (rect_cx, rect_cy), _, angle = best_rect
    ref_px = (int(round(rect_cx)), int(round(rect_cy)))
    orientation_rad = _normalize_orientation(angle)

    # Belt distance: sample just below the blob (on the belt), where depth is valid.
    h, w = depth_mm.shape[:2]
    belt_px = (min(max(mean_x, 0), w - 1), min(max(max_y + 1, 0), h - 1))
    depth_m = _find_valid_depth_m(depth_mm, belt_px[0], belt_px[1], depth_search_radius_px)
    if depth_m is None:
        return None
    z_band_mm = depth_m * 1000.0
    if belt_filter is not None:
        z_band_mm = belt_filter.update(depth_average_frames, z_band_mm)
        depth_m = z_band_mm / 1000.0

    # Back-project the blob center with the belt distance -> metric x/y (mm).
    x_mm = (ref_px[0] - cx) * depth_m / fx * 1000.0
    y_mm = (ref_px[1] - cy) * depth_m / fy * 1000.0

    return RobotDetection(
        x_mm=x_mm, y_mm=y_mm, z_band_mm=z_band_mm, orientation_rad=orientation_rad,
        ref_px=ref_px, belt_px=belt_px, contour=best, mask=obj_mask,
    )


def detect_object(color_bgr: np.ndarray, depth_mm: np.ndarray,
                  fx: float, fy: float, cx: float, cy: float,
                  params: RobotDetectionParams,
                  belt_filter: Optional[BeltDistanceFilter] = None) -> Optional[RobotDetection]:
    """Locate the block by color region and back-project its center.

    Returns ``None`` when no qualifying block is visible or no valid belt depth is
    found in the search window. ``color_bgr`` and ``depth_mm`` must share the same
    HxW grid.
    """
    candidate = belt_candidate_mask(color_bgr, params)
    return localize_largest_blob(
        candidate, depth_mm, fx, fy, cx, cy,
        min_contour_area=params.min_contour_area,
        use_depth_gate=params.use_depth_gate,
        depth_search_radius_px=params.depth_search_radius_px,
        depth_average_frames=params.depth_average_frames,
        belt_filter=belt_filter,
        max_contour_area=params.max_contour_area,
        roi_radius_px=params.roi_radius_px,
        expect_offset_x_mm=params.expect_offset_x_mm,
        expect_offset_y_mm=params.expect_offset_y_mm,
        select_nearest_to_expect=params.select_nearest_to_expect,
    )
