"""Endeffector-camera fine-localization via edge detection (approach 2).

Same task and same output contract as :mod:`robot_detection` (color region), but
the block footprint is found from **image gradients** instead of color: Canny
edges on the grayscale image, optionally fused with the **depth discontinuity**
(block<->belt), closed into a solid region, then gated against the depth
near-region. This is the originally-shelved "RGB edges + depth confirmation" idea,
kept as a parallel component so both approaches can be A/B-tested on the robot with
identical inputs/outputs.

Depth fusion (variant B): we do NOT Canny the raw depth (it is far too noisy on
this sensor). The reliable "depth edge" is the boundary of the near-region
(:func:`robot_detection.near_mask`) — the clean 0/belt step at the block outline.
OR-ing it into the color edges recovers the outline where the brightness contrast
is weak (e.g. a white block on the green belt). The block selection, depth gate,
orientation and back-projection are reused from :mod:`robot_detection` so the two
detectors stay directly comparable.

Pure numpy/cv2, no ROS imports, unit-testable with synthetic images.
"""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .robot_detection import (
    BeltDistanceFilter, RobotDetection, localize_largest_blob, near_mask,
)


@dataclass
class EdgeDetectionParams:
    """Tunables; see the component JSON for operator-facing descriptions."""

    canny_low: int = 50                   # lower Canny hysteresis threshold
    canny_high: int = 150                 # upper Canny hysteresis threshold
    blur_ksize: int = 5                   # pre-blur kernel (odd; reduces edge noise)
    use_depth_edges: bool = True          # fuse the depth step as a 2nd edge source
    min_contour_area: float = 500.0       # px
    morph_kernel_size: int = 15           # px, ellipse, closes the edge outline
    use_depth_gate: bool = True           # reject background edges via depth
    depth_search_radius_px: int = 2       # search window for a valid belt depth
    depth_average_frames: int = 5         # moving average window (1 = off)

    # -- Blob selection (identical to the color variant, so the A/B test from
    # B6 compares detection cores and nothing else) ------------------------
    max_contour_area: float = 50000.0     # px, 0 = off; oversized blob -> reject frame
    roi_radius_px: float = 0.0            # px, 0 = off; cutoff around the expected point
    expect_offset_x_mm: float = 0.0       # mm at the belt distance, height-independent
    expect_offset_y_mm: float = 0.0
    select_nearest_to_expect: bool = False


def color_edge_map(color_bgr: np.ndarray, params: EdgeDetectionParams) -> np.ndarray:
    """Canny edges on the (blurred) grayscale image."""
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    k = max(1, int(params.blur_ksize))
    if k % 2 == 0:
        k += 1  # GaussianBlur needs an odd kernel
    blur = cv2.GaussianBlur(gray, (k, k), 0)
    return cv2.Canny(blur, int(params.canny_low), int(params.canny_high))


def depth_edge_map(depth_mm: np.ndarray) -> np.ndarray:
    """Depth-step edge: the boundary of the near-region (block<->belt). Robust
    against the noisy sensor because the near-region is already thresholded."""
    near = near_mask(depth_mm)
    near_u8 = near.astype(np.uint8) * 255
    sk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    near_u8 = cv2.morphologyEx(near_u8, cv2.MORPH_OPEN, sk)   # drop tiny specks
    return cv2.morphologyEx(near_u8, cv2.MORPH_GRADIENT, sk)  # thin closed boundary


def edge_candidate_mask(color_bgr: np.ndarray, depth_mm: np.ndarray,
                        params: EdgeDetectionParams) -> np.ndarray:
    """Block-candidate mask from gradients: (color edges [+ depth-step edges])
    closed into solid blobs."""
    edges = color_edge_map(color_bgr, params)
    if params.use_depth_edges:
        edges = cv2.bitwise_or(edges, depth_edge_map(depth_mm))

    # Close the (possibly broken) outline so the block boundary forms a loop, then
    # fill the enclosed area to turn the outline into a solid region.
    ksize = max(1, int(params.morph_kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    filled = np.zeros_like(closed)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
    return filled


def detect_object_edges(color_bgr: np.ndarray, depth_mm: np.ndarray,
                        fx: float, fy: float, cx: float, cy: float,
                        params: EdgeDetectionParams,
                        belt_filter: Optional[BeltDistanceFilter] = None
                        ) -> Optional[RobotDetection]:
    """Locate the block from (fused) image edges and back-project its center.

    Returns ``None`` when no qualifying block is visible or no valid belt depth is
    found. ``color_bgr`` and ``depth_mm`` must share the same HxW grid.
    """
    candidate = edge_candidate_mask(color_bgr, depth_mm, params)
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
