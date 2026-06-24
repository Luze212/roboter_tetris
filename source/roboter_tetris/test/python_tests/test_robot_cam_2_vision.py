"""Tests for the edge-detection robot-camera module (approach 2, no ROS required).

A block on the belt forms a strong rectangular edge; the edges are closed into a
region and gated by the depth near-region — same selection/back-projection core as
the color approach, only the candidate mask differs.
"""

import math

import numpy as np

from roboter_tetris.vision.robot_detection_edge import (
    EdgeDetectionParams, detect_object_edges, edge_candidate_mask,
)
from roboter_tetris.vision.robot_detection import BeltDistanceFilter

BELT_GRAY = (90, 90, 90)
BLOCK_GRAY = (220, 220, 220)


def _intrinsics():
    return 300.0, 300.0, 160.0, 120.0  # fx, fy, cx, cy


def _scene(block=(130, 80, 60, 60), belt_mm=1000.0, block_near=True):
    """Color: gray belt with a bright block rectangle (strong edge at its border).
    Depth: belt everywhere; block region reads 0 (near) when ``block_near``."""
    h, w = 240, 320
    color = np.full((h, w, 3), BELT_GRAY, dtype=np.uint8)
    depth = np.full((h, w), belt_mm, dtype=np.float32)
    x, y, bw, bh = block
    color[y:y + bh, x:x + bw] = BLOCK_GRAY
    if block_near:
        depth[y:y + bh, x:x + bw] = 0.0
    return color, depth


def test_edge_mask_fills_block_region():
    color, depth = _scene(block=(130, 80, 60, 60))
    mask = edge_candidate_mask(color, depth, EdgeDetectionParams())
    # The closed/filled block region should cover a large connected area.
    assert int(np.count_nonzero(mask)) > 2000


def test_depth_edges_recover_low_contrast_block():
    # Block almost the same gray as the belt -> no usable color edge, but it reads
    # near in depth. Fusing the depth-step edge must recover it.
    color, depth = _scene(block=(130, 80, 60, 60), block_near=True)
    color[80:140, 130:190] = (95, 95, 95)  # ~ belt gray (90) -> Canny finds nothing
    fx, fy, cx, cy = _intrinsics()
    res_color = detect_object_edges(color, depth, fx, fy, cx, cy,
                                    EdgeDetectionParams(use_depth_edges=False,
                                                        depth_average_frames=1))
    res_fused = detect_object_edges(color, depth, fx, fy, cx, cy,
                                    EdgeDetectionParams(use_depth_edges=True,
                                                        depth_average_frames=1))
    assert res_color is None       # pure color edges cannot see the block
    assert res_fused is not None   # depth-step edge recovers the outline


def test_finds_block_and_backprojects():
    color, depth = _scene(block=(130, 80, 60, 60))  # centered in x (160)
    fx, fy, cx, cy = _intrinsics()
    res = detect_object_edges(color, depth, fx, fy, cx, cy,
                              EdgeDetectionParams(depth_average_frames=1))
    assert res is not None
    assert abs(res.x_mm) < 20.0
    assert abs(res.z_band_mm - 1000.0) < 1e-6


def test_returns_none_on_flat_belt():
    h, w = 240, 320
    color = np.full((h, w, 3), BELT_GRAY, dtype=np.uint8)  # no edges
    depth = np.full((h, w), 1000.0, dtype=np.float32)
    fx, fy, cx, cy = _intrinsics()
    assert detect_object_edges(color, depth, fx, fy, cx, cy, EdgeDetectionParams()) is None


def test_respects_min_area():
    color, depth = _scene(block=(150, 110, 6, 6))  # tiny block
    fx, fy, cx, cy = _intrinsics()
    params = EdgeDetectionParams(min_contour_area=5000.0)
    assert detect_object_edges(color, depth, fx, fy, cx, cy, params) is None


def test_depth_gate_rejects_far_edge():
    # A bright patch with valid (belt) depth is background clutter, not a block.
    color, depth = _scene(block=(130, 80, 60, 60), block_near=False)
    fx, fy, cx, cy = _intrinsics()
    assert detect_object_edges(color, depth, fx, fy, cx, cy,
                               EdgeDetectionParams(use_depth_gate=True,
                                                   depth_average_frames=1)) is None
    # Gate off: the edge region is accepted.
    assert detect_object_edges(color, depth, fx, fy, cx, cy,
                               EdgeDetectionParams(use_depth_gate=False,
                                                   depth_average_frames=1)) is not None


def test_orientation_in_range():
    color, depth = _scene(block=(120, 70, 80, 50))
    fx, fy, cx, cy = _intrinsics()
    res = detect_object_edges(color, depth, fx, fy, cx, cy,
                              EdgeDetectionParams(depth_average_frames=1))
    assert res is not None
    assert 0.0 <= res.orientation_rad < math.pi


def test_belt_filter_averages_in_detection():
    fx, fy, cx, cy = _intrinsics()
    params = EdgeDetectionParams(depth_average_frames=2)
    f = BeltDistanceFilter()
    c1, d1 = _scene(belt_mm=1000.0)
    c2, d2 = _scene(belt_mm=1200.0)
    r1 = detect_object_edges(c1, d1, fx, fy, cx, cy, params, f)
    r2 = detect_object_edges(c2, d2, fx, fy, cx, cy, params, f)
    assert abs(r1.z_band_mm - 1000.0) < 1e-6
    assert abs(r2.z_band_mm - 1100.0) < 1e-6
