"""Tests for the pure robot-camera localization module (no ROS required).

Color-region approach: a green belt with a colored block; the block is localized
in the color image, the aligned depth gates reflections and gives the belt
distance.
"""

import math

import numpy as np

from roboter_tetris.vision.robot_detection import (
    BeltDistanceFilter, RobotDetectionParams, detect_object,
)

GREEN_BGR = (40, 160, 40)   # belt
RED_BGR = (40, 40, 200)     # a block
WHITE_BGR = (235, 235, 235)


def _intrinsics():
    return 300.0, 300.0, 160.0, 120.0  # fx, fy, cx, cy


def _scene(block=(130, 80, 60, 60), block_bgr=RED_BGR, belt_mm=1000.0,
           block_near=True):
    """Color: green belt with a block rectangle. Depth: belt everywhere; the block
    region reads 0 (too close) when ``block_near`` (so the gate accepts it)."""
    h, w = 240, 320
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:] = GREEN_BGR
    depth = np.full((h, w), belt_mm, dtype=np.float32)
    x, y, bw, bh = block
    color[y:y + bh, x:x + bw] = block_bgr
    if block_near:
        depth[y:y + bh, x:x + bw] = 0.0
    return color, depth


def test_finds_block_and_backprojects():
    color, depth = _scene(block=(130, 80, 60, 60))  # centered in x (160)
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=1)
    res = detect_object(color, depth, fx, fy, cx, cy, params)
    assert res is not None
    # Block center x ~ 160 (== cx) -> x_mm ~ 0; belt distance 1000 mm.
    assert abs(res.x_mm) < 15.0
    assert abs(res.z_band_mm - 1000.0) < 1e-6


def test_returns_none_on_empty_belt():
    h, w = 240, 320
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:] = GREEN_BGR
    depth = np.full((h, w), 1000.0, dtype=np.float32)
    fx, fy, cx, cy = _intrinsics()
    assert detect_object(color, depth, fx, fy, cx, cy, RobotDetectionParams()) is None


def test_respects_min_area():
    color, depth = _scene(block=(150, 110, 6, 6))  # tiny block
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(min_contour_area=5000.0)
    assert detect_object(color, depth, fx, fy, cx, cy, params) is None


def test_depth_gate_rejects_belt_reflection():
    # A white patch on the belt that does NOT read near (valid belt depth) is a
    # reflection; the gate must reject it.
    color, depth = _scene(block=(130, 80, 60, 60), block_bgr=WHITE_BGR,
                          block_near=False)
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(use_depth_gate=True, depth_average_frames=1)
    assert detect_object(color, depth, fx, fy, cx, cy, params) is None
    # With the gate off, the white patch is accepted (color-only test mode).
    params_off = RobotDetectionParams(use_depth_gate=False, depth_average_frames=1)
    assert detect_object(color, depth, fx, fy, cx, cy, params_off) is not None


def test_depth_gate_keeps_near_white_block():
    # A white block that DOES read near must pass the gate (white vs reflection).
    color, depth = _scene(block=(130, 80, 60, 60), block_bgr=WHITE_BGR,
                          block_near=True)
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(use_depth_gate=True, depth_average_frames=1)
    assert detect_object(color, depth, fx, fy, cx, cy, params) is not None


def test_orientation_in_range():
    color, depth = _scene(block=(120, 70, 80, 50))
    fx, fy, cx, cy = _intrinsics()
    res = detect_object(color, depth, fx, fy, cx, cy,
                        RobotDetectionParams(depth_average_frames=1))
    assert res is not None
    assert 0.0 <= res.orientation_rad < math.pi


def test_belt_depth_search_skips_invalid():
    # Invalid stripe right under the block; the search radius must still find a
    # valid belt pixel nearby. Disable the gate so the elevated stripe doesn't
    # interfere with this isolated belt-sampling check.
    color, depth = _scene(block=(130, 80, 60, 60), belt_mm=1000.0)
    depth[140:142, :] = 0.0  # invalid stripe just below the block bottom (y=140)
    fx, fy, cx, cy = _intrinsics()
    res = detect_object(color, depth, fx, fy, cx, cy,
                        RobotDetectionParams(depth_search_radius_px=3,
                                             use_depth_gate=False,
                                             depth_average_frames=1))
    assert res is not None
    assert abs(res.z_band_mm - 1000.0) < 1e-6


def test_belt_filter_moving_average_converges():
    f = BeltDistanceFilter(window=3)
    assert f.update(3, 900.0) == 900.0
    assert f.update(3, 1100.0) == 1000.0          # mean(900, 1100)
    assert abs(f.update(3, 1000.0) - 1000.0) < 1e-9  # mean(900,1100,1000)
    # Window slides: oldest (900) drops out.
    assert abs(f.update(3, 1000.0) - (1100 + 1000 + 1000) / 3) < 1e-9


def test_belt_filter_averages_in_detection():
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=2)
    f = BeltDistanceFilter()
    c1, d1 = _scene(belt_mm=1000.0)
    c2, d2 = _scene(belt_mm=1200.0)
    r1 = detect_object(c1, d1, fx, fy, cx, cy, params, f)
    r2 = detect_object(c2, d2, fx, fy, cx, cy, params, f)
    assert abs(r1.z_band_mm - 1000.0) < 1e-6
    assert abs(r2.z_band_mm - 1100.0) < 1e-6  # mean(1000, 1200)
