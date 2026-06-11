"""Tests for the pure robot-camera localization module (no ROS required)."""

import numpy as np

from roboter_tetris.vision.robot_detection import (
    BeltDistanceFilter, RobotDetectionParams, detect_object,
)


def _intrinsics():
    return 300.0, 300.0, 160.0, 120.0  # fx, fy, cx, cy


def _scene(hole=(90, 80, 60, 60), belt_mm=1000.0, top_touch=False):
    """Depth frame: belt at ``belt_mm`` everywhere, a depth==0 hole = object."""
    h, w = 240, 320
    depth = np.full((h, w), belt_mm, dtype=np.float32)
    x, y, bw, bh = hole
    if top_touch:
        y = 0
    depth[y:y + bh, x:x + bw] = 0.0  # object reads as invalid depth
    return depth


def test_finds_hole_and_backprojects():
    depth = _scene(hole=(130, 80, 60, 60), belt_mm=1000.0)  # centered in x
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=1)
    res = detect_object(depth, fx, fy, cx, cy, params)
    assert res is not None
    # Hole centroid x ~ 160 (== cx) -> x_mm ~ 0; belt distance 1000 mm.
    assert abs(res.x_mm) < 15.0
    assert abs(res.z_band_mm - 1000.0) < 1e-6


def test_returns_none_when_no_hole():
    depth = np.full((240, 320), 1000.0, dtype=np.float32)
    fx, fy, cx, cy = _intrinsics()
    assert detect_object(depth, fx, fy, cx, cy, RobotDetectionParams()) is None


def test_respects_min_area():
    depth = _scene(hole=(150, 110, 6, 6))  # tiny hole
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(min_contour_area=5000.0)
    assert detect_object(depth, fx, fy, cx, cy, params) is None


def test_border_filter_top():
    fx, fy, cx, cy = _intrinsics()
    # Hole in the middle (does not touch top) -> rejected in 'top' mode.
    depth_mid = _scene(hole=(130, 90, 60, 60))
    params_top = RobotDetectionParams(border_filter_mode="top", depth_average_frames=1)
    assert detect_object(depth_mid, fx, fy, cx, cy, params_top) is None
    # Hole touching the top border -> accepted in 'top' mode.
    depth_top = _scene(hole=(130, 0, 60, 60), top_touch=True)
    assert detect_object(depth_top, fx, fy, cx, cy, params_top) is not None
    # 'none' mode accepts the middle hole.
    params_none = RobotDetectionParams(border_filter_mode="none", depth_average_frames=1)
    assert detect_object(depth_mid, fx, fy, cx, cy, params_none) is not None


def test_reference_point_modes_differ():
    depth = _scene(hole=(130, 60, 60, 80))  # taller than wide
    fx, fy, cx, cy = _intrinsics()
    centroid = detect_object(depth, fx, fy, cx, cy,
                             RobotDetectionParams(reference_point_mode="centroid",
                                                  depth_average_frames=1))
    bottom = detect_object(depth, fx, fy, cx, cy,
                           RobotDetectionParams(reference_point_mode="bottom_edge",
                                                depth_average_frames=1))
    # Bottom-edge reference sits lower in the image than the centroid.
    assert bottom.ref_px[1] > centroid.ref_px[1]


def test_belt_depth_search_skips_invalid():
    # Put a stripe of invalid depth right under the object; the search radius must
    # still find a valid belt pixel nearby.
    depth = _scene(hole=(130, 80, 60, 60), belt_mm=1000.0)
    depth[140:142, :] = 0.0  # invalid stripe just below the hole bottom (y=140)
    fx, fy, cx, cy = _intrinsics()
    res = detect_object(depth, fx, fy, cx, cy,
                        RobotDetectionParams(depth_search_radius_px=3, depth_average_frames=1))
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
    r1 = detect_object(_scene(belt_mm=1000.0), fx, fy, cx, cy, params, f)
    r2 = detect_object(_scene(belt_mm=1200.0), fx, fy, cx, cy, params, f)
    assert abs(r1.z_band_mm - 1000.0) < 1e-6
    assert abs(r2.z_band_mm - 1100.0) < 1e-6  # mean(1000, 1200)
