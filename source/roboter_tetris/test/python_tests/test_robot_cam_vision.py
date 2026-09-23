"""Tests for the pure robot-camera localization module (no ROS required).

Color-region approach: a green belt with a colored block; the block is localized
in the color image, the aligned depth gates reflections and gives the belt
distance.
"""

import math

import numpy as np

from roboter_tetris.vision.robot_detection import (
    BeltDistanceFilter, RobotDetectionParams, belt_reference_mm, detect_object,
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


def test_belt_distance_is_the_frame_median_not_the_spot_below_the_block():
    """22.09.2026 (Nachtrag 12 / K3): from close up the spot just below the blob
    is the block's shadow down to the image border, or the block itself. The
    top face reads valid depth (650 of 1000 mm here, like 184 of 284 at the
    setup) and the shadow below it reads 0. Sampling there returned the top
    face or nothing; the frame median returns the belt."""
    color, depth = _scene(block=(130, 80, 60, 60), belt_mm=1000.0, block_near=False)
    depth[80:140, 130:190] = 650.0   # top face, valid depth, clearly near
    depth[140:, 120:200] = 0.0       # shadow from the blob to the bottom edge
    fx, fy, cx, cy = _intrinsics()
    res = detect_object(color, depth, fx, fy, cx, cy,
                        RobotDetectionParams(depth_average_frames=1))
    assert res is not None
    assert abs(res.z_band_mm - 1000.0) < 1e-6
    # Block centre at pixel x 160 = cx: back-projected with the belt distance.
    assert abs(res.x_mm) < 5.0


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


# -- Blob selection guards (shared core, so robot_cam_2 inherits them) ---------


def _two_blob_scene(belt_mm=1000.0, near=(140, 90, 40, 40), far=(250, 40, 60, 160)):
    """A small blob near the optical axis and a much larger one at the frame
    border -- the situation both variants got wrong at the setup on 15.09."""
    h, w = 240, 320
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:] = GREEN_BGR
    depth = np.full((h, w), belt_mm, dtype=np.float32)
    for (x, y, bw, bh) in (near, far):
        color[y:y + bh, x:x + bw] = RED_BGR
        depth[y:y + bh, x:x + bw] = 0.0
    return color, depth


def test_belt_reference_is_the_median_of_valid_depth():
    depth = np.full((10, 10), 800.0, dtype=np.float32)
    depth[0:3, 0:3] = 0.0  # invalid readings must not drag the median
    assert belt_reference_mm(depth) == 800.0
    assert belt_reference_mm(np.zeros((4, 4), dtype=np.float32)) is None


def test_oversized_blob_rejects_the_whole_frame():
    """Reproduces the false alarm of 15.09.: when the belt mask fails, belt,
    glare and block merge into one image-filling blob whose minAreaRect centre
    lands on the principal point -- a plausible-looking few millimetres that
    max_korrektur_m in the follower does NOT catch."""
    h, w = 240, 320
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:] = GREEN_BGR
    depth = np.full((h, w), 1000.0, dtype=np.float32)
    # Centred on the principal point (160, 120) so the reported position really
    # is the misleading "few millimetres off centre" seen at the setup.
    color[20:220, 10:310] = WHITE_BGR      # 200 x 300 = 60 000 px
    depth[20:220, 10:310] = 0.0
    fx, fy, cx, cy = _intrinsics()

    # Without the upper bound the blob is accepted and reports the image centre.
    loose = RobotDetectionParams(depth_average_frames=1, max_contour_area=0.0)
    bogus = detect_object(color, depth, fx, fy, cx, cy, loose)
    assert bogus is not None
    assert abs(bogus.x_mm) < 30.0 and abs(bogus.y_mm) < 30.0   # the signature

    # With it, the frame is rejected outright.
    guarded = RobotDetectionParams(depth_average_frames=1, max_contour_area=50000.0)
    assert detect_object(color, depth, fx, fy, cx, cy, guarded) is None


def test_largest_blob_wins_without_guards():
    """Negative control: this is today's behaviour and the reason the edge
    variant reported the machine structure at the frame border."""
    color, depth = _two_blob_scene()
    fx, fy, cx, cy = _intrinsics()
    res = detect_object(color, depth, fx, fy, cx, cy,
                        RobotDetectionParams(depth_average_frames=1))
    assert res is not None
    assert res.ref_px[0] > 250      # the big blob at the border, not the block


def test_roi_excludes_the_blob_at_the_border():
    color, depth = _two_blob_scene()
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=1, roi_radius_px=60.0)
    res = detect_object(color, depth, fx, fy, cx, cy, params)
    assert res is not None
    assert abs(res.ref_px[0] - 160) < 10    # the small blob on the axis
    assert abs(res.x_mm) < 40.0


def test_selection_by_expectation_beats_area():
    """Without any ROI: ranking by distance alone must already pick the block."""
    color, depth = _two_blob_scene()
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=1,
                                  select_nearest_to_expect=True)
    res = detect_object(color, depth, fx, fy, cx, cy, params)
    assert res is not None
    assert abs(res.ref_px[0] - 160) < 10


def test_expectation_offset_is_height_independent():
    """The offset is carried in mm, so one parameter value holds at any camera
    height -- the reason it is not expressed in pixels (B8 is still open).

    u = cx + offset_mm * fx / z, so 200 mm sits at u = 220 for a 1000 mm belt
    distance and at u = 190 for 2000 mm. A tight ROI around the expected point
    must find the block in both scenes with the SAME parameter value.
    """
    fx, fy, cx, cy = _intrinsics()
    params = RobotDetectionParams(depth_average_frames=1,
                                  expect_offset_x_mm=200.0,
                                  roi_radius_px=25.0)
    for belt_mm, blob_corner_x in ((1000.0, 200), (2000.0, 170)):
        # corner + half the 40 px width -> centre at u = 220 resp. 190
        color, depth = _two_blob_scene(belt_mm=belt_mm,
                                       near=(blob_corner_x, 90, 40, 40))
        res = detect_object(color, depth, fx, fy, cx, cy, params)
        expected_u = cx + 200.0 * fx / belt_mm
        assert res is not None, f"kein Treffer bei Banddistanz {belt_mm} mm"
        assert abs(res.ref_px[0] - expected_u) < 10, (
            f"Banddistanz {belt_mm}: {res.ref_px[0]} statt ~{expected_u}")
