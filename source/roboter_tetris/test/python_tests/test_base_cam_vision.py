"""Tests for the pure vision modules (no ROS/modulo required)."""

import math

import numpy as np
import pytest

from roboter_tetris.vision.color_estimation import (
    COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_RED, COLOR_UNKNOWN, COLOR_WHITE,
    COLOR_YELLOW, classify_hsv, estimate_object_color_id,
)
from roboter_tetris.vision.detection import (
    DetectionParams, build_cam_to_robot, compute_robust_orientation_2d,
    detect_objects, normalize_orientation_half_turn,
)
from roboter_tetris.vision.tracker import TrackedObject, VisionTracker


# -- classify_hsv ----------------------------------------------------------------

@pytest.mark.parametrize("hsv,expected", [
    ((0, 200, 30), COLOR_BLACK),       # v < 45
    ((0, 20, 200), COLOR_WHITE),       # low saturation, bright
    ((5, 200, 200), COLOR_RED),
    ((175, 200, 200), COLOR_RED),      # hue wrap-around
    ((25, 200, 200), COLOR_YELLOW),
    ((60, 200, 200), COLOR_GREEN),
    ((110, 200, 200), COLOR_BLUE),
    ((150, 200, 200), COLOR_UNKNOWN),
])
def test_classify_hsv(hsv, expected):
    assert classify_hsv(*hsv) == expected


def test_color_voting_is_robust_against_noise_pixels():
    import cv2
    hsv = np.zeros((40, 40, 3), dtype=np.uint8)
    hsv[:, :] = (60, 200, 200)  # green
    hsv[20, 20] = (110, 200, 200)  # single blue noise pixel at the center
    contour = np.array([[[5, 5]], [[35, 5]], [[35, 35]], [[5, 35]]], dtype=np.int32)
    # Majority vote over the patch must out-vote the single noise pixel.
    assert estimate_object_color_id(hsv, contour, (20, 20)) == COLOR_GREEN


# -- transform -------------------------------------------------------------------

def test_identity_transform():
    t = build_cam_to_robot(0, 0, 0, 0, 0, 0)
    np.testing.assert_allclose(t, np.eye(4), atol=1e-12)


def test_translation_only():
    t = build_cam_to_robot(1.0, 2.0, 3.0, 0, 0, 0)
    p = t @ np.array([0.5, 0.5, 0.5, 1.0])
    np.testing.assert_allclose(p[:3], [1.5, 2.5, 3.5], atol=1e-12)


def test_rotation_order_is_zyx():
    # 90° yaw maps x->y; with 90° roll applied FIRST (ZYX), z maps to... verify
    # against the explicit matrix product for a hand-checked point.
    t = build_cam_to_robot(0, 0, 0, 90.0, 0.0, 90.0)
    p = (t @ np.array([0.0, 0.0, 1.0, 1.0]))[:3]
    # Rx(90°): z->y; then Rz(90°): y->-x  => (0,0,1) -> (-1, 0, 0)... wait:
    # Rx(90): (0,0,1)->(0,-1,0)? Rx: y' = cos*y - sin*z = -1, z' = sin*y + cos*z = 0.
    # So (0,-1,0); Rz(90): x' = cos*x - sin*y = 1, y' = sin*x + cos*y = 0 -> (1,0,0).
    np.testing.assert_allclose(p, [1.0, 0.0, 0.0], atol=1e-9)


# -- orientation -----------------------------------------------------------------

def test_normalize_half_turn():
    assert abs(normalize_orientation_half_turn(math.pi + 0.3) - 0.3) < 1e-9
    assert abs(normalize_orientation_half_turn(-0.3) - (math.pi - 0.3)) < 1e-9
    assert math.isnan(normalize_orientation_half_turn(float("nan")))


def test_robust_orientation_long_axis():
    # Rectangle longer along +x: corners in order 0..3, edge 0-1 is the short one.
    box_px = np.array([[0, 0], [0, 10], [40, 10], [40, 0]], dtype=np.float32)
    corners_xy = np.array([[0.0, 0.0], [0.0, 0.01], [0.04, 0.01], [0.04, 0.0]])
    angle = compute_robust_orientation_2d(box_px, corners_xy)
    assert abs(angle - 0.0) < 1e-6  # long axis along x


# -- detection (synthetic scene) ---------------------------------------------------

def _synthetic_scene(obj_height_mm=50.0, conveyor_z=865.0, size_px=60,
                     center=(120, 100), color_hsv=(60, 200, 200)):
    import cv2
    h, w = 240, 320
    depth = np.full((h, w), conveyor_z, dtype=np.float32)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:, :] = (180, 180, 180)
    x0 = center[0] - size_px // 2
    y0 = center[1] - size_px // 2
    depth[y0:y0 + size_px, x0:x0 + size_px] = conveyor_z - obj_height_mm
    hsv_pixel = np.uint8([[list(color_hsv)]])
    bgr = cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0][0]
    color[y0:y0 + size_px, x0:x0 + size_px] = bgr
    return color, depth


def _intrinsics():
    return 300.0, 300.0, 160.0, 120.0  # fx, fy, cx, cy


def _looking_down(height_m=0.865):
    """Camera straight above the belt, belt surface at world z = 0: Rx(180°)
    turns camera z (depth, down) into world -z."""
    return build_cam_to_robot(0.0, 0.0, height_m, 180.0, 0.0, 0.0)


def test_detection_finds_object_with_expected_height_and_color():
    color, depth = _synthetic_scene(obj_height_mm=50.0)
    fx, fy, cx, cy = _intrinsics()
    params = DetectionParams(cam_to_robot=_looking_down(), belt_surface_z_mm=0.0,
                             erosion_px=0)
    detections, infos = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert len(detections) == 1
    det = detections[0]
    # Height: world z of the top face above the belt surface -- no bias.
    assert abs(det.height - 50.0) < 1.0
    # S1 field 4 is mid-height.
    assert abs(det.z - 25.0) < 1.0
    assert det.color == COLOR_GREEN
    # Square footprint must be flagged (orientation freezing in the tracker).
    assert det.square
    # 60 px at z=815 mm with f=300 -> 60*815/300 = 163 mm edge length.
    assert abs(det.length - 163.0) < 8.0
    assert abs(det.width - 163.0) < 8.0
    assert len(infos) == 1


def test_detection_respects_min_area_and_border_rejection():
    color, depth = _synthetic_scene()
    fx, fy, cx, cy = _intrinsics()
    # Min area larger than the object's pixel area -> nothing detected.
    params = DetectionParams(cam_to_robot=np.eye(4), min_contour_area=10000.0)
    detections, _ = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert detections == []
    # Object touching the border (via ROI ending inside the object) -> rejected.
    params = DetectionParams(cam_to_robot=np.eye(4), roi=(0, 0, 130, 240))
    detections, _ = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert detections == []


def test_tall_block_off_centre_is_placed_at_its_top_face_not_on_the_belt():
    """Nachtrag 13 (23.09.2026): the corner pixels of the rectangle sit on the
    block edge and read the belt. Deprojected with their own depth, the top
    outline landed on the belt along the viewing ray -- 16 mm off for a 100 mm
    block at the setup. Here the ring around the block reads belt depth, as
    the edge pixels of the real camera do."""
    conveyor = 865.0
    color, depth = _synthetic_scene(obj_height_mm=100.0, conveyor_z=conveyor,
                                    size_px=40, center=(240, 180))
    x0, y0 = 240 - 20, 180 - 20
    depth[y0:y0 + 40, x0] = conveyor            # edge columns/rows read the belt
    depth[y0:y0 + 40, x0 + 39] = conveyor
    depth[y0, x0:x0 + 40] = conveyor
    depth[y0 + 39, x0:x0 + 40] = conveyor
    fx, fy, cx, cy = _intrinsics()
    params = DetectionParams(cam_to_robot=_looking_down(conveyor / 1000.0),
                             belt_surface_z_mm=0.0, erosion_px=0)
    detections, _ = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert len(detections) == 1
    det = detections[0]
    top_depth = conveyor - 100.0
    u_c, v_c = x0 + 19.5, y0 + 19.5              # centre of the 40 px block
    expect_x = (u_c - cx) * top_depth / fx       # camera x = world x
    expect_y = -(v_c - cy) * top_depth / fy      # Rx(180°) flips y
    assert abs(det.x - expect_x) < 3.0, (det.x, expect_x)
    assert abs(det.y - expect_y) < 3.0, (det.y, expect_y)
    # On the belt the same pixel would be 13 % further out: 27 mm here.
    assert abs(det.x - (u_c - cx) * conveyor / fx) > 20.0
    assert abs(det.height - 100.0) < 1.0


def test_top_depth_bias_raises_the_top_face_and_the_height():
    """Nachtrag 13 / L6: the L515 reads block tops too deep; the bias takes it
    off the top-face depth -- the height grows by exactly that much."""
    color, depth = _synthetic_scene(obj_height_mm=50.0)
    fx, fy, cx, cy = _intrinsics()
    base = dict(cam_to_robot=_looking_down(), belt_surface_z_mm=0.0, erosion_px=0)
    d0, _ = detect_objects(color, depth, fx, fy, cx, cy, DetectionParams(**base))
    d1, _ = detect_objects(color, depth, fx, fy, cx, cy,
                           DetectionParams(top_depth_bias_mm=11.5, **base))
    assert abs((d1[0].height - d0[0].height) - 11.5) < 1e-6


def test_detection_applies_transform_and_offsets():
    color, depth = _synthetic_scene(center=(160, 120))  # on the optical axis
    fx, fy, cx, cy = _intrinsics()
    t = build_cam_to_robot(1.0, 0.0, 0.0, 0, 0, 0)  # shift x by 1 m
    params = DetectionParams(cam_to_robot=t, x_offset_mm=-35.0)
    detections, _ = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert len(detections) == 1
    # Centered object: camera x ~ 0 -> robot x ~ 1000 mm, plus offset -35.
    assert abs(detections[0].x - 965.0) < 10.0


def test_erosion_shrinks_footprint_not_height():
    color, depth = _synthetic_scene(obj_height_mm=50.0, size_px=60)
    fx, fy, cx, cy = _intrinsics()
    d0, _ = detect_objects(color, depth, fx, fy, cx, cy,
                           DetectionParams(cam_to_robot=np.eye(4), erosion_px=0))
    d3, _ = detect_objects(color, depth, fx, fy, cx, cy,
                           DetectionParams(cam_to_robot=np.eye(4), erosion_px=3))
    assert len(d0) == 1 and len(d3) == 1
    # Erosion shrinks the measured footprint (sheds the outer ring)...
    assert d3[0].length < d0[0].length
    assert d3[0].width < d0[0].width
    # ...but height comes from the full contour and stays put (decoupled).
    assert abs(d3[0].height - d0[0].height) < 1.0


def test_detection_search_area_filter():
    color, depth = _synthetic_scene()
    fx, fy, cx, cy = _intrinsics()
    params = DetectionParams(cam_to_robot=np.eye(4),
                             search_area_y_min=10000.0, search_area_y_max=20000.0)
    detections, _ = detect_objects(color, depth, fx, fy, cx, cy, params)
    assert detections == []


# -- tracker -----------------------------------------------------------------------

def _det(x=0.0, y=0.0, z=100.0, **kw):
    return TrackedObject(x=x, y=y, z=z, orientation=0.5, length=80.0,
                         width=40.0, height=50.0, **kw)


def test_tracker_candidate_phase_and_promotion():
    tr = VisionTracker()
    # First sighting: only a candidate, no active track yet.
    tr.update([_det(y=-700.0)], 0.0)
    assert tr.get_active_objects() == []
    # Second sighting nearby: promoted to a track with id 1.
    tr.update([_det(y=-690.0)], 0.1)
    objs = tr.get_active_objects()
    assert len(objs) == 1
    assert objs[0].id == 1


def test_tracker_velocity_measured_in_region():
    tr = VisionTracker()
    tr.update([_det(y=-700.0)], 0.0)
    tr.update([_det(y=-700.0)], 0.1)       # promotion (vy = 0)
    tr.update([_det(y=-690.0)], 0.2)       # 100 mm/s within the region
    objs = tr.get_active_objects()
    assert len(objs) == 1
    assert abs(objs[0].vy - 100.0) < 1e-6


def test_tracker_predicts_invisible_tracks():
    tr = VisionTracker()
    tr.set_prediction_velocity(100.0)
    tr.update([_det(y=-700.0)], 0.0)
    tr.update([_det(y=-690.0)], 0.1)       # promote
    # Object disappears; the track must coast with the prediction velocity.
    tr.update([], 0.2)
    objs = tr.get_active_objects()
    assert len(objs) == 1
    assert objs[0].y > -690.0


def test_tracker_takes_measured_y_outside_the_region():
    """Umsetzungsplan 2.4 / Nachtrag 6 Z4. Outside the velocity region the C++
    original coasted y with the belt velocity even when a detection existed.
    That made every velocity estimate circular, and it hid a block toppling on
    placement -- blocks are placed upstream, outside the region."""
    tr = VisionTracker()
    tr.set_prediction_velocity(-100.0)      # belt runs along -y at 100 mm/s
    tr.update([_det(y=100.0)], 0.0)         # upstream, outside the region
    tr.update([_det(y=90.0)], 0.1)          # promote
    tr.update([_det(y=55.0)], 0.2)          # the block topples: centre jumps
    obj = tr.get_active_objects()[0]
    assert obj.y == 55.0                    # measured, not coasted 90 - 10 = 80


def test_tracker_still_coasts_missed_frames_outside_the_region():
    """Only the matched-detection branch changed; a missing detection is still
    bridged with the prediction velocity."""
    tr = VisionTracker()
    tr.set_prediction_velocity(-100.0)
    tr.update([_det(y=100.0)], 0.0)
    tr.update([_det(y=90.0)], 0.1)          # promote
    tr.update([], 0.2)                      # missed frame
    obj = tr.get_active_objects()[0]
    assert abs(obj.y - 80.0) < 1e-9


def test_tracker_deletes_after_max_missed_in_region():
    tr = VisionTracker()
    tr.set_prediction_velocity(0.0)        # stays inside the region
    tr.update([_det(y=-700.0)], 0.0)
    tr.update([_det(y=-700.0)], 0.1)       # promote
    for i in range(2, 6):                  # 4 missed frames > max 3
        tr.update([], 0.1 * i)
    assert tr.get_active_objects() == []


def test_tracker_deletes_outside_conveyor_borders():
    tr = VisionTracker()
    tr.set_prediction_velocity(-1000.0)    # fast toward the conveyor end
    tr.update([_det(y=-1000.0)], 0.0)
    tr.update([_det(y=-1005.0)], 0.1)      # promote (outside region: lenient)
    tr.update([], 0.5)                     # coasts to y < -1080 -> deleted
    assert tr.get_active_objects() == []


def test_tracker_square_keeps_previous_orientation():
    tr = VisionTracker()
    first = _det(y=-700.0, square=True)
    first.orientation = 0.5
    tr.update([first], 0.0)
    second = _det(y=-695.0, square=True)
    second.orientation = 1.4               # flapping minAreaRect angle
    tr.update([second], 0.1)               # promotion resolves vs candidate
    objs = tr.get_active_objects()
    assert len(objs) == 1
    assert abs(objs[0].orientation - 0.5) < 1e-9


def test_tracker_global_velocity_applies_to_all_tracks():
    tr = VisionTracker()
    tr.update([_det(y=-700.0), _det(x=400.0, y=-700.0)], 0.0)
    tr.update([_det(y=-700.0), _det(x=400.0, y=-700.0)], 0.1)
    tr.set_global_velocity(-123.0)
    assert all(o.vy == -123.0 for o in tr.get_active_objects())
