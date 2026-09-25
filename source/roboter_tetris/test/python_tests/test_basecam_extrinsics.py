"""Tests for the base camera calibration (stage 1 and 2), without ROS.

A synthetic setup stands in for the robot: the camera sits at the L6 pose, the
gripper holds the AprilGrid flat with the tool axis horizontal, the jaws hide
the first tag column. The solution has to find the camera again from pixels
alone -- that is the proof of the method before the robot moves.
"""

import json
import math

import cv2
import numpy as np
import pytest

from roboter_tetris.basecam_extrinsics import (
    CheckLimits, ExtrinsicsRecord, GridSpec, PlanParams, Sample, average_detections,
    axis_rotation, belt_displacement_mm, belt_plane_check, belt_sample_points, board_slip,
    cal_from_matrix, camera_moved, depth_points, fit_plane, grid_points, identify_layout,
    invert, layout_candidates, load_record, make_tag_detector, make_transform,
    matrix_from_cal, plan_poses, plan_problems, record_from_dict, record_to_dict,
    reference_corners_world, required_start_height, rotation_angle_deg, save_record, solve_from_references,
    solve_pnp, solve_stage1, validate,
)
from roboter_tetris.vision.detection import build_cam_to_robot

L6 = (-0.7787, 0.7934, 0.9163, 179.46, 0.45, 179.76)
X_TRUE = matrix_from_cal(*L6)
K = np.array([[905.0, 0.0, 640.0], [0.0, 905.0, 360.0], [0.0, 0.0, 1.0]])
D = np.array([0.02, -0.04, 0.0005, -0.0003, 0.01])
IMAGE = (1280, 720)
ROI = (342, 60, 618, 580)  # base_cam default since L21: the belt
BELT_Z = 0.0536
SPEC = GridSpec()

# Flange with the tool axis horizontal, pointing away from the robot base, tool
# y down: the jaws close along tool y, i.e. vertically, so the plate between
# them holds tool x and z and lies flat.
_OUT = np.array([-0.78, 0.76, 0.0]) / np.linalg.norm([-0.78, 0.76])
_TOOL_Y = np.array([0.0, 0.0, -1.0])
_R_START = np.column_stack([np.cross(_TOOL_Y, _OUT), _TOOL_Y, _OUT])
_PIVOT = np.array([-0.78, 0.76, 0.38])
START = make_transform(_R_START, _PIVOT - 0.37 * _OUT)
# Board in the jaws: x along the tool axis from 0.25 m on, face up.
Y_TRUE = make_transform(np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]),
                        (0.088, 0.0, 0.25))


def _project(obj, cam_T_obj):
    rvec, _ = cv2.Rodrigues(cam_T_obj[:3, :3])
    px, _ = cv2.projectPoints(obj, rvec, cam_T_obj[:3, 3], K, D)
    return px.reshape(-1, 2)


def _sample(world_T_flange, rng, noise_px=0.15, flange_T_board=Y_TRUE):
    """Pixels of the visible, unhidden board corners at this flange pose."""
    C = invert(X_TRUE) @ world_T_flange @ flange_T_board
    obj, img = [], []
    for tag in range(SPEC.rows * SPEC.cols):
        if tag % SPEC.cols == 0:
            continue  # first column under the jaws
        corners = SPEC.tag_corners(tag)
        cam = (C @ np.c_[corners, np.ones(4)].T)[:3]
        if np.any(cam[2] <= 0):
            continue
        px = _project(corners, C)
        if np.all((px >= 0) & (px < IMAGE)):
            obj.append(corners)
            img.append(px)
    img = np.vstack(img)
    return Sample(world_T_flange, np.vstack(obj), img + rng.normal(0, noise_px, img.shape))


def _samples(seed=1):
    rng = np.random.default_rng(seed)
    plan = plan_poses(START, PlanParams())
    return {role: [_sample(p.world_T_flange, rng) for p in plan if p.role == role]
            for role in ("kalibrieren", "pruefen", "wiederholen")}


def _pose_error(A, B):
    delta = invert(A) @ B
    return 1000.0 * np.linalg.norm(delta[:3, 3]), rotation_angle_deg(delta[:3, :3])


# -- conventions --------------------------------------------------------------

def test_cal_matches_base_cam_convention():
    assert np.allclose(X_TRUE, build_cam_to_robot(*L6), atol=1e-12)
    cal = cal_from_matrix(X_TRUE)
    got = [cal[k] for k in ("cal_x", "cal_y", "cal_z", "cal_roll", "cal_pitch", "cal_yaw")]
    assert np.allclose(got, L6, atol=1e-9)


# -- grid and detection -------------------------------------------------------

def test_grid_geometry_kalibr_layout():
    pitch = 0.026
    assert SPEC.size_m == pytest.approx((0.280, 0.176))
    assert np.allclose(SPEC.tag_corners(0)[3], [0, 0, 0])            # bottom-left
    assert np.allclose(SPEC.tag_corners(0)[1], [0.02, 0.02, 0])       # top-right
    assert np.allclose(SPEC.tag_corners(1)[3], [pitch, 0, 0])         # along the row
    assert np.allclose(SPEC.tag_corners(11)[3], [0, pitch, 0])        # next row up
    assert np.allclose(SPEC.tag_corners(76)[1], [0.280, 0.176, 0])
    top = GridSpec(origin="top_left")
    assert np.allclose(top.tag_corners(0)[3], [0, 6 * pitch, 0])
    assert SPEC.tag_corners(77) is None and SPEC.tag_corners(-1) is None


def _render_grid(spec, ppm=2000, margin=60):
    tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side = int(round(spec.tag_size_m * ppm))
    w_m, h_m = spec.size_m
    img = np.full((2 * margin + int(round(h_m * ppm)), 2 * margin + int(round(w_m * ppm))),
                  255, np.uint8)
    for tag in range(spec.rows * spec.cols):
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(tag_dict, tag, side)
        else:
            marker = cv2.aruco.drawMarker(tag_dict, tag, side)
        x0, y0 = spec.tag_corners(tag)[0][:2]  # top-left corner, board frame
        u = margin + int(round(x0 * ppm))
        v = margin + int(round((h_m - y0) * ppm))
        img[v:v + side, u:u + side] = marker
    # pinhole at d = 0.5 m that produces exactly this image (pixel edges at -0.5)
    d = 0.5
    Kr = np.array([[ppm * d, 0, margin - 0.5], [0, ppm * d, margin - 0.5 + h_m * ppm], [0, 0, 1]])
    return img, Kr, d


def test_detection_finds_every_tag_and_the_pose():
    img, Kr, d = _render_grid(SPEC)
    detections = make_tag_detector()(img)
    assert sorted(detections) == list(range(77))
    obj, px = grid_points(SPEC, detections)
    T, err = solve_pnp(obj, px, Kr, np.zeros(5))
    assert err < 0.3
    expected = make_transform(np.diag([1.0, -1.0, -1.0]), (0, 0, d))
    shift_mm, angle = _pose_error(expected, T)
    assert shift_mm < 0.5 and angle < 0.1


def test_identify_layout_only_the_true_layout_fits():
    img, Kr, _ = _render_grid(SPEC)
    detections = make_tag_detector()(img)
    ranked = identify_layout(detections, layout_candidates(SPEC), Kr, np.zeros(5))
    assert ranked[0][1] == SPEC and ranked[0][0] < 0.3
    assert all(err > 10 * ranked[0][0] for err, _ in ranked[1:])


def test_average_detections_drops_rare_tags():
    a = {1: np.zeros((4, 2)), 2: np.ones((4, 2))}
    b = {1: np.full((4, 2), 2.0)}
    avg = average_detections([a, b, b], min_fraction=0.8)
    assert list(avg) == [1] and np.allclose(avg[1], 4.0 / 3.0)


# -- pose plan ----------------------------------------------------------------

def test_plan_covers_rotations_and_stays_in_bounds():
    params = PlanParams()
    plan = plan_poses(START, params)
    roles = [p.role for p in plan]
    assert roles.count("kalibrieren") == 20 * params.passes and roles.count("pruefen") == 4
    assert roles[-1] == "wiederholen" and np.allclose(plan[-1].world_T_flange, plan[0].world_T_flange)
    assert np.allclose(plan[0].world_T_flange, START)
    turns = [rotation_angle_deg(p.world_T_flange[:3, :3] @ START[:3, :3].T) for p in plan]
    assert max(turns) > 30.0                      # real rotation diversity
    pivot = lambda T: T[:3, :3] @ [0, 0, params.pivot_along_tool_m] + T[:3, 3]
    for p in plan:
        off = pivot(p.world_T_flange) - pivot(START)
        assert abs(off[0]) <= params.max_shift_m + 1e-9 and abs(off[1]) <= params.max_shift_m + 1e-9
        assert abs(off[2]) <= params.height_step_m + 1e-9
    assert plan_problems(START, plan, params) == []


def test_required_start_height_is_the_limit():
    params = PlanParams()
    level = required_start_height(params)
    assert 0.3 < level < 0.4
    for z, ok in ((level + 0.002, True), (level - 0.02, False)):
        start = START.copy()
        start[:3, 3] += (z - _PIVOT[2]) * np.array([0, 0, 1.0])
        assert (plan_problems(start, plan_poses(start, params), params) == []) is ok


def test_plan_problems_catch_low_start_and_vertical_tool():
    params = PlanParams()
    low = START.copy()
    low[2, 3] = 0.14
    assert any("Mindesthöhe" in m for m in plan_problems(low, plan_poses(low, params), params))
    down = make_transform(np.diag([1.0, -1.0, -1.0]), START[:3, 3])
    assert any("waagerecht" in m for m in plan_problems(down, plan_poses(down, params), params))


# -- stage 1 ------------------------------------------------------------------

def test_stage1_finds_camera_and_board_in_jaws():
    s = _samples()
    result = solve_stage1(s["kalibrieren"], K, D)
    shift_mm, angle = _pose_error(X_TRUE, result.world_T_cam)
    assert shift_mm < 0.5 and angle < 0.03
    y_shift, y_angle = _pose_error(Y_TRUE, result.flange_T_board)
    assert y_shift < 0.5 and y_angle < 0.05
    assert result.rms_px < 0.25 and result.points_rejected < 20
    assert result.position_std_mm < 0.5 and result.rotation_std_deg < 0.05
    check = validate(result, s["pruefen"], K, D)
    assert check.rms_px < 0.3 and check.mean_mm < 1.5


def test_stage1_averages_robot_pose_errors_on_the_belt():
    """The robot reports each pose slightly off (0.1 mm / 0.02 deg) -- the error
    that dominates at the setup. With the default plan the belt stays within 1 mm."""
    rng = np.random.default_rng(8)
    cal = []
    for p in plan_poses(START, PlanParams()):
        if p.role != "kalibrieren":
            continue
        s = _sample(p.world_T_flange, rng, noise_px=0.2)
        s.world_T_flange = make_transform(
            axis_rotation(rng.normal(size=3), abs(rng.normal(0, 0.02))),
            rng.normal(0, 0.0001, 3)) @ s.world_T_flange
        cal.append(s)
    result = solve_stage1(cal, K, D)
    belt = belt_sample_points(X_TRUE, K, IMAGE, BELT_Z, roi=ROI)
    assert belt_displacement_mm(X_TRUE, result.world_T_cam, belt) < 1.0


def test_stage1_survives_outliers():
    s = _samples(seed=2)["kalibrieren"]
    s[3].img[:10] += 25.0
    result = solve_stage1(s, K, D)
    shift_mm, angle = _pose_error(X_TRUE, result.world_T_cam)
    assert result.points_rejected >= 10
    assert shift_mm < 0.5 and angle < 0.03


def test_stage1_drops_a_pose_the_robot_reported_wrong():
    s = _samples(seed=9)["kalibrieren"]
    s[5].world_T_flange = make_transform(np.eye(3), (0.01, 0, 0)) @ s[5].world_T_flange
    result = solve_stage1(s, K, D)
    shift_mm, angle = _pose_error(X_TRUE, result.world_T_cam)
    assert result.poses_dropped == 1 and math.isnan(result.per_sample_rms_px[5])
    assert shift_mm < 0.5 and angle < 0.03


def test_stage1_needs_enough_poses():
    with pytest.raises(ValueError):
        solve_stage1(_samples()["kalibrieren"][:5], K, D)


def test_board_slip_between_first_and_last_visit():
    rng = np.random.default_rng(3)
    first = _sample(START, rng)
    same = _sample(START, rng)
    slipped = _sample(START, rng, flange_T_board=Y_TRUE @ make_transform(np.eye(3), (0.002, 0, 0)))
    assert board_slip(X_TRUE, first, same, K, D)[0] < 0.5
    shift, _ = board_slip(X_TRUE, first, slipped, K, D)
    assert shift == pytest.approx(2.0, abs=0.5)


# -- belt plane ---------------------------------------------------------------

def _belt_depth(world_T_cam, noise_m=0.001, seed=4):
    vs, us = np.mgrid[0:IMAGE[1], 0:IMAGE[0]]
    rays = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones(us.shape)], -1)
    world_rays = rays @ world_T_cam[:3, :3].T
    t = (BELT_Z - world_T_cam[2, 3]) / world_rays[..., 2]   # = camera z, since ray z_cam = 1
    return t + np.random.default_rng(seed).normal(0, noise_m, t.shape)


def test_belt_plane_matches_calibration():
    plane = fit_plane(depth_points(_belt_depth(X_TRUE), K, stride=8))
    tilt, height = belt_plane_check(X_TRUE, plane, BELT_Z)
    assert tilt < 0.05 and abs(height) < 0.5 and plane.rms_mm < 1.5


def test_belt_plane_reveals_wrong_tilt():
    plane = fit_plane(depth_points(_belt_depth(X_TRUE), K, stride=8))
    tilted = X_TRUE @ make_transform(axis_rotation((1, 0, 0), 1.0), (0, 0, 0))
    tilt, _ = belt_plane_check(tilted, plane, BELT_Z)
    assert tilt == pytest.approx(1.0, abs=0.05)


# -- stage 2 ------------------------------------------------------------------

REF_SIZE = 0.07
REF_POSES = {  # tags on the conveyor frame, face up, beside the belt
    100: (-0.60, 0.55, 0.09), 101: (-0.60, 1.05, 0.09),
    102: (-1.00, 0.55, 0.09), 103: (-1.00, 1.05, 0.09),
}


def _reference_world():
    local = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]]) * REF_SIZE / 2
    return {i: local + np.array(p) for i, p in REF_POSES.items()}


def _reference_pixels(world_T_cam, rng, noise_px=0.15):
    cam_T_world = invert(world_T_cam)
    out = {}
    for i, corners in _reference_world().items():
        px = _project(corners, cam_T_world)
        out[i] = px + rng.normal(0, noise_px, px.shape)
    return out


def test_reference_tags_are_located_in_world():
    rng = np.random.default_rng(5)
    seen = _reference_pixels(X_TRUE, rng)
    for i, truth in _reference_world().items():
        got = reference_corners_world(X_TRUE, seen[i], REF_SIZE, K, D)
        assert np.max(np.linalg.norm(got - truth, axis=1)) < 0.004  # depth of a single tag


def test_stage2_recovers_moved_camera_and_check_flags_it():
    rng = np.random.default_rng(6)
    refs = _reference_world()
    moved = make_transform(axis_rotation((0.3, 1, 0), 0.6), (0.004, -0.008, 0.003)) @ X_TRUE
    X2, err, used = solve_from_references(refs, _reference_pixels(moved, rng), K, D, guess=X_TRUE)
    assert used == [100, 101, 102, 103] and err < 0.3
    shift_mm, angle = _pose_error(moved, X2)
    assert shift_mm < 1.5 and angle < 0.05
    belt = belt_sample_points(X_TRUE, K, IMAGE, BELT_Z)
    limits = CheckLimits()
    assert camera_moved(X_TRUE, X2, belt, limits)[0]
    X_same, _, _ = solve_from_references(refs, _reference_pixels(X_TRUE, rng), K, D, guess=X_TRUE)
    is_moved, shift, _ = camera_moved(X_TRUE, X_same, belt, limits)
    assert not is_moved and shift < 1.0


def test_stage2_needs_two_reference_tags():
    rng = np.random.default_rng(7)
    seen = {100: _reference_pixels(X_TRUE, rng)[100]}
    with pytest.raises(ValueError):
        solve_from_references(_reference_world(), seen, K, D)


def test_belt_displacement():
    belt = belt_sample_points(X_TRUE, K, IMAGE, BELT_Z)
    assert len(belt) == 25 and np.allclose(belt[:, 2], BELT_Z)
    assert belt_displacement_mm(X_TRUE, X_TRUE, belt) == pytest.approx(0.0, abs=1e-9)
    shifted = make_transform(np.eye(3), (0.003, 0.004, 0)) @ X_TRUE
    assert belt_displacement_mm(X_TRUE, shifted, belt) == pytest.approx(5.0)


# -- file ---------------------------------------------------------------------

def test_record_roundtrip(tmp_path):
    record = ExtrinsicsRecord(world_T_cam=X_TRUE, method="stufe1",
                              quality={"rms_px": 0.21}, reference_size_m=REF_SIZE,
                              references=_reference_world(), grid=SPEC.to_dict())
    path = str(tmp_path / "sub" / "base_cam_extrinsics.json")
    save_record(path, record)
    back = load_record(path)
    assert np.allclose(back.world_T_cam, X_TRUE, atol=1e-8)
    assert back.method == "stufe1" and back.quality == {"rms_px": 0.21}
    assert sorted(back.references) == [100, 101, 102, 103]
    assert np.allclose(back.references[101], _reference_world()[101], atol=1e-6)
    cal = json.load(open(path))["cal"]
    assert cal["cal_roll"] == pytest.approx(179.46) and cal["cal_z"] == pytest.approx(0.9163)
    assert not (tmp_path / "sub" / "base_cam_extrinsics.json.tmp").exists()


def test_broken_files_are_refused(tmp_path):
    good = record_to_dict(ExtrinsicsRecord(world_T_cam=X_TRUE, method="stufe1"))
    with pytest.raises(ValueError, match="schema"):
        record_from_dict({**good, "schema": "etwas anderes"})
    bent = [row[:] for row in good["matrix"]]
    bent[0][0] += 0.01
    with pytest.raises(ValueError, match="orthonormal"):
        record_from_dict({**good, "matrix": bent})
    with pytest.raises(ValueError, match="4x4"):
        record_from_dict({**good, "matrix": [[1, 0], [0, 1]]})
    broken = tmp_path / "kaputt.json"
    broken.write_text("{ nicht json")
    with pytest.raises(ValueError, match="JSON"):
        load_record(str(broken))
    with pytest.raises(ValueError, match="nicht lesbar"):
        load_record(str(tmp_path / "fehlt.json"))
