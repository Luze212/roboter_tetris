"""Tests for the run sequence of the base camera calibration, without ROS.

A simulated robot follows the commanded flange pose after a short delay, a
simulated camera sees the board in the jaws and the reference tags on the
conveyor frame. The geometry is the one of test_basecam_extrinsics.
"""

import numpy as np
import pytest

from roboter_tetris.basecam_extrinsics import (
    ExtrinsicsRecord, GridSpec, PlanParams, axis_rotation, invert, make_transform,
    required_start_height, rotation_angle_deg,
)
from roboter_tetris.calibration_run import (
    FEHLER, FERTIG, Camera, CalibrationRun, Frame, RunParams,
)
from test_basecam_extrinsics import (
    D, IMAGE, K, SPEC, START, X_TRUE, Y_TRUE, _belt_depth, _reference_world,
)

CAMERA = Camera(K, D, IMAGE)
_TAGS = [t for t in range(SPEC.rows * SPEC.cols) if t % SPEC.cols]   # column 0 under the jaws
_CORNERS = np.vstack([SPEC.tag_corners(t) for t in _TAGS])
_DEPTH = _belt_depth(X_TRUE)


def _project(points, cam_T_points):
    import cv2
    rvec, _ = cv2.Rodrigues(cam_T_points[:3, :3])
    px, _ = cv2.projectPoints(points, rvec, cam_T_points[:3, 3], K, D)
    return px.reshape(-1, 2)


def _detections(flange, rng, X=X_TRUE, board=True, flange_T_board=Y_TRUE, refs=True):
    out = {}
    if board:
        C = invert(X) @ flange @ flange_T_board
        z = (C[:3, :3] @ _CORNERS.T)[2] + C[2, 3]
        px = _project(_CORNERS, C).reshape(-1, 4, 2)
        for tag, corners, depth in zip(_TAGS, px, z.reshape(-1, 4)):
            if np.all(depth > 0) and np.all((corners >= 0) & (corners < IMAGE)):
                out[tag] = corners + rng.normal(0, 0.15, corners.shape)
    if refs:
        for tag, corners in _reference_world().items():
            px = _project(corners, invert(X))
            out[tag] = px + rng.normal(0, 0.15, px.shape)
    return out


class Robot:
    """Arrives at a new target three cycles after it was commanded."""

    def __init__(self, flange):
        self.flange = flange.copy()
        self._target = None
        self._delay = 0

    def follow(self, target):
        if target is None:
            return
        if self._target is None or not np.allclose(target, self._target):
            self._target = target.copy()
            self._delay = 3
        if self._delay:
            self._delay -= 1
            if not self._delay:
                self.flange = self._target.copy()


def _run(run, start=START, max_steps=6000, seed=0, **seen):
    rng = np.random.default_rng(seed)
    robot, now, targets = Robot(start), 0.0, []
    for _ in range(max_steps):
        now += 0.05
        frame = None
        if run.wants_frames():
            board_fn = seen.get("flange_T_board")
            fb = board_fn(run) if board_fn else Y_TRUE
            frame = Frame(now, _detections(robot.flange, rng, flange_T_board=fb,
                                           **{k: v for k, v in seen.items()
                                              if k != "flange_T_board"}), _DEPTH)
        target = run.step(now, robot.flange.copy(), CAMERA, frame)
        targets.append(target)
        robot.follow(target)
        if not run.running:
            break
    return targets, run.pop_events()


def _error(A, B):
    delta = invert(A) @ B
    return 1000.0 * np.linalg.norm(delta[:3, 3]), rotation_angle_deg(delta[:3, :3])


def _moved_camera():
    return make_transform(axis_rotation((0.3, 1, 0), 0.6), (0.004, -0.008, 0.003)) @ X_TRUE


# -- stage 1 ------------------------------------------------------------------

def test_stage1_runs_through_and_finds_the_camera():
    run = CalibrationRun(RunParams(), None, X_TRUE)
    targets, events = _run(run)
    assert run.state == FERTIG, events
    assert np.allclose(targets[0], START)                        # holds first
    assert np.allclose(targets[-1], START)                       # and returns
    shift, angle = _error(X_TRUE, run.result.world_T_cam)
    assert shift < 0.5 and angle < 0.03
    q = run.result.quality
    assert q["posen"] == 40 and q["posen_ausgelassen"] == 0
    assert q["rutschen_mm"] < 0.5 and q["pruefposen_mm_mittel"] < 1.0
    assert q["gegen_vorher_band_mm"] < 0.5
    assert abs(q["ebene_neigung_grad"]) < 0.1 and abs(q["ebene_hoehe_mm"]) < 1.0
    assert sorted(run.result.references) == [100, 101, 102, 103]
    for tag, truth in _reference_world().items():
        assert np.max(np.linalg.norm(run.result.references[tag] - truth, axis=1)) < 0.004
    assert run.result.method == "stufe1_farbkamera" and run.result.grid == SPEC.to_dict()


def test_raw_data_keeps_what_the_solution_saw_also_after_a_refusal():
    import json
    def slipping(run):
        return Y_TRUE @ make_transform(np.eye(3), (0.002, 0, 0)) if run.progress[0] >= 20 else Y_TRUE
    run = CalibrationRun(RunParams(), None, X_TRUE)
    _run(run, flange_T_board=slipping)
    raw = json.loads(json.dumps(run.raw_data()))                 # must be plain JSON
    assert run.state == FEHLER and len(raw["samples"]["kalibrieren"]) == 40
    first = raw["samples"]["kalibrieren"][0]
    assert np.array(first["flange"]).shape == (4, 4) and len(first["obj"]) == len(first["img"])
    assert raw["report"]["rutschen_mm"] > 1.0 and raw["grid"] == SPEC.to_dict()
    # depth at every pose: the board region of the depth image, and the belt at the start
    assert all(smp["depth_plane"] is not None and len(smp["depth_grid"]) > 100
               for smp in raw["samples"]["kalibrieren"])
    assert len(raw["belt_depth_grid"]) > 1000 and raw["camera_z_from_depth"] is not None


def test_stage1_finds_the_board_layout_itself():
    params = RunParams(grid=GridSpec(rows=11, cols=7))           # set up the wrong way round
    run = CalibrationRun(params, None, X_TRUE)
    _, events = _run(run)
    assert run.state == FERTIG, events
    assert any("Board-Layout erkannt: 7 Zeilen x 11 Spalten" in e for e in events)


def test_stage1_refuses_a_start_too_low_and_never_moves():
    low = START.copy()
    low[2, 3] -= 0.15
    run = CalibrationRun(RunParams(), None, X_TRUE)
    targets, events = _run(run, start=low)
    assert run.state == FEHLER
    assert any("Plan nicht fahrbar" in e for e in events)
    assert all(np.allclose(t, low) for t in targets if t is not None)


def test_stage1_without_board_in_view_never_moves():
    run = CalibrationRun(RunParams(), None, X_TRUE)
    targets, events = _run(run, board=False)
    assert run.state == FEHLER and any("Board nicht im Bild" in e for e in events)
    assert all(np.allclose(t, START) for t in targets if t is not None)


def test_stage1_board_slipping_in_the_jaws_writes_nothing_and_returns():
    def slipping(run):  # 2 mm along the board after the first pass
        return Y_TRUE @ make_transform(np.eye(3), (0.002, 0, 0)) if run.progress[0] >= 20 else Y_TRUE
    run = CalibrationRun(RunParams(), None, X_TRUE)
    targets, events = _run(run, flange_T_board=slipping)
    assert run.state == FEHLER and run.result is None
    assert any("Gütegrenzen" in e for e in events)
    assert np.allclose(targets[-1], START)


def test_start_height_of_the_default_plan():
    # the operator needs this number when driving the start pose
    assert required_start_height(PlanParams()) == pytest.approx(0.294, abs=0.002)


# -- stage 2 and check ----------------------------------------------------------

def _stored(X=X_TRUE, with_refs=True):
    return ExtrinsicsRecord(world_T_cam=X, method="stufe1", created="2026-09-25",
                            reference_size_m=0.07,
                            references=_reference_world() if with_refs else {})


def test_stage2_solves_a_moved_camera_without_motion():
    moved = _moved_camera()
    run = CalibrationRun(RunParams(mode="stufe2"), _stored(), X_TRUE)
    targets, events = _run(run, X=moved, board=False)
    assert run.state == FERTIG, events
    assert all(t is None for t in targets)                        # never commands
    shift, angle = _error(moved, run.result.world_T_cam)
    assert shift < 1.5 and angle < 0.05
    assert run.moved and run.result.method == "stufe2"
    assert sorted(run.result.references) == [100, 101, 102, 103]
    assert any("nachkalibrieren" in e for e in events)


def test_check_reports_an_unchanged_camera_and_writes_nothing():
    run = CalibrationRun(RunParams(mode="pruefen"), _stored(), X_TRUE)
    targets, events = _run(run, board=False)
    assert run.state == FERTIG and run.moved is False and run.result is None
    assert run.report["gegen_datei_band_mm"] < 1.0
    assert all(t is None for t in targets)


def test_check_needs_reference_tags_in_the_file():
    run = CalibrationRun(RunParams(mode="pruefen"), _stored(with_refs=False), X_TRUE)
    _, events = _run(run, board=False)
    assert run.state == FEHLER and any("Referenzmarken" in e for e in events)


def test_bad_parameters_stop_before_anything_happens():
    run = CalibrationRun(RunParams(mode="irgendwas"), None, X_TRUE)
    assert run.state == FEHLER and "Betriebsart" in run.pop_events()[0]
    overlap = RunParams(reference_first_id=50)
    assert any("überschneiden" in m for m in overlap.problems())


def test_flange_floor_lifts_poses_and_keeps_their_turns():
    from roboter_tetris.basecam_extrinsics import plan_poses, plan_problems
    floor = START[2, 3] - 0.06
    tilted = dict(max_tilt_deg=20.0, max_pitch_deg=12.0)
    free = plan_poses(START, PlanParams(**tilted))
    lifted = plan_poses(START, PlanParams(min_flange_z_m=floor, **tilted))
    assert min(p.world_T_flange[2, 3] for p in free) < floor - 0.05
    assert min(p.world_T_flange[2, 3] for p in lifted) == pytest.approx(floor)
    for b in lifted:                               # same turns, order may differ
        assert any(np.allclose(a.world_T_flange[:3, :3], b.world_T_flange[:3, :3]) for a in free)
    assert plan_problems(START, lifted, PlanParams(min_flange_z_m=floor, **tilted)) == []


def test_stage1_with_the_flange_floor_still_finds_the_camera():
    run = CalibrationRun(RunParams(plan=PlanParams(min_flange_z_m=START[2, 3] - 0.06)), None, X_TRUE)
    _, events = _run(run)
    assert run.state == FERTIG, events
    shift, angle = _error(X_TRUE, run.result.world_T_cam)
    assert shift < 0.5 and angle < 0.03

