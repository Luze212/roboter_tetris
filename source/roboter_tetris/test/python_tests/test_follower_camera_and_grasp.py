"""Tests for the follower, stages 4c (robot camera) and 4d (grasp cycle).

No ROS, no numpy -- runs with plain python3. Robot frame throughout (Nachtrag 8
/ F1): belt at x = -0.816, running along -y.
"""

import math
from dataclasses import replace

from roboter_tetris.contracts import (
    OUTCOME_LOST, OUTCOME_MISSED_GRIP, OUTCOME_PLACED, OUTCOME_TOO_LATE,
    STATE_ABORT, STATE_DESCEND, STATE_FOLLOW, STATE_GRASP, STATE_LIFT,
    STATE_PLACE, STATE_RELEASE, STATE_WAIT, ObjectPosition, TrackEntry,
    pack_target, unpack_target,
)
from roboter_tetris.follower_logic import (
    CameraBlend, FollowerCore, FollowerParams, GripperFeedback, Pose,
    PoseHistory, camera_to_world, quat_from_rpy, robot_cam_world_xy, rotate,
    vertical_orientation,
)

PARAMS = FollowerParams(
    ws_x_min=-1.10, ws_x_max=-0.20, ws_y_min=-0.60, ws_y_max=0.60,
    ws_z_min=0.30, ws_z_max=0.80,
    observe_x=-0.80, observe_y=-0.10, observe_z=0.60, observe_yaw_deg=90.0)
BELT_X = -0.816
V = (0.0, -0.100)
DOWN = vertical_orientation(0.0)
OPEN = GripperFeedback(motion_done=True, has_object=False)


def _s4(t, y, x=BELT_X, tid=1, has=True, plane=0.9, height=0.1):
    track = TrackEntry(id=float(tid), color=0.0, x=x, y=y, z=0.1036,
                       orientation=0.3, length=0.05, width=0.05, height=height,
                       status=0.0, vx=0.0, vy=-0.1, v_change=0.0, ori_quality=0.9)
    return unpack_target(pack_target(t, has, -0.05, plane, V,
                                      track if has else None))


def _conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _s2(t, flange, world_x, world_y, block_height, params=PARAMS, valid=True):
    """Inverse of the follower's chain: what robot_cam would report for a
    block top at (world_x, world_y) seen from ``flange``."""
    top_z = params.belt_surface_z_m + block_height
    rel = (world_x - flange.x, world_y - flange.y, top_z - flange.z)
    in_flange = rotate(_conj(flange.orientation), rel)
    q_he = quat_from_rpy(math.radians(params.handeye_roll_deg),
                         math.radians(params.handeye_pitch_deg),
                         math.radians(params.handeye_yaw_deg))
    p = rotate(_conj(q_he), (in_flange[0] - params.handeye_x,
                             in_flange[1] - params.handeye_y,
                             in_flange[2] - params.handeye_z))
    z_belt = p[2] + block_height
    return ObjectPosition(t=t, valid=1.0 if valid else 0.0,
                          x=p[0] * z_belt / p[2], y=p[1] * z_belt / p[2],
                          z_belt=z_belt, orientation=0.0)


# == 4c: hand-eye and height correction ============================================================

def test_hand_eye_rpy_reproduce_the_calibration_quaternion():
    """Calibration_results_final.yaml: qw 0.6978, qx 0.000363, qy 0.01987, qz 0.716."""
    q = quat_from_rpy(math.radians(PARAMS.handeye_roll_deg),
                      math.radians(PARAMS.handeye_pitch_deg),
                      math.radians(PARAMS.handeye_yaw_deg))
    assert all(abs(a - b) < 2e-4 for a, b in zip(q, (0.6978, 0.000363, 0.01987, 0.716)))


def test_camera_point_to_world_against_a_hand_calculated_point():
    """Direction flange -> camera, spelled out by hand (C1). Hand-eye yaw 90 deg,
    no roll/pitch; flange straight down (yaw 0) at (-0.8, 0, 0.6):
      Rz(90) * (0.01, 0.02, 0.4)        = (-0.02, 0.01, 0.4)
      + t (0.1087, -0.03436, -0.05987)  = (0.0887, -0.02436, 0.34013)
      tool down (x, y, z) -> (x, -y, -z) + flange = (-0.7113, 0.02436, 0.25987)
    The wrong direction (camera -> flange) lands elsewhere, and still looks
    plausible -- that is why this is checked against a number."""
    params = replace(PARAMS, handeye_roll_deg=0.0, handeye_pitch_deg=0.0,
                     handeye_yaw_deg=90.0)
    flange = Pose(-0.8, 0.0, 0.6, *DOWN)
    world = camera_to_world(flange, (0.01, 0.02, 0.4), params)
    assert all(abs(a - b) < 1e-9 for a, b in zip(world, (-0.7113, 0.02436, 0.25987)))


def test_height_correction_comes_first():
    """robot_cam back-projects with the belt distance; the point is on the top
    face. For a 100 mm block at 0.5 m belt distance the raw x is 25 % too large."""
    params = replace(PARAMS, handeye_x=0.0, handeye_y=0.0, handeye_z=0.0,
                     handeye_roll_deg=0.0, handeye_pitch_deg=0.0, handeye_yaw_deg=0.0)
    flange = Pose(-0.8, 0.0, 0.6, *DOWN)
    m = ObjectPosition(t=0.0, valid=1.0, x=0.02, y=0.0, z_belt=0.5, orientation=0.0)
    x, y = robot_cam_world_xy(m, 0.1, flange, params)
    assert abs(x - (-0.8 + 0.016)) < 1e-12 and abs(y) < 1e-12
    assert robot_cam_world_xy(m._replace(z_belt=0.09), 0.1, flange, params) is None


def test_synthetic_measurements_round_trip():
    flange = Pose(-0.8, -0.1, 0.6, *vertical_orientation(1.2))
    m = _s2(0.0, flange, -0.83, -0.12, 0.1)
    x, y = robot_cam_world_xy(m, 0.1, flange, PARAMS)
    assert abs(x - (-0.83)) < 1e-9 and abs(y - (-0.12)) < 1e-9


# == 4c: the blend ===================================================================================

def _history(pose, t0=0.0, t1=2.0):
    history = PoseHistory(max_age_s=5.0)
    for k in range(int((t1 - t0) * 100) + 1):
        history.add(t0 + k * 0.01, pose)
    return history


FLANGE = Pose(-0.816, -0.20, 0.60, *vertical_orientation(math.radians(90)))
BOTH = replace(PARAMS, weight_along=1.0, weight_across=1.0)


def _feed(blend, params, offset=(0.0, 0.012), seconds=1.0, valid=True, y0=-0.20,
          history=None, t0=0.0):
    """Base camera sees the block at y0; the true block is ``offset`` away."""
    history = history or _history(FLANGE, t0, t0 + seconds + 1.0)
    target = _s4(t0, y0)
    applied = (0.0, 0.0)
    for k in range(int(seconds * 100)):
        now = t0 + k * 0.01
        m = None
        if k % 3 == 0:
            age = now - target.t
            m = _s2(now, FLANGE, target.x + offset[0], target.y + target.vy * age + offset[1],
                    0.1, params, valid=valid)
        applied = blend.update(now, 0.01, m, target, history, params)
    return applied, target


def test_without_weights_the_robot_camera_has_no_effect():
    blend = CameraBlend()
    applied, _ = _feed(blend, PARAMS)
    assert applied == (0.0, 0.0) and blend.ramp == 0.0


def test_correction_is_learned_and_faded_in():
    blend = CameraBlend()
    applied, _ = _feed(blend, BOTH, offset=(0.0, 0.012))
    assert abs(applied[1] - 0.012) < 1e-6 and abs(applied[0]) < 1e-6
    assert blend.ramp == 1.0


def test_weights_act_per_belt_axis():
    """Belt along -y: 'along' is y, 'across' is x."""
    blend = CameraBlend()
    applied, _ = _feed(blend, replace(PARAMS, weight_along=0.0, weight_across=1.0),
                       offset=(0.008, 0.012))
    assert abs(applied[0] - 0.008) < 1e-6 and abs(applied[1]) < 1e-6


def test_ramp_takes_w_ramp_s():
    blend = CameraBlend()
    _feed(blend, BOTH, seconds=0.10)
    assert 0.3 < blend.ramp < 0.6                   # halfway after ~0.1 s of 0.2 s


def test_invalid_or_stale_measurements_fade_out():
    blend = CameraBlend()
    history = _history(FLANGE, 0.0, 4.0)
    _feed(blend, BOTH, history=history)
    _feed(blend, BOTH, valid=False, seconds=0.3, history=history, t0=1.0)
    assert blend.ramp == 0.0
    blend2 = CameraBlend()
    _feed(blend2, BOTH, history=history)
    target = _s4(1.0, -0.20)
    for k in range(60):                   # no new images: stale after 0.3 s,
        blend2.update(1.0 + k * 0.01, 0.01, None, target, history, BOTH)  # faded 0.2 s later
    assert blend2.ramp == 0.0


def test_a_different_block_in_view_is_rejected():
    """R4: a correction beyond max_correction_m is another object."""
    blend = CameraBlend()
    applied, _ = _feed(blend, BOTH, offset=(0.0, 0.080))
    assert applied == (0.0, 0.0) and blend.ramp == 0.0
    assert any("R4" in e for e in blend.events)


def test_image_time_outside_the_flange_history_is_reported():
    blend = CameraBlend()
    _feed(blend, BOTH, history=_history(FLANGE, 10.0, 11.0))
    assert blend.ramp == 0.0
    assert any("Ringpuffer" in e for e in blend.events)


def test_frozen_correction_ignores_new_measurements():
    """F2: from ABSENKEN on the camera leaves its range -- the correction must
    stay, not fade out."""
    blend = CameraBlend()
    history = _history(FLANGE, 0.0, 4.0)
    applied, target = _feed(blend, BOTH, history=history)
    blend.freeze(target, BOTH)
    later, _ = _feed(blend, BOTH, valid=False, seconds=0.5, history=history, t0=1.0)
    assert later == applied


# == 4d: grasp height and the grasp cycle, step by step ============================================

def test_grip_height_is_mid_block_but_never_too_close_to_the_belt():
    assert abs(PARAMS.grip_flange_z(0.100) - (0.0536 + 0.050 + 0.235)) < 1e-12
    assert abs(PARAMS.grip_flange_z(0.020) - (0.0536 + 0.015 + 0.235)) < 1e-12


def _following_core(params=PARAMS, y=-0.02, plane=0.9, height=0.1):
    """A core in FOLGEN whose flange sits exactly on the predicted block."""
    core = FollowerCore(params)
    at = Pose(BELT_X, y - 0.1 * 0.0, params.observe_z, *vertical_orientation(-math.pi / 2))
    core.step(at, None, 0.0, None, OPEN)
    core.step(at, _s4(0.0, y, plane=plane, height=height), 0.0, None, OPEN)
    assert core.state == STATE_FOLLOW
    return core


def _on_block(core, t, y, z=None, plane=0.9, height=0.1, gripper=OPEN, has=True):
    """One cycle with the flange exactly on the block predicted for t."""
    z = core.params.observe_z if z is None else z
    flange = Pose(BELT_X, y, z, *vertical_orientation(-math.pi / 2))
    return core.step(flange, _s4(t, y, plane=plane, height=height, has=has), t,
                     None, gripper)


def test_descend_starts_after_stable_cycles_within_tolerance():
    core = _following_core()                   # 1st stable cycle
    for k in range(1, 9):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert core.state == STATE_FOLLOW          # 9 cycles
    out = _on_block(core, 0.09, -0.029)
    assert out.state == STATE_DESCEND          # the 10th
    for k in range(10, 20):
        out = _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert abs(out.target.z - (0.60 - 0.15 * 0.10)) < 1e-9     # 0.15 m/s for 0.1 s


def test_crossing_the_grasp_plane_before_descending_is_outcome_3():
    core = _following_core(plane=0.10)                         # plane at y = -0.10
    _on_block(core, 0.01, -0.02, z=0.70, plane=0.10)           # out of tolerance
    _on_block(core, 0.02, -0.11, z=0.70, plane=0.10)
    assert core.picked == (1.0, 1.0, float(OUTCOME_TOO_LATE))


def test_the_plane_no_longer_counts_once_descending():
    core = _following_core(plane=0.10)
    for k in range(1, 11):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k, plane=0.10)
    assert core.state == STATE_DESCEND
    for k in range(11, 120):                                   # on past y = -0.10
        _on_block(core, k * 0.01, -0.02 - 0.001 * k, plane=0.10)
    assert core.state == STATE_DESCEND and core.picked[0] == 0.0


def test_growing_error_while_descending_goes_back_up_to_follow():
    core = _following_core()
    for k in range(1, 11):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert core.state == STATE_DESCEND
    flange = Pose(BELT_X + 0.02, -0.04, 0.55, *vertical_orientation(-math.pi / 2))
    out = core.step(flange, _s4(0.12, -0.032), 0.12, None, OPEN)
    assert out.state == STATE_FOLLOW and out.target.z == PARAMS.observe_z


def _descended_core(height=0.1):
    core = _following_core(height=height)
    for k in range(1, 11):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k, height=height)
    grip = PARAMS.grip_flange_z(height)
    out = _on_block(core, 0.20, -0.04, z=grip, height=height)
    assert out.state == STATE_GRASP and out.gripper_close
    return core, grip


def test_reaching_grip_height_closes_the_gripper_with_frozen_yaw():
    core, grip = _descended_core()
    out = _on_block(core, 0.21, -0.041, z=grip)
    assert out.target.z == grip and out.gripper_close


def test_has_object_leads_to_lift():
    core, grip = _descended_core()
    _on_block(core, 0.21, -0.041, z=grip, gripper=GripperFeedback(False, False))
    out = _on_block(core, 0.70, -0.09, z=grip, gripper=GripperFeedback(True, True))
    assert out.state == STATE_LIFT and core.holding and out.gripper_close


def test_motion_done_without_object_is_a_missed_grip_and_opens_first():
    core, grip = _descended_core()
    _on_block(core, 0.21, -0.041, z=grip, gripper=GripperFeedback(False, False))
    out = _on_block(core, 0.70, -0.09, z=grip, gripper=GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_MISSED_GRIP))
    assert out.state == STATE_ABORT and out.gripper_close is False
    # Opening: the flange holds still until the gripper reports it is open.
    flange = Pose(BELT_X, -0.09, grip, *vertical_orientation(-math.pi / 2))
    out = core.step(flange, _s4(0.71, -0.091, has=False), 0.71, None,
                    GripperFeedback(False, False))
    assert out.target.z == grip
    core.step(flange, None, 0.90, None, GripperFeedback(True, False))
    out = core.step(flange, None, 0.91, None, GripperFeedback(True, False))
    assert out.target.z == PARAMS.transfer_height_m          # now up


def test_grasp_timeout_is_a_missed_grip():
    core, grip = _descended_core()
    _on_block(core, 0.21, -0.041, z=grip, gripper=GripperFeedback(True, False))
    _on_block(core, 2.30, -0.25, z=grip, gripper=GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_MISSED_GRIP))


def _lifting_core():
    core, grip = _descended_core()
    _on_block(core, 0.21, -0.041, z=grip, gripper=GripperFeedback(False, False))
    _on_block(core, 0.70, -0.09, z=grip, gripper=GripperFeedback(True, True))
    assert core.state == STATE_LIFT
    return core, grip


def test_withdrawn_target_does_not_abort_a_held_block():
    """Z12: once lifted, the block leaves the image, its ID vanishes and the
    target is withdrawn -- that is expected, not a reason to drop it."""
    core, grip = _lifting_core()
    flange = Pose(BELT_X, -0.10, grip + 0.05, *vertical_orientation(-math.pi / 2))
    first = core.step(flange, _s4(0.8, 0.0, has=False), 0.80, None,
                      GripperFeedback(True, True))
    out = core.step(flange, _s4(0.9, 0.0, has=False), 0.90, None,
                    GripperFeedback(True, True))
    assert out.state == STATE_LIFT and core.picked[0] == 0.0
    assert abs((out.target.y - first.target.y) - (-0.1 * 0.1)) < 1e-9   # with the belt


def test_lift_goes_up_then_to_the_place_pose_and_releases():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    yaw = vertical_orientation(-math.pi / 2)
    core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.9, None, held)
    out = core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.91, None, held)
    assert out.state == STATE_LIFT and out.target.z == PARAMS.transfer_height_m
    out = core.step(Pose(BELT_X, -0.10, PARAMS.transfer_height_m, *yaw), None, 1.5,
                    None, held)
    assert out.state == STATE_PLACE and out.target == PARAMS.place_pose()
    at_place = PARAMS.place_pose()
    out = core.step(at_place, None, 4.0, None, held)
    assert out.state == STATE_RELEASE and out.gripper_close is False
    core.step(at_place, None, 4.05, None, GripperFeedback(False, True))   # opening
    out = core.step(at_place, None, 4.3, None, GripperFeedback(True, False))
    assert out.state == STATE_WAIT
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED))
    assert not core.has_aborted and not core.holding


def test_a_short_has_object_flicker_is_not_a_lost_block():
    core, grip = _lifting_core()
    flange = Pose(BELT_X, -0.10, grip + 0.02, *vertical_orientation(-math.pi / 2))
    for t in (0.80, 0.81, 0.82, 0.83, 0.84):          # 40 ms of flicker, 5 cycles
        core.step(flange, None, t, None, GripperFeedback(True, False))
    core.step(flange, None, 0.85, None, GripperFeedback(True, True))
    assert core.state == STATE_LIFT
    core.step(flange, None, 0.90, None, GripperFeedback(True, False))
    core.step(flange, None, 1.01, None, GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_LOST))


def test_abort_with_a_block_in_the_gripper_still_places_it():
    """Z12: gripper stays closed, straight up, place pose, open there, outcome 0."""
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    flange = Pose(BELT_X, -0.10, grip + 0.02, *vertical_orientation(-math.pi / 2))
    core.abort("Test: künstlicher Abbruch", flange, 0.8)
    out = core.step(flange, None, 0.81, None, held)
    assert out.state == STATE_ABORT and out.gripper_close
    assert out.target.z == PARAMS.transfer_height_m and out.target.x == BELT_X
    up = flange._replace(z=PARAMS.transfer_height_m)
    out = core.step(up, None, 1.5, None, held)
    assert out.state == STATE_PLACE and out.gripper_close
    at_place = PARAMS.place_pose()
    core.step(at_place, None, 4.0, None, held)
    core.step(at_place, None, 4.05, None, GripperFeedback(False, True))
    core.step(at_place, None, 4.3, None, GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED)) and core.has_aborted


def test_unreachable_place_pose_keeps_the_block_instead_of_dropping_it():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    yaw = vertical_orientation(-math.pi / 2)
    stuck = Pose(BELT_X, -0.10, PARAMS.transfer_height_m, *yaw)
    core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.9, None, held)
    core.step(stuck, None, 1.0, None, held)
    assert core.state == STATE_PLACE
    core.step(stuck, None, 9.5, None, held)       # 1st timeout: abort with the
    assert core.state == STATE_PLACE and core.holding   # block, already up: retry
    assert any("Zeitüberschreitung in ABLEGEN" in e for e in core.pop_events())
    for t in (18.0, 25.0, 40.0):                               # 2nd: hold on
        out = core.step(stuck, None, t, None, held)
        assert out.state == STATE_PLACE and out.gripper_close
    assert any("Eingriff" in e for e in core.pop_events())


def test_release_without_motion_confirmation_still_finishes():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    at_place = PARAMS.place_pose()
    core._enter(STATE_PLACE, "Test", 5.0)
    core.step(at_place, None, 5.0, None, held)
    out = core.step(at_place, None, 7.2, None, GripperFeedback(True, True))
    assert out.state == STATE_WAIT and core.picked[2] == float(OUTCOME_PLACED)


def test_robot_camera_can_be_required_for_the_grasp():
    params = replace(PARAMS, require_robot_cam_for_grasp=True)
    core = _following_core(params=params)
    for k in range(1, 30):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert core.state == STATE_FOLLOW                          # no camera, no grasp


# == Closed loop: two full runs ======================================================================

class _Gripper:
    """Robotiq stand-in: motion takes ``duration`` s after a command; closing
    on a block within ``reach`` gives has_object."""

    def __init__(self, duration=0.4, reach=0.008):
        self.duration, self.reach = duration, reach
        self.closed = False
        self.busy_until = 0.0
        self.has_object = False

    def command(self, close, now, flange, block):
        if close != self.closed:
            self.closed = close
            self.busy_until = now + self.duration
            if close:
                self.has_object = (block is not None
                                   and math.hypot(flange[0] - block[0],
                                                  flange[1] - block[1]) <= self.reach)
            else:
                self.has_object = False

    def feedback(self, now):
        return GripperFeedback(now >= self.busy_until, self.has_object)


def _run_cycle(params, base_cam_offset=(0.0, 0.0), seconds=25.0):
    """The whole loop: moving block, base camera (optionally off by an offset)
    at 30 Hz with 50 ms latency, robot camera at 30 Hz while the flange is at
    observation height, first-order attractor K = 5 capped at 0.25 m/s,
    gripper stand-in. After has_object the block rides in the gripper and the
    target is withdrawn -- as on the real belt."""
    core = FollowerCore(params)
    flange = list(params.observe_pose().position)
    gripper = _Gripper()
    states, min_z, ramp_max = [], 1.0, 0.0
    lifted = False
    for step in range(int(seconds * 100)):
        now = step * 0.01
        frame_t = math.floor((now - 0.05) * 30) / 30
        block_now = (BELT_X, 0.30 - 0.1 * now) if not lifted else None
        s4 = None
        if frame_t >= 0.5 and not lifted:
            true_y = 0.30 - 0.1 * frame_t
            s4 = _s4(frame_t, true_y + base_cam_offset[1], x=BELT_X + base_cam_offset[0],
                     plane=0.15)
        elif lifted:
            s4 = _s4(now, 0.0, has=False)
        orientation = (core._last_cmd.orientation if core._last_cmd is not None
                       else params.observe_pose().orientation)
        pose = Pose(*flange, *orientation)
        core.history.add(now, pose)
        m = None
        if not lifted and step % 3 == 0 and abs(flange[2] - params.observe_z) < 0.02:
            m = _s2(now, pose, BELT_X, 0.30 - 0.1 * now, 0.1, params)
        out = core.step(pose, s4, now, m, gripper.feedback(now))
        gripper.command(out.gripper_close, now, flange, block_now)
        if core.holding:
            lifted = True
        if out.target is not None:
            vel = [5.0 * (t - f) for t, f in zip(out.target.position, flange)]
            norm = math.sqrt(sum(c * c for c in vel))
            scale = min(1.0, 0.25 / norm) if norm > 0 else 1.0
            flange = [f + c * scale * 0.01 for f, c in zip(flange, vel)]
        if not states or states[-1] != out.state:
            states.append(out.state)
        min_z = min(min_z, flange[2])
        ramp_max = max(ramp_max, out.status.w_effective)
    return core, states, min_z, ramp_max


def test_full_cycle_with_the_base_camera_alone():
    """4d with w = 0 (Z10): grasp, lift, place, release -- and the withdrawn
    target after the lift does not stop the cycle (Z12)."""
    core, states, min_z, _ = _run_cycle(PARAMS)
    assert states[:9] == [STATE_WAIT, 1.0, STATE_FOLLOW, STATE_DESCEND, STATE_GRASP,
                          STATE_LIFT, STATE_PLACE, STATE_RELEASE, STATE_WAIT], states
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED))
    assert min_z >= PARAMS.grip_flange_z(0.1) - 0.002


def test_a_base_camera_offset_misses_without_and_grips_with_the_robot_camera():
    """4c: the base camera is 12 mm off across the belt (C3-like). Alone, the
    gripper closes 12 mm beside the block and misses. With the robot camera
    blended in across the belt, the correction pulls it onto the block."""
    offset = (0.012, 0.0)
    core, _, _, _ = _run_cycle(PARAMS, base_cam_offset=offset)
    assert core.picked[2] == float(OUTCOME_MISSED_GRIP)
    blended = replace(PARAMS, weight_along=1.0, weight_across=1.0)
    core, _, _, ramp_max = _run_cycle(blended, base_cam_offset=offset)
    assert core.picked[2] == float(OUTCOME_PLACED)
    assert ramp_max == 1.0
