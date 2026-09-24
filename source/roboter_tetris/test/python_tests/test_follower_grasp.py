"""Tests for the follower, stage 4d (grasp cycle).

No ROS, no numpy -- runs with plain python3. Robot frame throughout (Nachtrag 8
/ F1): belt at x = -0.816, running along -y.
"""

import math

from roboter_tetris.contracts import (
    OUTCOME_LOST, OUTCOME_MISSED_GRIP, OUTCOME_PLACED, OUTCOME_TOO_LATE,
    STATE_ABORT, STATE_DESCEND, STATE_FOLLOW, STATE_GRASP, STATE_LIFT,
    STATE_PLACE, STATE_RELEASE, STATE_WAIT, TrackEntry, pack_target,
    unpack_target,
)
from roboter_tetris.follower_logic import (
    FollowerCore, FollowerParams, GripperFeedback, Pose, vertical_orientation,
)

PARAMS = FollowerParams(
    ws_x_min=-1.10, ws_x_max=-0.20, ws_y_min=-0.60, ws_y_max=0.60,
    ws_z_min=0.30, ws_z_max=0.80,
    observe_x=-0.80, observe_y=-0.10, observe_z=0.60, observe_yaw_deg=90.0,
    # Mode 1 pinned: these tests predate the mode-2 default (L26); mode 2 has its own.
    use_block_orientation=False)
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


# == 4d: grasp height and the grasp cycle, step by step ============================================

def test_grip_height_is_mid_block_but_never_too_close_to_the_belt():
    assert abs(PARAMS.grip_flange_z(0.100) - (0.0536 + 0.050 + 0.235)) < 1e-12
    # Floor 16 mm (pad centre): the closed jaw tip stays 6 mm above the belt (L26).
    assert abs(PARAMS.grip_flange_z(0.020) - (0.0536 + 0.016 + 0.235)) < 1e-12


def _following_core(params=PARAMS, y=-0.02, plane=0.9, height=0.1):
    """A core in FOLGEN whose flange sits exactly on the predicted block."""
    core = FollowerCore(params)
    at = Pose(BELT_X, y - 0.1 * 0.0, params.observe_z, *vertical_orientation(-math.pi / 2))
    core.step(at, None, 0.0, OPEN)
    core.step(at, _s4(0.0, y, plane=plane, height=height), 0.0, OPEN)
    assert core.state == STATE_FOLLOW
    return core


def _on_block(core, t, y, z=None, plane=0.9, height=0.1, gripper=OPEN, has=True):
    """One cycle with the flange exactly on the block predicted for t."""
    z = core.params.observe_z if z is None else z
    flange = Pose(BELT_X, y, z, *vertical_orientation(-math.pi / 2))
    return core.step(flange, _s4(t, y, plane=plane, height=height, has=has), t, gripper)


def test_descend_starts_after_stable_cycles_within_tolerance():
    core = _following_core()                   # 1st stable cycle
    for k in range(1, 9):
        _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert core.state == STATE_FOLLOW          # 9 cycles
    out = _on_block(core, 0.09, -0.029)
    assert out.state == STATE_DESCEND          # the 10th
    for k in range(10, 20):
        out = _on_block(core, k * 0.01, -0.02 - 0.001 * k)
    assert abs(out.target.z - (0.60 - 0.25 * 0.10)) < 1e-9     # 0.25 m/s for 0.1 s (L24)


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
    out = core.step(flange, _s4(0.12, -0.032), 0.12, OPEN)
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
    out = core.step(flange, _s4(0.71, -0.091, has=False), 0.71,
                    GripperFeedback(False, False))
    assert out.target.z == grip
    core.step(flange, None, 0.90, GripperFeedback(True, False))
    out = core.step(flange, None, 0.91, GripperFeedback(True, False))
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
    first = core.step(flange, _s4(0.8, 0.0, has=False), 0.80,
                      GripperFeedback(True, True))
    out = core.step(flange, _s4(0.9, 0.0, has=False), 0.90,
                    GripperFeedback(True, True))
    assert out.state == STATE_LIFT and core.picked[0] == 0.0
    assert abs((out.target.y - first.target.y) - (-0.1 * 0.1)) < 1e-9   # with the belt


def test_lift_goes_up_then_to_the_place_pose_and_releases():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    yaw = vertical_orientation(-math.pi / 2)
    core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.9, held)
    out = core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.91, held)
    assert out.state == STATE_LIFT and out.target.z == PARAMS.transfer_height_m
    out = core.step(Pose(BELT_X, -0.10, PARAMS.transfer_height_m, *yaw), None, 1.5, held)
    assert out.state == STATE_PLACE and out.target == PARAMS.place_pose()
    at_place = PARAMS.place_pose()
    out = core.step(at_place, None, 4.0, held)
    assert out.state == STATE_RELEASE and out.gripper_close is False
    core.step(at_place, None, 4.05, GripperFeedback(False, True))   # opening
    out = core.step(at_place, None, 4.3, GripperFeedback(True, False))
    assert out.state == STATE_WAIT
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED))
    assert not core.has_aborted and not core.holding


def test_a_short_has_object_flicker_is_not_a_lost_block():
    core, grip = _lifting_core()
    flange = Pose(BELT_X, -0.10, grip + 0.02, *vertical_orientation(-math.pi / 2))
    for t in (0.80, 0.81, 0.82, 0.83, 0.84):          # 40 ms of flicker, 5 cycles
        core.step(flange, None, t, GripperFeedback(True, False))
    core.step(flange, None, 0.85, GripperFeedback(True, True))
    assert core.state == STATE_LIFT
    core.step(flange, None, 0.90, GripperFeedback(True, False))
    core.step(flange, None, 1.01, GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_LOST))


def test_abort_with_a_block_in_the_gripper_still_places_it():
    """Z12: gripper stays closed, straight up, place pose, open there, outcome 0."""
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    flange = Pose(BELT_X, -0.10, grip + 0.02, *vertical_orientation(-math.pi / 2))
    core.abort("Test: künstlicher Abbruch", flange, 0.8)
    out = core.step(flange, None, 0.81, held)
    assert out.state == STATE_ABORT and out.gripper_close
    assert out.target.z == PARAMS.transfer_height_m and out.target.x == BELT_X
    up = flange._replace(z=PARAMS.transfer_height_m)
    out = core.step(up, None, 1.5, held)
    assert out.state == STATE_PLACE and out.gripper_close
    at_place = PARAMS.place_pose()
    core.step(at_place, None, 4.0, held)
    core.step(at_place, None, 4.05, GripperFeedback(False, True))
    core.step(at_place, None, 4.3, GripperFeedback(True, False))
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED)) and core.has_aborted


def test_unreachable_place_pose_keeps_the_block_instead_of_dropping_it():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    yaw = vertical_orientation(-math.pi / 2)
    stuck = Pose(BELT_X, -0.10, PARAMS.transfer_height_m, *yaw)
    core.step(Pose(BELT_X, -0.10, grip + 0.10, *yaw), None, 0.9, held)
    core.step(stuck, None, 1.0, held)
    assert core.state == STATE_PLACE
    core.step(stuck, None, 9.5, held)       # 1st timeout: abort with the
    assert core.state == STATE_PLACE and core.holding   # block, already up: retry
    assert any("Zeitüberschreitung in ABLEGEN" in e for e in core.pop_events())
    for t in (18.0, 25.0, 40.0):                               # 2nd: hold on
        out = core.step(stuck, None, t, held)
        assert out.state == STATE_PLACE and out.gripper_close
    assert any("Eingriff" in e for e in core.pop_events())


def test_release_without_motion_confirmation_still_finishes():
    core, grip = _lifting_core()
    held = GripperFeedback(True, True)
    at_place = PARAMS.place_pose()
    core._enter(STATE_PLACE, "Test", 5.0)
    core.step(at_place, None, 5.0, held)
    out = core.step(at_place, None, 7.2, GripperFeedback(True, True))
    assert out.state == STATE_WAIT and core.picked[2] == float(OUTCOME_PLACED)


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
    at 30 Hz with 50 ms latency, first-order attractor K = 5 capped at
    0.25 m/s, gripper stand-in. After has_object the block rides in the
    gripper and the target is withdrawn -- as on the real belt."""
    core = FollowerCore(params)
    flange = list(params.observe_pose().position)
    gripper = _Gripper()
    states, min_z = [], 1.0
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
        out = core.step(pose, s4, now, gripper.feedback(now))
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
    return core, states, min_z


def test_full_cycle_with_the_base_camera_alone():
    """4d with w = 0 (Z10): grasp, lift, place, release -- and the withdrawn
    target after the lift does not stop the cycle (Z12)."""
    core, states, min_z = _run_cycle(PARAMS)
    assert states[:9] == [STATE_WAIT, 1.0, STATE_FOLLOW, STATE_DESCEND, STATE_GRASP,
                          STATE_LIFT, STATE_PLACE, STATE_RELEASE, STATE_WAIT], states
    assert core.picked == (1.0, 1.0, float(OUTCOME_PLACED))
    assert min_z >= PARAMS.grip_flange_z(0.1) - 0.002


def test_a_base_camera_offset_across_the_belt_is_a_missed_grip():
    """A limit of the system: the base camera 12 mm off across the belt (C3-like)
    and the gripper closes 12 mm beside the block -- a missed grip, handled
    cleanly (outcome 1). The robot camera correction that caught this went with
    stage 4c (Nachtrag 13 / L22)."""
    core, _, _ = _run_cycle(PARAMS, base_cam_offset=(0.012, 0.0))
    assert core.picked[2] == float(OUTCOME_MISSED_GRIP)
