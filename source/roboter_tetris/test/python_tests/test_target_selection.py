"""Tests for the target selection behind `priority_handler` (project goal 4).

No ROS, no numpy -- this runs with plain python3. The end-to-end test at the
bottom drives the synthetic belt from ``test/tools/fake_objects.py`` through the
real velocity estimation into the selection.
"""

import importlib.util
import math
import os
from dataclasses import replace

from roboter_tetris.contracts import (
    OUTCOME_PLACED, OUTCOME_TOO_LATE, TRACK_FINAL, TRACK_SETTLING, PickedId,
    TrackEntry, TracksMsg, unpack_not_pickable, unpack_objects, unpack_target,
)
from roboter_tetris.target_selection import (
    SelectorParams, TargetSelector, approach_point, belt_frame, fits_gripper,
    t_available, t_needed,
)
from roboter_tetris.track_estimation import TrackEstimator

_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "tools", "fake_objects.py")
_spec = importlib.util.spec_from_file_location("fake_objects", _TOOL)
fake_objects = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fake_objects)

V_BELT = (0.0, -0.100)          # along -y
FLANGE = (0.80, -0.60)          # waiting above the zone
# A fixed test zone, independent of the placeholder defaults:
# y -1.0 ... -0.5  ->  s = -y from 0.5 to 1.0.
# Grasp plane: 1.0 - 0.1 * (1.0 + 1.0 + 0.5) * 1.2 = 0.70, i.e. y = -0.70.
P = SelectorParams(zone_x_min=0.6, zone_x_max=1.0, zone_y_min=-1.0, zone_y_max=-0.5,
                   t_descend_s=1.0)


def _track(tid, y, x=0.80, status=TRACK_FINAL, length=0.05, width=0.05,
           height=0.10, vx=0.0, vy=-0.1, ori_quality=0.9):
    return TrackEntry(id=float(tid), color=0.0, x=x, y=y, z=0.1036,
                      orientation=0.3, length=length, width=width, height=height,
                      status=float(status), vx=vx, vy=vy, v_change=0.001,
                      ori_quality=ori_quality)


def _msg(tracks, t=10.0, v_belt=V_BELT, n_pool=1):
    return TracksMsg(t=t, v_belt_x=v_belt[0], v_belt_y=v_belt[1],
                     n_pool=float(n_pool), tracks=list(tracks))


def _target(selection):
    return unpack_target(selection.target)


# -- The belt frame -----------------------------------------------------------------

def test_frame_along_minus_y_gives_the_zone_edges_and_the_plane():
    frame = belt_frame(P, _msg([]))
    assert abs(frame.zone_upstream - 0.5) < 1e-12
    assert abs(frame.zone_end - 1.0) < 1e-12
    assert abs(frame.grasp_plane - 0.70) < 1e-12      # 0.30 m before the end (Z11)
    assert frame.feasible


def test_no_frame_without_a_belt_estimate_or_on_a_standing_belt():
    assert belt_frame(P, _msg([], n_pool=0)) is None
    assert belt_frame(P, _msg([], v_belt=(0.0, -0.005))) is None


def test_plane_moves_upstream_on_a_faster_belt():
    slow = belt_frame(P, _msg([], v_belt=(0.0, -0.10)))
    fast = belt_frame(P, _msg([], v_belt=(0.0, -0.15)))
    assert fast.grasp_plane < slow.grasp_plane


def test_slanted_belt_keeps_the_bounds_inside_the_zone():
    """The whole upstream edge lies at or before zone_upstream, the whole
    downstream edge at or after zone_end -- so [upstream, end] is inside."""
    p = P
    frame = belt_frame(p, _msg([], v_belt=(0.02, -0.098)))
    for x in (p.zone_x_min, p.zone_x_max):
        assert frame.s(x, p.zone_y_max) <= frame.zone_upstream + 1e-12
        assert frame.s(x, p.zone_y_min) >= frame.zone_end - 1e-12
    assert frame.zone_upstream < frame.zone_end


def test_too_short_zone_is_not_feasible_and_nothing_is_chosen():
    params = replace(P, zone_y_min=-0.75)         # 0.25 m < 0.30 m
    selector = TargetSelector(params)
    sel = selector.step(_msg([_track(1, y=-0.40)]), FLANGE)
    assert not sel.frame.feasible
    assert selector.locked_id is None


# -- Reachability -------------------------------------------------------------------

def test_reachability_at_the_boundary():
    """Kandidat iff t_available > factor * t_needed -- checked from both sides."""
    params = P
    frame = belt_frame(params, _msg([]))
    flange = (0.80, -0.60)
    # Block on the flange's lane inside the zone: approach distance = |dy|.
    # Solve t_available = 1.5 * t_needed for the block position y:
    #   (0.70 + y) / 0.1 = 1.5 * ((-0.60 - y) / 0.25 + 0.6)
    y_edge = (1.5 * (-0.60 / 0.25 + 0.6) - 7.0) / (10.0 + 1.5 / 0.25)
    for y, expected in ((y_edge + 0.002, True), (y_edge - 0.002, False)):
        track = _track(1, y=y)
        assert (t_available(frame, track)
                > 1.5 * t_needed(params, frame, track, flange)) == expected
        selector = TargetSelector(params)
        selector.step(_msg([track]), flange)
        assert (selector.locked_id == 1.0) == expected


def test_distance_is_measured_to_where_anfahren_waits():
    """Upstream of the zone the robot waits at zone_upstream (Nachtrag 7 / H1)."""
    params = P
    frame = belt_frame(params, _msg([]))
    far_upstream = _track(1, y=0.20)                  # 0.7 m before the zone
    ax, ay = approach_point(frame, far_upstream)
    assert abs(ax - 0.80) < 1e-12 and abs(ay - (-0.5)) < 1e-12
    assert abs(t_needed(params, frame, far_upstream, FLANGE)
               - (0.10 / 0.25 + 0.6)) < 1e-12


def test_a_block_upstream_of_the_zone_can_be_chosen():
    """E10 / Z11: otherwise the clamp to zone_upstream would never act."""
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.20)]), FLANGE)
    assert selector.locked_id == 1.0


def test_block_past_the_plane_is_never_chosen():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=-0.75)]), FLANGE)
    assert selector.locked_id is None


def test_nothing_is_chosen_without_the_robot_position():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.0)]), None)
    assert selector.locked_id is None


# -- Graspability and lane ------------------------------------------------------------

def test_too_flat_block_is_never_chosen():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.0, height=0.029)]), FLANGE)
    assert selector.locked_id is None
    selector.step(_msg([_track(2, y=0.0, height=0.030)]), FLANGE)
    assert selector.locked_id == 2.0


def test_the_diagonal_must_fit_the_gripper():
    """Nachtrag 7 / H2: 127 mm - 10 mm = 117 mm against the diagonal."""
    params = P
    assert fits_gripper(params, _track(1, 0.0, length=0.100, width=0.050))   # 112
    assert not fits_gripper(params, _track(1, 0.0, length=0.100, width=0.070))  # 122
    selector = TargetSelector(params)
    selector.step(_msg([_track(1, y=0.0, length=0.100, width=0.070)]), FLANGE)
    assert selector.locked_id is None


def test_ungraspable_block_is_logged_once():
    selector = TargetSelector(P)
    for _ in range(5):
        selector.step(_msg([_track(1, y=0.0, height=0.025)]), FLANGE)
    lines = selector.pop_events()
    assert len(lines) == 1 and "Höhe 25 mm" in lines[0]


def test_block_whose_lane_misses_the_zone_is_never_chosen():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.0, x=1.05)]), FLANGE)
    assert selector.locked_id is None


def test_settling_block_is_not_a_candidate():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.0, status=TRACK_SETTLING)]), FLANGE)
    assert selector.locked_id is None


# -- Choice and lock ----------------------------------------------------------------

def test_the_most_urgent_candidate_is_chosen():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.10), _track(2, y=-0.20),
                        _track(3, y=0.30)]), FLANGE)
    assert selector.locked_id == 2.0


def test_lock_holds_when_a_more_urgent_block_appears():
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=0.20)]), FLANGE)
    selector.step(_msg([_track(1, y=0.19), _track(2, y=-0.30)]), FLANGE)
    assert selector.locked_id == 1.0


def test_lock_holds_past_the_plane_and_the_zone_end():
    """Reachability is no reason to withdraw (P4); the follower decides at the
    plane whether the grasp may still start."""
    selector = TargetSelector(P)
    selector.step(_msg([_track(1, y=-0.30)]), FLANGE)
    for y in (-0.72, -1.05):
        sel = selector.step(_msg([_track(1, y=y)]), None)   # robot state gone too
        assert selector.locked_id == 1.0
        assert _target(sel).has_target == 1.0
    assert _target(sel).t_rest < 0.0


# -- Withdrawal -----------------------------------------------------------------------

def _locked_on(tid=1, y=-0.30):
    selector = TargetSelector(P)
    selector.step(_msg([_track(tid, y=y)]), FLANGE)
    assert selector.locked_id == float(tid)
    return selector


def test_picked_id_releases_for_every_outcome_and_excludes_the_block():
    for outcome in (OUTCOME_PLACED, 1, 2, OUTCOME_TOO_LATE):
        selector = _locked_on()
        selector.on_picked(PickedId(seq=1.0, id=1.0, outcome=float(outcome)))
        assert selector.locked_id is None
        selector.step(_msg([_track(1, y=-0.30)]), FLANGE)
        assert selector.locked_id is None, "ein erledigter Klotz wurde neu gewählt"


def test_the_same_seq_is_handled_once_and_seq_zero_not_at_all():
    selector = TargetSelector(P)
    selector.on_picked(PickedId(seq=0.0, id=1.0, outcome=0.0))   # nothing yet
    selector.step(_msg([_track(1, y=-0.30)]), FLANGE)
    assert selector.locked_id == 1.0
    selector.on_picked(PickedId(seq=1.0, id=7.0, outcome=0.0))   # another block
    assert selector.locked_id == 1.0
    selector.on_picked(PickedId(seq=2.0, id=1.0, outcome=0.0))
    assert selector.locked_id is None
    # The follower was reactivated (seq back to 0), then its first attempt:
    selector.step(_msg([_track(2, y=-0.30)]), FLANGE)
    selector.on_picked(PickedId(seq=0.0, id=2.0, outcome=0.0))
    assert selector.locked_id == 2.0
    selector.on_picked(PickedId(seq=1.0, id=2.0, outcome=0.0))
    assert selector.locked_id is None


def test_lost_id_releases_the_target():
    selector = _locked_on()
    selector.step(_msg([]), FLANGE)
    assert selector.locked_id is None
    assert "verloren" in selector.pop_events()[-1]


def test_settling_again_releases_and_the_block_may_come_back():
    selector = _locked_on()
    selector.step(_msg([_track(1, y=-0.31, status=TRACK_SETTLING)]), FLANGE)
    assert selector.locked_id is None
    selector.step(_msg([_track(1, y=-0.33)]), FLANGE)
    assert selector.locked_id == 1.0


def test_explicit_withdraw_for_stale_input():
    selector = _locked_on()
    selector.withdraw("Eingang steht")
    sel = selector.output(_msg([_track(1, y=-0.30)]))
    assert _target(sel).has_target == 0.0


# -- Outputs --------------------------------------------------------------------------

def test_s4_carries_the_pool_velocity_not_the_tracks_own():
    selector = TargetSelector(P)
    track = _track(1, y=-0.30, vx=0.004, vy=-0.093, ori_quality=0.42)
    target = _target(selector.step(_msg([track]), FLANGE))
    assert target.has_target == 1.0 and target.id == 1.0
    assert (target.vx, target.vy) == V_BELT
    assert target.ori_quality == 0.42
    assert abs(target.t_rest - (0.70 - 0.30) / 0.1) < 1e-9
    assert target.t == 10.0


def test_zone_fields_stay_valid_without_a_target():
    sel = TargetSelector(P).step(_msg([]), FLANGE)
    target = _target(sel)
    assert target.has_target == 0.0
    assert abs(target.zone_upstream - 0.5) < 1e-12
    assert abs(target.grasp_plane - 0.7) < 1e-12
    assert (target.vx, target.vy) == V_BELT          # needed to read 12 and 15


def test_without_an_estimate_the_zone_fields_are_zero():
    sel = TargetSelector(P).step(_msg([_track(1, y=-0.3)], n_pool=0), FLANGE)
    target = _target(sel)
    assert target.has_target == 0.0
    assert (target.zone_upstream, target.vx, target.vy, target.grasp_plane) == \
        (0.0, 0.0, 0.0, 0.0)
    assert unpack_not_pickable(sel.not_pickable).ids == []


def test_not_pickable_lists_tracks_past_the_plane_except_the_target():
    selector = _locked_on(tid=1)
    sel = selector.step(_msg([_track(1, y=-0.75), _track(2, y=-0.80),
                              _track(3, y=-0.60),
                              _track(4, y=-0.90, status=TRACK_SETTLING)]), FLANGE)
    assert unpack_not_pickable(sel.not_pickable).ids == [2.0, 4.0]


def test_zone_empty_is_about_space_not_status():
    selector = TargetSelector(P)
    assert selector.step(_msg([_track(1, y=0.10)]), None).zone_empty
    assert not selector.step(
        _msg([_track(1, y=-0.60, status=TRACK_SETTLING)]), None).zone_empty


# -- End to end: synthetic belt -> vectoring -> selection -----------------------------

def test_synthetic_run_picks_1_and_2_and_never_the_small_cube():
    """Blocks 1 (50x50x100) and 2 (76x50x50) are graspable, 3 (25 mm cube) is
    not. A stand-in follower 'grasps' each target 2 s after it was chosen."""
    belt = fake_objects.FakeBelt(fake_objects.default_blocks(),
                                 noise_sigma_m=0.0005, seed=3)
    estimator = TrackEstimator()
    selector = TargetSelector()                 # the zone defaults of 23.09.2026
    # Waiting above the upstream end of the zone (y +0.40): from the old wait
    # position at the zone end the approach is too long and nothing is chosen.
    flange = (fake_objects.BELT_CENTER_X_M, 0.30)
    chosen, gripped, seq = [], set(), 0
    locked_since = None
    fps = 30.0
    for k in range(int(19.0 * fps)):
        t = k / fps
        objects = [o for o in unpack_objects(belt.signal_at(t)).objects
                   if o.id not in gripped]
        estimator.update(t, objects)
        v_belt, n_pool, entries = estimator.snapshot(t)
        msg = _msg(entries, t=t, v_belt=v_belt or (0.0, 0.0), n_pool=n_pool)
        sel = selector.step(msg, flange)
        if selector.locked_id is not None and selector.locked_id not in chosen:
            chosen.append(selector.locked_id)
            locked_since = t
        if selector.locked_id is not None and t - locked_since >= 2.0:
            seq += 1
            gripped.add(selector.locked_id)
            selector.on_picked(PickedId(float(seq), selector.locked_id, 0.0))
    assert chosen == [1.0, 2.0]
    # The cube crossed the grasp plane (y = -0.03) at about t = 14.3 s.
    assert 3.0 in unpack_not_pickable(sel.not_pickable).ids
    target = unpack_target(sel.target)
    assert abs(target.vy - fake_objects.DEFAULT_V_BELT_MPS) < 0.001
    d = SelectorParams()
    zone_end = -d.zone_y_min                                 # s = -y
    process = d.t_descend_s + d.t_grasp_s + d.t_lift_s
    assert math.isclose(target.grasp_plane,
                        zone_end - 0.1 * process * d.grasp_time_margin, abs_tol=0.001)
