"""Tests for the follower logic, stage 4a: start, observation pose, safety gate,
flange history. No ROS, no numpy -- runs with plain python3.

Robot-frame numbers from the setup: belt surface touched at x = -0.70 ... -0.93,
y = -0.20 ... +0.07 (Nachtrag 5 / M9); place pose x = -0.316, y = +0.476.
"""

import math
from dataclasses import replace

from roboter_tetris.contracts import (
    OUTCOME_ABORTED, STATE_ABORT, STATE_APPROACH, STATE_FOLLOW, STATE_WAIT,
    TrackEntry, pack_target, unpack_target,
)
from roboter_tetris.follower_logic import (
    FollowerCore, FollowerParams, Pose, PoseHistory, SafetyGate, Workspace,
    desired_yaw, nearest_equivalent, tracking_point, vertical_orientation,
    yaw_of,
)

# Provisional values for the tests only -- B10 and B8 are open.
PARAMS = FollowerParams(
    ws_x_min=-1.10, ws_x_max=-0.20, ws_y_min=-0.60, ws_y_max=0.60,
    ws_z_min=0.30, ws_z_max=0.80,
    observe_x=-0.80, observe_y=-0.10, observe_z=0.60, observe_yaw_deg=90.0)
DOWN = vertical_orientation(0.0)


def _rotate(q, v):
    """Rotate vector v by the unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q_vec, v); v' = v + w t + cross(q_vec, t)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _close(a, b, tol=1e-9):
    return all(abs(p - q) < tol for p, q in zip(a, b))


# -- Orientation ------------------------------------------------------------------------

def test_vertical_orientation_points_the_tool_down_with_the_given_yaw():
    for yaw_deg in (0.0, 37.0, 90.0, -120.0):
        q = vertical_orientation(math.radians(yaw_deg))
        assert _close(_rotate(q, (0, 0, 1)), (0, 0, -1))
        yaw = math.radians(yaw_deg)
        assert _close(_rotate(q, (1, 0, 0)), (math.cos(yaw), math.sin(yaw), 0))


def test_measured_place_orientation_has_the_same_form():
    """B9: (w, x, y, z) = 0.006857, 0.680692, 0.732524, -0.004575 -- 0.94 deg
    off vertical. The formula reproduces it within that."""
    measured = (0.006857, 0.680692, 0.732524, -0.004575)
    yaw = 2 * math.atan2(measured[2], measured[1])
    assert _close(vertical_orientation(yaw), measured, tol=0.01)


# -- Parameters -------------------------------------------------------------------------

def test_missing_parameters_are_named():
    problems = FollowerParams(observe_x=-0.8).problems()
    assert len(problems) == 1 and "ws_x_min" in problems[0] \
        and "observe_yaw_deg" in problems[0] and "observe_x," not in problems[0]


def test_a_complete_consistent_set_has_no_problems():
    assert PARAMS.problems() == []


def test_inconsistent_sets_are_rejected():
    assert "Arbeitsraum leer in y" in replace(PARAMS, ws_y_min=0.7).problems()
    assert any("Beobachtungspose" in p
               for p in replace(PARAMS, observe_z=0.9).problems())
    assert any("transfer_height_m" in p
               for p in replace(PARAMS, transfer_height_m=0.25).problems())
    assert any("nicht endlich" in p
               for p in replace(PARAMS, observe_x=float("nan")).problems())


# -- Workspace and safety gate --------------------------------------------------------------

def test_workspace_clamps_and_says_so():
    ws = PARAMS.workspace()
    inside = Pose(-0.8, 0.0, 0.5, *DOWN)
    assert ws.clamp(inside) == (inside, False)
    clamped, flag = ws.clamp(Pose(-1.5, 0.0, 0.1, *DOWN))
    assert flag and (clamped.x, clamped.z) == (-1.10, 0.30)


def test_gate_rejects_non_finite_targets():
    result = SafetyGate(PARAMS).check(Pose(float("nan"), 0, 0.5, *DOWN))
    assert result.pose is None and "nicht endlich" in result.abort_reason


def test_gate_aborts_on_a_jump_within_a_state_but_not_after_reset():
    gate = SafetyGate(PARAMS)
    assert gate.check(Pose(-0.8, 0.0, 0.5, *DOWN)).pose is not None
    assert gate.check(Pose(-0.8, 0.04, 0.5, *DOWN)).pose is not None
    jumped = gate.check(Pose(-0.8, 0.20, 0.5, *DOWN))
    assert jumped.pose is None and "springt" in jumped.abort_reason
    gate.reset()
    assert gate.check(Pose(-0.8, 0.20, 0.5, *DOWN)).pose is not None


def test_gate_clamps_to_the_workspace():
    result = SafetyGate(PARAMS).check(Pose(-0.8, 0.0, 0.1, *DOWN))
    assert result.clamped and result.pose.z == 0.30 and result.abort_reason is None


# -- Flange history -------------------------------------------------------------------------

def test_history_interpolates_position_between_samples():
    history = PoseHistory()
    history.add(1.00, Pose(0.0, 0.0, 0.5, *DOWN))
    history.add(1.01, Pose(0.002, 0.0, 0.5, *vertical_orientation(1.0)))
    pose = history.at(1.0075)
    assert abs(pose.x - 0.0015) < 1e-12
    assert pose.orientation == vertical_orientation(1.0)    # nearer sample
    assert history.at(0.99) is None and history.at(1.02) is None


def test_history_drops_old_and_out_of_order_samples():
    history = PoseHistory(max_age_s=1.0)
    for k in range(300):
        history.add(k * 0.01, Pose(k, 0, 0, *DOWN))
    history.add(1.0, Pose(-1, 0, 0, *DOWN))            # older than the newest
    assert 100 <= len(history) <= 101                   # about one second
    assert history.at(2.99).x == 299


# -- State machine ----------------------------------------------------------------------------

def test_without_a_flange_pose_nothing_is_published():
    core = FollowerCore(PARAMS)
    out = core.step(None)
    assert out.target is None and out.state == STATE_ABORT
    assert core.step(Pose(float("nan"), 0, 0.4, *DOWN)).target is None


def test_start_rises_straight_up_keeping_the_orientation():
    tilted = (0.9, 0.1, 0.3, 0.3)
    start = Pose(-0.75, 0.05, 0.35, *tilted)
    out = FollowerCore(PARAMS).step(start)
    assert out.state == STATE_ABORT
    assert out.target == Pose(-0.75, 0.05, PARAMS.transfer_height_m, *tilted)
    assert out.gripper_close is False


def test_after_rising_the_follower_waits_at_the_observation_pose():
    core = FollowerCore(PARAMS)
    core.step(Pose(-0.75, 0.05, 0.35, *DOWN))
    out = core.step(Pose(-0.75, 0.05, PARAMS.transfer_height_m - 0.005, *DOWN))
    assert out.state == STATE_WAIT
    assert out.target == PARAMS.observe_pose()
    assert core.pop_events()[-1] == "ABBRUCH -> WARTEN: angehoben"


def test_start_above_the_free_height_does_not_descend_first():
    core = FollowerCore(PARAMS)
    out = core.step(Pose(-0.75, 0.05, 0.70, *DOWN))
    assert out.state == STATE_WAIT          # already high enough


def test_start_outside_the_workspace_is_clamped_and_reported_once():
    core = FollowerCore(PARAMS)
    for _ in range(3):
        out = core.step(Pose(-1.30, 0.05, 0.35, *DOWN))
    assert out.target.x == -1.10
    assert sum("gedeckelt" in e for e in core.pop_events()) == 1


def test_a_jump_of_the_observation_pose_takes_the_abort_path():
    """Moving the observation pose by more than max_target_jump_m during
    WARTEN is treated like wrong data: rise first, then go there."""
    core = FollowerCore(PARAMS)
    flange = Pose(-0.80, -0.10, 0.60, *vertical_orientation(math.radians(90)))
    assert core.step(flange).state == STATE_WAIT
    core.params = replace(PARAMS, observe_y=0.20)
    out = core.step(flange)
    assert out.target is None and out.state == STATE_ABORT
    out = core.step(flange)                  # already at the free height
    assert out.state == STATE_WAIT and out.target.y == 0.20


# -- Closed loop: the path the robot actually takes ----------------------------------------

def test_simulated_start_goes_up_first_and_never_down_through_the_belt():
    """A first-order attractor, K = 5, 100 Hz. Start low over the belt: the
    flange must reach the free height before moving sideways."""
    core = FollowerCore(PARAMS)
    flange = [-0.72, 0.02, 0.34]                # just above a standing block
    k, dt = 5.0, 0.01
    path = []
    for _ in range(800):
        out = core.step(Pose(*flange, *DOWN))
        if out.target is not None:
            for i, axis in enumerate(("x", "y", "z")):
                flange[i] += k * (getattr(out.target, axis) - flange[i]) * dt
        path.append((tuple(flange), out.state))
    lateral_during_rise = max(math.hypot(p[0] + 0.72, p[1] - 0.02)
                              for p, state in path if state == STATE_ABORT)
    assert lateral_during_rise < 1e-9
    assert min(p[2] for p, _ in path) >= 0.34 - 1e-9
    assert path[-1][1] == STATE_WAIT
    assert math.dist(path[-1][0], PARAMS.observe_pose().position) < 0.001


# == Stage 4b: ANFAHREN and FOLGEN ============================================================

BELT_X = -0.816                 # robot frame, M9
V = (0.0, -0.100)               # along -y: s = -y
ZONE_UPSTREAM = -0.05           # y = +0.05
# 4b on its own: no descending (stable_cycles never reached), long FOLGEN.
TRACK = replace(PARAMS, timeout_track_s=30.0, stable_cycles=10**9)
AT_OBSERVE = Pose(-0.80, -0.10, 0.60, *vertical_orientation(math.radians(90)))


def _s4(t, y, x=BELT_X, tid=1, has=True, v=V, orientation=0.3, quality=0.9,
        plane=0.9, height=0.1):
    track = TrackEntry(id=float(tid), color=0.0, x=x, y=y, z=0.1036,
                       orientation=orientation, length=0.05, width=0.05,
                       height=height, status=0.0, vx=0.0, vy=-0.1, v_change=0.0,
                       ori_quality=quality)
    # plane 0.9 (y = -0.9): out of the way for the 4b tests; 4d sets its own.
    return unpack_target(pack_target(t, has, ZONE_UPSTREAM, plane, v,
                                      track if has else None))


def _tracking_core(params=TRACK, y=0.30, now=0.0):
    """A core that has just taken target 1 at y. Upstream of the zone it is in
    ANFAHREN; already inside, it moves on to FOLGEN in the same cycle."""
    core = FollowerCore(params)
    core.step(AT_OBSERVE, None, now)               # rise done -> WARTEN
    out = core.step(AT_OBSERVE, _s4(now, y), now)
    assert out.state in (STATE_APPROACH, STATE_FOLLOW)
    return core


# -- Prediction, lead, clamp ------------------------------------------------------------------

def test_prediction_and_lead_as_time():
    point = tracking_point(_s4(1.0, -0.20), 1.1, TRACK, clamp_upstream=False)
    assert abs(point.block_y - (-0.20 - 0.1 * 0.1)) < 1e-12        # 0.1 s old
    assert abs(point.target_y - (point.block_y - 0.1 * 0.2)) < 1e-12  # lead 0.2 s
    assert point.block_x == BELT_X and not point.capped


def test_prediction_horizon_is_capped_both_ways():
    old = tracking_point(_s4(1.0, -0.20), 2.0, TRACK, clamp_upstream=False)
    cap = TRACK.max_extrapolation_s
    assert old.capped and abs(old.block_y - (-0.20 - 0.1 * cap)) < 1e-12
    future = tracking_point(_s4(1.0, -0.20), 0.9, TRACK, clamp_upstream=False)
    assert future.capped and future.block_y == -0.20


def test_standing_target_needs_no_special_case():
    point = tracking_point(_s4(1.0, -0.20, v=(0.0, 0.0)), 1.1, TRACK,
                           clamp_upstream=False)
    assert (point.target_x, point.target_y) == (BELT_X, -0.20)


def test_approach_clamps_only_upstream_and_keeps_the_lane():
    point = tracking_point(_s4(1.0, 0.40, x=-0.75), 1.0, TRACK, clamp_upstream=True)
    assert abs(point.target_y - 0.05) < 1e-12         # waits at zone_upstream ...
    assert point.target_x == -0.75                    # ... on the block's lane
    inside = tracking_point(_s4(1.0, -0.30), 1.0, TRACK, clamp_upstream=True)
    assert abs(inside.target_y - (-0.30 - 0.02)) < 1e-12   # no clamp downstream


# -- Yaw ----------------------------------------------------------------------------------------

def test_mode_1_follows_the_measured_belt_direction():
    assert abs(desired_yaw(_s4(1.0, 0.0), TRACK) - (-math.pi / 2)) < 1e-12
    offset = replace(TRACK, gripper_yaw_offset_deg=90.0)
    assert abs(desired_yaw(_s4(1.0, 0.0), offset)) < 1e-12


def test_mode_2_uses_the_block_angle_only_with_enough_quality():
    mode2 = replace(TRACK, use_block_orientation=True)
    assert desired_yaw(_s4(1.0, 0.0, orientation=0.3, quality=0.9), mode2) == 0.3
    assert abs(desired_yaw(_s4(1.0, 0.0, orientation=0.3, quality=0.5), mode2)
               - (-math.pi / 2)) < 1e-12


def test_the_nearer_of_the_two_equivalent_wrist_angles_is_chosen():
    assert abs(nearest_equivalent(-math.pi / 2, 1.5) - math.pi / 2) < 1e-12
    # The block angle wraps from just below pi to just above 0: no half turn.
    assert abs(nearest_equivalent(0.01, 3.13) - (0.01 + math.pi)) < 1e-12


def test_yaw_is_read_back_from_a_vertical_quaternion():
    for yaw in (-2.0, -0.4, 0.0, 1.2, 3.0):
        assert abs(yaw_of(vertical_orientation(yaw)) - yaw) < 1e-12


# -- States -------------------------------------------------------------------------------------

def test_waiting_follower_takes_a_target_and_waits_at_the_zone_edge():
    core = _tracking_core(y=0.30)
    out = core.step(AT_OBSERVE, _s4(0.0, 0.30), 0.0)
    assert out.state == STATE_APPROACH
    assert abs(out.target.y - 0.05) < 1e-12 and out.target.x == BELT_X
    assert out.target.z == TRACK.observe_z
    assert out.status.target_id == 1.0


def test_approach_turns_into_following_when_the_block_enters_the_zone():
    core = _tracking_core(y=0.30)
    assert core.step(AT_OBSERVE, _s4(1.0, 0.10), 1.0).state == STATE_APPROACH
    out = core.step(AT_OBSERVE, _s4(1.1, 0.04), 1.1)
    assert out.state == STATE_FOLLOW
    assert abs(out.target.y - (0.04 - 0.02)) < 1e-12        # no clamp any more


def test_each_abort_reason_ends_the_attempt_with_picked_id():
    cases = {
        "zurückgezogen": lambda: _s4(1.0, 0.2, has=False),
        "ID gewechselt": lambda: _s4(1.0, 0.2, tid=7),
        "nicht endlich": lambda: _s4(1.0, float("nan")),
        "ohne Bandgeschwindigkeit": lambda: _s4(1.0, 0.2, v=(0.0, 0.0)),
    }
    for label, make in cases.items():
        core = _tracking_core()
        out = core.step(AT_OBSERVE, make(), 1.0)
        assert out.state in (STATE_ABORT, STATE_WAIT), label
        assert core.picked == (1.0, 1.0, float(OUTCOME_ABORTED)), label
        assert core.has_aborted
        assert any(label in e for e in core.pop_events()), label


def test_standing_s4_timestamp_aborts():
    core = _tracking_core(y=0.30)
    limit = TRACK.target_timeout_s
    assert core.step(AT_OBSERVE, _s4(0.0, 0.30), limit - 0.1).state == STATE_APPROACH
    core.step(AT_OBSERVE, _s4(0.0, 0.30), limit + 0.1)
    assert core.picked[0] == 1.0
    assert any("steht still" in e for e in core.pop_events())


def test_timeouts_in_approach_and_follow():
    # Block 0.25 m before the zone -> expected wait 2.5 s, plus 2 s.
    core = _tracking_core(params=PARAMS, y=0.30)
    for k in range(1, 45):
        core.step(AT_OBSERVE, _s4(k * 0.1, 0.30), k * 0.1)   # block not moving
    assert core.picked[0] == 0.0
    core.step(AT_OBSERVE, _s4(4.6, 0.30), 4.6)
    assert core.picked[0] == 1.0
    assert any("ANFAHREN" in e and "Zeit" in e for e in core.pop_events())

    core = _tracking_core(params=PARAMS, y=0.0)              # already in the zone
    for k in range(1, 30):                                   # block runs on
        core.step(AT_OBSERVE, _s4(k * 0.1, -0.01 * k), k * 0.1)
    assert core.state == STATE_FOLLOW and core.picked[0] == 0.0
    core.step(AT_OBSERVE, _s4(3.1, -0.31), 3.1)              # 3 s in FOLGEN
    assert core.picked[0] == 1.0


def test_a_finished_target_is_not_taken_again():
    """After an abort, S4 may show the same target for a few cycles until the
    priority_handler has read picked_id."""
    core = _tracking_core()
    core.step(AT_OBSERVE, _s4(1.0, 0.2, tid=7), 1.0)       # abort on ID 1
    for k in range(5):
        out = core.step(AT_OBSERVE, _s4(1.0 + k * 0.01, 0.2, tid=7), 1.0 + k * 0.01)
    assert out.state == STATE_APPROACH                     # takes 7, a new one
    core2 = _tracking_core()
    core2.step(AT_OBSERVE, _s4(1.0, 0.2, has=False), 1.0)   # abort on ID 1
    out = core2.step(AT_OBSERVE, _s4(1.01, 0.2), 1.01)     # 1 still shown
    assert out.state == STATE_WAIT and core2.picked[0] == 1.0


def test_tracking_target_outside_the_workspace_aborts():
    core = _tracking_core(y=-0.20)
    core.step(AT_OBSERVE, _s4(0.1, -0.21), 0.1)
    out = core.step(AT_OBSERVE, _s4(0.2, -0.62), 0.2)        # beyond ws_y_min
    assert out.target is None or out.target.y >= PARAMS.ws_y_min
    assert out.state in (STATE_ABORT, STATE_WAIT) and core.picked[0] == 1.0


def test_status_reports_the_error_in_belt_coordinates():
    core = _tracking_core(y=-0.10, now=1.0)
    flange = AT_OBSERVE._replace(x=BELT_X + 0.01, y=-0.10 - 0.03)
    out = core.step(flange, _s4(1.0, -0.10), 1.0)
    assert abs(out.status.err_along - 0.03) < 1e-12      # 30 mm downstream
    assert abs(out.status.err_across - 0.01) < 1e-12
    assert abs(out.status.err_z) < 1e-12


def test_attempts_are_counted():
    core = _tracking_core()
    core.step(AT_OBSERVE, _s4(1.0, 0.2, tid=7), 1.0)        # abort 1, take 7 next
    core.step(AT_OBSERVE, _s4(1.01, 0.2, tid=7), 1.01)
    core.step(AT_OBSERVE, _s4(1.02, 0.2, has=False), 1.02)  # abort 7
    assert core.picked == (2.0, 7.0, float(OUTCOME_ABORTED))


# -- Closed loop: following a moving block ----------------------------------------------------

def _follow(lead_time_s, seconds=8.0, k_gain=5.0, v_max=0.25, latency=0.05):
    """Attractor of first order with gain K and speed cap, 100 Hz. The block
    runs at 0.1 m/s; S4 comes at 30 Hz, ``latency`` s after its timestamp."""
    params = replace(TRACK, lead_time_s=lead_time_s)
    core = FollowerCore(params)
    flange = list(AT_OBSERVE.position)
    errors, events = [], []
    s4 = None
    for step in range(int(seconds * 100)):
        now = step * 0.01
        frame_t = math.floor((now - latency) * 30) / 30
        if frame_t >= 0:
            s4 = _s4(frame_t, 0.30 - 0.1 * frame_t)
        out = core.step(Pose(*flange, *AT_OBSERVE.orientation), s4, now)
        events += core.pop_events()
        if out.target is not None:
            step_v = [k_gain * (t - f) for t, f in zip(out.target.position, flange)]
            norm = math.sqrt(sum(c * c for c in step_v))
            scale = min(1.0, v_max / norm) if norm > 0 else 1.0
            flange = [f + c * scale * 0.01 for f, c in zip(flange, step_v)]
        if out.state == STATE_FOLLOW:
            errors.append(out.status.err_along)
    return core, errors, events


def test_with_the_right_lead_the_flange_sits_on_the_block():
    """lead_time_s = 1/K: the steady-state lag v/K is cancelled (Nachtrag 6 / Z6)."""
    core, errors, events = _follow(lead_time_s=0.2)
    assert core.state == STATE_FOLLOW
    steady = errors[-100:]
    assert max(abs(e) for e in steady) < 0.002
    assert not any("Vorhalt" in e for e in events)


def test_without_lead_the_flange_lags_by_v_over_k_and_says_so():
    """The B4 calibration signal: 0.1 m/s / K 5 = 20 mm behind the block."""
    core, errors, events = _follow(lead_time_s=0.0)
    steady = sum(errors[-100:]) / 100
    assert abs(steady - (-0.020)) < 0.002
    assert any("Vorhalt passt nicht" in e and "hinter" in e for e in events)

