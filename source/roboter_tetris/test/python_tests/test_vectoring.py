"""Tests for the velocity estimation behind `vectoring` (project goal 3).

The ground truth comes from ``test/tools/fake_objects.py``: on the real belt
nobody knows the true velocity, on the synthetic one we do. No ROS, no numpy --
this runs with plain python3.
"""

import importlib.util
import math
import os

from roboter_tetris.contracts import (
    ObjectEntry, TRACK_FINAL, TRACK_PREDICTED, TRACK_SETTLING, unpack_objects,
    unpack_tracks,
)
from roboter_tetris.track_estimation import (
    EstimatorParams, TrackEstimator, mean_orientation,
)

_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "tools", "fake_objects.py")
_spec = importlib.util.spec_from_file_location("fake_objects", _TOOL)
fake_objects = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fake_objects)

FPS = 30.0
V_TRUE = -0.100              # fake_objects default: 0.1 m/s along -y


def _run(belt, seconds, est=None, t0=0.0, every_frame=None):
    """Feed ``seconds`` of the synthetic belt at 30 Hz; return the estimator.

    ``every_frame(t, est)`` is called after each update, for time histories.
    """
    est = est or TrackEstimator()
    for k in range(int(round(seconds * FPS))):
        t = t0 + k / FPS
        msg = unpack_objects(belt.signal_at(t - t0))
        est.update(t, msg.objects)
        if every_frame:
            every_frame(t, est)
    return est


def _one_block(noise=0.0005, seed=1, topple=None):
    blocks = fake_objects.default_blocks()[:1]
    blocks[0].topple = topple
    return fake_objects.FakeBelt(blocks, noise_sigma_m=noise, seed=seed)


def _obj(oid, x, y, height=0.1, orientation=0.0):
    return ObjectEntry(id=float(oid), color=0.0, x=x, y=y, z=0.1036,
                       orientation=orientation, length=0.05, width=0.05,
                       height=height)


# -- Velocity -------------------------------------------------------------------

def test_velocity_matches_the_ground_truth():
    est = _run(_one_block(), 6.0)
    v, n_pool = est.belt_velocity()
    assert n_pool == 1
    assert abs(v[1] - V_TRUE) < 0.0005         # better than 0.5 mm/s
    assert abs(v[0]) < 0.0005


def test_direction_is_estimated_not_assumed():
    """A belt running at an angle: both components come out, nothing assumes y."""
    est = TrackEstimator()
    vx, vy = 0.030, -0.095
    for k in range(180):
        t = k / FPS
        est.update(t, [_obj(1, 0.8 + vx * t, 0.3 + vy * t)])
    v, _ = est.belt_velocity()
    assert abs(v[0] - vx) < 1e-9 and abs(v[1] - vy) < 1e-9


# -- Settling -------------------------------------------------------------------

def test_block_settles_and_stays_final():
    history = []
    _run(_one_block(), 4.0,
         every_frame=lambda t, est: history.append(est.snapshot(t)[2][0].status))
    first_final = history.index(float(TRACK_FINAL))
    # Two half windows of 15 are needed: never final before the 30th frame.
    assert first_final >= 29
    assert all(s == float(TRACK_SETTLING) for s in history[:first_final])
    assert all(s == float(TRACK_FINAL) for s in history[first_final:])


def test_toppling_block_settles_late_but_with_the_right_velocity():
    """The regression test for Z9. The first design declared a block that
    toppled by 13 or 25 mm final in every case, 16-31 mm/s off. Now it must
    stay settling until the jump has left both half windows -- and then be
    final with the correct velocity."""
    reference_block = fake_objects.default_blocks()[0]
    cases = {
        "25 mm": fake_objects.topple_forward(reference_block, after_s=0.3),
        "13 mm": fake_objects.Topple(after_s=0.3, shift_along_m=0.013,
                                     length=0.100, width=0.050, height=0.050),
    }
    for label, topple in cases.items():
        seen = []
        _run(_one_block(topple=topple), 4.0,
             every_frame=lambda t, est: seen.append((t, est.snapshot(t)[2][0])))
        first = next((t, e) for t, e in seen if e.status == float(TRACK_FINAL))
        t_final, entry = first
        assert t_final > 0.3 + 2 * 15 / FPS - 0.05, \
            f"{label}: final nach {t_final:.2f}s, der Sprung war noch im Fenster"
        assert abs(entry.vy - V_TRUE) < 0.003, \
            f"{label}: final mit vy = {entry.vy * 1000:.1f} mm/s"
        # And it reports its lying shape, not a mix of standing and lying.
        assert abs(seen[-1][1].height - 0.050) < 1e-9


def test_status_3_reports_the_last_measurement_and_no_quality():
    est = TrackEstimator()
    est.update(0.0, [_obj(1, 0.8, 0.30, height=0.1)])
    est.update(1 / FPS, [_obj(1, 0.8, 0.29, height=0.1)])
    entry = est.snapshot(1 / FPS)[2][0]
    assert entry.status == float(TRACK_SETTLING)
    assert entry.y == 0.29
    assert entry.ori_quality == 0.0


# -- Outliers and restarts ----------------------------------------------------------

def test_single_outlier_is_dropped_and_the_track_stays_final():
    est = TrackEstimator()
    statuses = []
    for k in range(120):
        t = k / FPS
        y = 0.3 + V_TRUE * t + (0.040 if k == 80 else 0.0)   # one bad frame
        est.update(t, [_obj(1, 0.8, y)])
        statuses.append(est.snapshot(t)[2][0].status)
    assert all(s == float(TRACK_FINAL) for s in statuses[40:])
    v, _ = est.belt_velocity()
    assert abs(v[1] - V_TRUE) < 1e-9          # the outlier did not leak in


def test_persistent_deviation_restarts_the_track_but_keeps_its_pool_share():
    est = TrackEstimator()
    for k in range(120):
        t = k / FPS
        y = 0.3 + V_TRUE * t + (0.040 if k >= 80 else 0.0)   # lasting jump
        est.update(t, [_obj(1, 0.8, y)])
        if k == 79:
            assert est.snapshot(t)[2][0].status == float(TRACK_FINAL)
        if k == 83:
            assert est.snapshot(t)[2][0].status == float(TRACK_SETTLING)
    _, n_pool = est.belt_velocity()
    assert n_pool >= 1                          # the clean section stayed


def test_non_finite_measurement_is_dropped_not_the_track():
    est = TrackEstimator()
    est.update(0.0, [_obj(1, 0.8, 0.3)])
    est.update(1 / FPS, [_obj(1, float("nan"), 0.29)])
    est.update(2 / FPS, [_obj(1, 0.8, 0.28)])
    tracks = est.snapshot(2 / FPS)[2]
    assert len(tracks) == 1 and tracks[0].y == 0.28


# -- Pool ---------------------------------------------------------------------------

def test_no_belt_estimate_before_the_first_block_settles():
    est = _run(_one_block(), 0.5)
    assert est.belt_velocity() == (None, 0)
    msg = unpack_tracks(est.pack(0.5))
    assert msg.n_pool == 0.0                    # stated, not a silent 0.0 speed


def test_pool_keeps_the_share_of_blocks_that_have_left():
    est = TrackEstimator()
    for k in range(90):                         # two blocks for 3 s
        t = k / FPS
        est.update(t, [_obj(1, 0.8, 0.3 + V_TRUE * t),
                       _obj(2, 0.7, 0.6 + V_TRUE * t)])
    for k in range(90, 150):                    # block 1 is gone (gripped)
        t = k / FPS
        est.update(t, [_obj(2, 0.7, 0.6 + V_TRUE * t)])
    entries = {e.id: e for e in est.snapshot(t)[2]}
    # Track 1 is no longer measured: carried on with the belt (Nachtrag 13 / L10) ...
    assert entries[1.0].status == TRACK_PREDICTED
    assert entries[2.0].status == TRACK_FINAL
    assert abs(entries[1.0].y - (0.3 + V_TRUE * t)) < 0.002
    assert est.belt_velocity()[1] == 2          # ... and its pool share stays
    for k in range(150, 150 + int((est.params.predict_max_s + 0.5) * FPS)):
        t = k / FPS
        est.update(t, [_obj(2, 0.7, 0.6 + V_TRUE * t)])
    assert [e.id for e in est.snapshot(t)[2]] == [2.0]   # predicted at most predict_max_s
    assert est.belt_velocity()[1] == 2


def test_a_settling_track_is_forgotten_not_predicted():
    est = TrackEstimator()
    est.update(0.0, [_obj(1, 0.8, 0.3)])
    est.update(0.1, [_obj(1, 0.8, 0.3 + V_TRUE * 0.1)])
    est.update(0.1 + est.params.track_expiry_s + 0.1, [])
    assert est.snapshot(0.1 + est.params.track_expiry_s + 0.1)[2] == []


def test_a_new_measurement_on_a_predicted_track_ends_the_prediction():
    """Short occlusion in the image: base_cam sees the block again under a new
    ID. The predicted old track must not live on as a duplicate."""
    est = TrackEstimator()
    for k in range(60):
        t = k / FPS
        est.update(t, [_obj(1, 0.8, 0.3 + V_TRUE * t)])
    for k in range(60, 66):                     # 0.2 s unseen -> predicted
        t = k / FPS
        est.update(t, [])
    assert est.snapshot(t)[2][0].status == TRACK_PREDICTED
    t = 66 / FPS
    est.update(t, [_obj(7, 0.8, 0.3 + V_TRUE * t)])
    assert [e.id for e in est.snapshot(t)[2]] == [7.0]


def test_standing_objects_stay_out_of_the_pool():
    """Block 3, 23.09.2026: two blocks measured standing for half a minute
    buried every moving block -- the pool said 0 mm/s while each block ran at
    -127 mm/s. Sections below pool_min_speed_mps stay out."""
    est = TrackEstimator(EstimatorParams(settle_half_window=5, smoothing_window=5))
    t = 0.0
    for _ in range(300):                               # 30 s standing, 10 Hz
        est.update(t, [_obj(1, -0.80, 0.70)])
        t += 0.1
    assert est.belt_velocity() == (None, 0)           # standing: no belt estimate
    y = 0.95
    for _ in range(30):                                # 3 s moving at -0.127 m/s
        est.update(t, [_obj(2, -0.85, y)])
        t += 0.1
        y -= 0.0127
    v, n_pool = est.belt_velocity()
    assert n_pool == 1
    assert abs(v[1] - (-0.127)) < 1e-6 and abs(v[0]) < 1e-9


def test_a_long_clean_section_counts_more_than_a_short_one():
    est = TrackEstimator()
    for k in range(180):                        # 6 s at exactly 0.100 m/s
        t = k / FPS
        objs = [_obj(1, 0.8, 0.3 + V_TRUE * t)]
        if 60 <= k < 96:                        # 1.2 s at 0.110 m/s
            objs.append(_obj(2, 0.7, 0.9 - 0.110 * (t - 2.0)))
        est.update(t, objs)
    v, n_pool = est.belt_velocity()
    assert n_pool == 2
    # An unweighted mean would be -0.105; the long section must dominate.
    assert abs(v[1] - V_TRUE) < abs(v[1] - (-0.105))


def test_large_timestamps_give_the_same_result():
    """With epoch timestamps around 1.7e9 s the naive sums cancel to noise.
    Times are kept relative to the section start, so nothing changes."""
    a = _run(_one_block(seed=5), 5.0, t0=0.0)
    b = _run(_one_block(seed=5), 5.0, t0=1.7e9)
    va, vb = a.belt_velocity()[0], b.belt_velocity()[0]
    assert vb is not None, (
        "mit Zeitstempeln um 1,7e9 s gibt es gar keinen Schätzwert -- die Summen "
        "sind ausgelöscht (Zeiten nicht relativ zum Abschnittsbeginn?)")
    assert abs(va[0] - vb[0]) < 1e-6 and abs(va[1] - vb[1]) < 1e-6


# -- Smoothing ----------------------------------------------------------------------

def test_smoothed_position_is_valid_for_the_header_time():
    est = _run(_one_block(noise=0.0005, seed=2), 4.0)
    t = (4.0 * FPS - 1) / FPS
    entry = est.snapshot(t)[2][0]
    y_true = fake_objects.Y_SPAWN_M + V_TRUE * t
    assert abs(entry.y - y_true) < 0.001
    assert abs(entry.x - fake_objects.BELT_CENTER_X_M) < 0.001


def test_orientation_mean_uses_the_doubled_angle():
    # Angles near 0 and near pi are the same line: mean near 0, quality near 1.
    mean, quality = mean_orientation([0.02, math.pi - 0.02] * 10)
    assert min(mean, math.pi - mean) < 1e-9 and quality > 0.99
    # 70 % at 0 deg, 30 % at 90 deg: the dominant one wins, never 45 deg.
    mean, quality = mean_orientation([0.0] * 7 + [math.pi / 2] * 3)
    assert min(mean, math.pi - mean) < 1e-9
    assert abs(quality - 0.4) < 1e-9


def test_final_track_reports_orientation_quality():
    est = TrackEstimator()
    for k in range(60):
        t = k / FPS
        est.update(t, [_obj(1, 0.8, 0.3 + V_TRUE * t, orientation=0.6)])
    entry = est.snapshot(t)[2][0]
    assert entry.status == float(TRACK_FINAL)
    assert abs(entry.orientation - 0.6) < 1e-9 and entry.ori_quality > 0.999


# -- Output ---------------------------------------------------------------------------

def test_pack_produces_a_valid_s3_array():
    est = _run(_one_block(), 3.0)
    msg = unpack_tracks(est.pack(3.0))
    assert msg.n_pool == 1.0
    assert abs(msg.v_belt_y - V_TRUE) < 0.001
    assert len(msg.tracks) == 1 and msg.tracks[0].status == float(TRACK_FINAL)


def test_live_window_change_does_not_leave_a_track_unsettlable():
    est = TrackEstimator(EstimatorParams(settle_half_window=5, smoothing_window=5))
    for k in range(5):
        est.update(k / FPS, [_obj(1, 0.8, 0.3 + V_TRUE * k / FPS)])
    est.params = EstimatorParams(settle_half_window=15, smoothing_window=30)
    for k in range(5, 60):
        est.update(k / FPS, [_obj(1, 0.8, 0.3 + V_TRUE * k / FPS)])
    assert est.snapshot(59 / FPS)[2][0].status == float(TRACK_FINAL)
