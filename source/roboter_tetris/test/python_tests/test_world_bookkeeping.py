"""Tests for the bookkeeping behind `data_tracker` (display only).

No ROS, no numpy -- this runs with plain python3. The end-to-end test drives
the synthetic belt through `vectoring`'s estimator and the target selection
into the list, with a stand-in follower that grips, carries and places.
"""

import importlib.util
import os

from roboter_tetris.contracts import (
    OUTCOME_PLACED, OUTCOME_TOO_LATE, TRACK_FINAL, PickedId, TrackEntry,
    TracksMsg, unpack_not_pickable, unpack_objects, unpack_world_state,
)
from roboter_tetris.target_selection import TargetSelector
from roboter_tetris.track_estimation import TrackEstimator
from roboter_tetris.world_bookkeeping import WorldBook

_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "tools", "fake_objects.py")
_spec = importlib.util.spec_from_file_location("fake_objects", _TOOL)
fake_objects = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fake_objects)


def _track(tid, y=0.0, status=TRACK_FINAL):
    return TrackEntry(id=float(tid), color=2.0, x=0.8, y=y, z=0.1036,
                      orientation=0.3, length=0.05, width=0.05, height=0.1,
                      status=float(status), vx=0.0, vy=-0.1, v_change=0.001,
                      ori_quality=0.9)


def _msg(t, tracks, v_belt=(0.0, -0.1), n_pool=1):
    return TracksMsg(t=float(t), v_belt_x=v_belt[0], v_belt_y=v_belt[1],
                     n_pool=float(n_pool), tracks=list(tracks))


def _entries(book):
    return {e.id: e for e in unpack_world_state(book.pack()).entries}


# -- The list ---------------------------------------------------------------------

def test_empty_before_the_first_frame():
    msg = unpack_world_state(WorldBook().pack())
    assert (msg.t, msg.n_pool, msg.entries) == (0.0, 0.0, [])


def test_ids_appear_in_order_of_first_appearance_with_their_s3_fields():
    book = WorldBook()
    book.update_tracks(_msg(1.0, [_track(5, y=0.2)]))
    book.update_tracks(_msg(1.1, [_track(2, y=0.3), _track(5, y=0.19)]))
    entries = unpack_world_state(book.pack()).entries
    assert [e.id for e in entries] == [5.0, 2.0]
    assert tuple(entries[0])[:14] == tuple(_track(5, y=0.19))
    assert (entries[0].picked, entries[0].out_of_bounds, entries[0].present) == \
        (0.0, 0.0, 1.0)


def test_header_is_passed_through_unchanged():
    book = WorldBook()
    book.update_tracks(_msg(4.5, [], v_belt=(0.003, -0.097), n_pool=3))
    msg = unpack_world_state(book.pack())
    assert (msg.t, msg.v_belt_x, msg.v_belt_y, msg.n_pool) == (4.5, 0.003, -0.097, 3.0)


# -- Flags ----------------------------------------------------------------------------

def test_picked_needs_outcome_zero_and_each_seq_counts_once():
    book = WorldBook()
    book.update_tracks(_msg(1.0, [_track(1), _track(2)]))
    book.on_picked(PickedId(1.0, 1.0, float(OUTCOME_TOO_LATE)))
    book.on_picked(PickedId(0.0, 2.0, float(OUTCOME_PLACED)))   # no attempt yet
    assert _entries(book)[1.0].picked == 0.0
    assert _entries(book)[2.0].picked == 0.0
    book.on_picked(PickedId(2.0, 2.0, float(OUTCOME_PLACED)))
    assert _entries(book)[2.0].picked == 1.0


def test_out_of_bounds_sticks_after_the_id_leaves_not_pickable():
    book = WorldBook()
    book.update_tracks(_msg(1.0, [_track(1)]))
    book.mark_out_of_bounds([1.0, 99.0])          # 99 unknown: ignored
    book.mark_out_of_bounds([])
    assert _entries(book)[1.0].out_of_bounds == 1.0
    assert 99.0 not in _entries(book)


# -- Follow-up and expiry ---------------------------------------------------------------

def test_vanished_block_keeps_its_last_values_with_present_zero():
    book = WorldBook()
    book.update_tracks(_msg(1.0, [_track(1, y=-0.6)]))
    book.update_tracks(_msg(1.1, []))
    entry = _entries(book)[1.0]
    assert entry.present == 0.0 and entry.y == -0.6


def test_pick_reported_after_the_block_left_the_image_still_arrives():
    """The reason for the follow-up: the block leaves the image when it is
    lifted, picked_id comes only after placing it (Nachtrag 7 / T1)."""
    book = WorldBook(expiry_after_done_s=10.0)
    book.update_tracks(_msg(0.0, [_track(1)]))
    book.update_tracks(_msg(5.0, []))                       # lifted
    book.update_tracks(_msg(9.0, []))
    book.on_picked(PickedId(1.0, 1.0, float(OUTCOME_PLACED)))   # placed
    assert _entries(book)[1.0].picked == 1.0
    book.update_tracks(_msg(16.0, []))
    assert 1.0 in _entries(book), "Frist muss ab dem picked neu laufen"
    book.update_tracks(_msg(19.0, []))
    assert 1.0 not in _entries(book)


def test_block_back_before_expiry_is_present_and_no_longer_done():
    book = WorldBook(expiry_after_done_s=2.0)
    book.update_tracks(_msg(0.0, [_track(1)]))
    book.update_tracks(_msg(0.1, []))
    book.update_tracks(_msg(0.5, [_track(1)]))
    assert _entries(book)[1.0].present == 1.0
    book.update_tracks(_msg(5.0, [_track(1)]))
    assert 1.0 in _entries(book)


def test_flagged_block_still_tracked_expires_and_stays_out_until_it_leaves():
    book = WorldBook(expiry_after_done_s=2.0)
    book.update_tracks(_msg(0.0, [_track(1)]))
    book.mark_out_of_bounds([1.0])
    book.update_tracks(_msg(2.0, [_track(1)]))
    assert 1.0 not in _entries(book)
    book.update_tracks(_msg(2.1, [_track(1)]))
    assert 1.0 not in _entries(book), "abgelaufener Eintrag kam zurück"
    book.update_tracks(_msg(2.2, []))                 # it left
    book.update_tracks(_msg(2.3, [_track(1)]))        # same ID, new block
    assert _entries(book)[1.0].out_of_bounds == 0.0


def test_expiry_counts_in_s3_time_not_in_calls():
    book = WorldBook(expiry_after_done_s=1.0)
    book.update_tracks(_msg(0.0, [_track(1)]))
    book.update_tracks(_msg(0.1, []))
    for _ in range(50):
        book.update_tracks(_msg(0.1, []))
    assert 1.0 in _entries(book)


def test_length_stays_bounded_over_many_picks():
    """300 blocks, one every 2 s, each gripped 3 s after it appeared and
    placed 3 s later: the list holds only the recent ones."""
    book = WorldBook(expiry_after_done_s=10.0)
    longest, seq = 0, 0
    for k in range(int(620 / 0.1)):
        t = k * 0.1
        alive = [_track(i) for i in range(300) if 2 * i <= t < 2 * i + 3]
        book.update_tracks(_msg(t, alive))
        for i in range(300):
            if abs(t - (2 * i + 6)) < 1e-9:
                seq += 1
                book.on_picked(PickedId(float(seq), float(i), float(OUTCOME_PLACED)))
        longest = max(longest, len(book))
    assert seq == 300
    assert longest <= 2 + (3 + 10) / 2 + 1
    assert len(book) == 0


# -- End to end: belt -> vectoring -> selection -> list ---------------------------------

def test_synthetic_run_shows_picks_and_the_missed_cube():
    belt = fake_objects.FakeBelt(fake_objects.default_blocks(),
                                 noise_sigma_m=0.0005, seed=4)
    estimator, selector, book = TrackEstimator(), TargetSelector(), WorldBook()
    gripped, placed_at, seq = {}, {}, 0
    locked_since = {}
    seen_picked, seen_oob, seen_gone = set(), set(), set()
    fps = 30.0
    # 23 s: a gripped block stays in tracks as predicted (Nachtrag 13 / L10), so
    # the target stays locked until it is placed and the run takes a little longer.
    for k in range(int(23.0 * fps)):
        t = k / fps
        objects = [o for o in unpack_objects(belt.signal_at(t)).objects
                   if o.id not in gripped]
        estimator.update(t, objects)
        v_belt, n_pool, entries = estimator.snapshot(t)
        msg = TracksMsg(t, (v_belt or (0.0, 0.0))[0], (v_belt or (0.0, 0.0))[1],
                        float(n_pool), entries)
        selection = selector.step(msg, (fake_objects.BELT_CENTER_X_M, 0.30))  # wait above the zone start
        # Stand-in follower: grip 2 s after choosing, place 3 s after gripping.
        target = selector.locked_id
        if target is not None:
            locked_since.setdefault(target, t)
            if t - locked_since[target] >= 2.0 and target not in gripped:
                gripped[target] = t
        for tid, t_grip in gripped.items():
            if tid not in placed_at and t - t_grip >= 3.0:
                placed_at[tid] = t
                seq += 1
                picked = PickedId(float(seq), tid, float(OUTCOME_PLACED))
                selector.on_picked(picked)
                book.on_picked(picked)
        if k % 3 == 0:                                     # data_tracker at 10 Hz
            book.update_tracks(msg)
            book.mark_out_of_bounds(unpack_not_pickable(selection.not_pickable).ids)
            for e in unpack_world_state(book.pack()).entries:
                if e.picked:
                    seen_picked.add(e.id)
                if e.out_of_bounds:
                    seen_oob.add(e.id)
                if not e.present:
                    seen_gone.add(e.id)
    assert seen_picked == {1.0, 2.0}
    assert seen_gone >= {1.0, 2.0}
    assert seen_oob == {3.0}
    final = _entries(book)
    assert 1.0 not in final and 2.0 not in final       # expired after placing
    # The cube keeps its out_of_bounds flag while listed; with the zone upstream
    # (23.09.2026) it may already have left the list by the end of the run.
    assert 3.0 not in final or final[3.0].out_of_bounds == 1.0
