"""Tests for the overview image of `interface_streamer` (Phase 5).

Text without ROS; composition with synthetic images (numpy + OpenCV, like the
vision tests).
"""

import math

import numpy as np

from roboter_tetris.contracts import (
    COLOR_BLUE, COLOR_RED, STATE_FOLLOW, TRACK_FINAL, TRACK_SETTLING,
    TrackEntry, WorldEntry, pack_follower_status, pack_world_state,
    unpack_follower_status, unpack_world_state,
)
from roboter_tetris.interface_layout import (
    LIST_ROWS, compose, fit_image, object_row, object_rows, panel_height,
    status_lines,
)


def _entry(tid, status=TRACK_FINAL, color=COLOR_BLUE, picked=0.0, oob=0.0,
           present=1.0, vy=-0.1, v_change=0.004):
    track = TrackEntry(id=float(tid), color=float(color), x=-0.816, y=0.1, z=0.1036,
                       orientation=0.3, length=0.05, width=0.05, height=0.1,
                       status=float(status), vx=0.0, vy=vy, v_change=v_change,
                       ori_quality=0.9)
    return WorldEntry(*track, picked=picked, out_of_bounds=oob, present=present)


def _world(entries, n_pool=4, v=(0.0017, -0.101)):
    return unpack_world_state(pack_world_state(10.0, v, n_pool, entries))


def _status():
    return unpack_follower_status(
        pack_follower_status(10.0, STATE_FOLLOW, 3, 0.0021, -0.0008, 0.0014, 0.65))


# -- Text -----------------------------------------------------------------------------

def test_status_lines_show_state_target_weight_errors_and_belt():
    rows = [r.text for r in status_lines(_status(), _world([]))]
    assert rows[0] == "Zustand: FOLGEN   Ziel: 3   w: 0.65"
    assert rows[1] == "Abweichung  laengs +2.1 mm   quer -0.8 mm   z +1.4 mm"
    speed = math.hypot(0.0017, -0.101) * 1000
    heading = math.degrees(math.atan2(-0.101, 0.0017))
    assert rows[2] == (f"Band  v = {speed:.0f} mm/s   Richtung {heading:+.1f} Grad   "
                       f"(4 Kloetze)")


def test_missing_or_stale_sources_are_said_not_hidden():
    rows = status_lines(None, None)
    assert rows[0].text == "Follower: keine Daten" and rows[0].warning
    assert rows[2].text == "Band: keine Daten"
    stale = status_lines(_status(), _world([]), follower_stale=True, world_stale=True)
    assert "(veraltet)" in stale[0].text and "(veraltet)" in stale[2].text


def test_no_belt_estimate_yet_is_stated():
    row = status_lines(_status(), _world([], n_pool=0))[2]
    assert row.text.startswith("Band: noch keine Schaetzung")


def test_each_block_shows_why_it_is_or_is_not_a_candidate():
    assert "final" in object_row(_entry(3)).text and "v = 100 mm/s" in object_row(_entry(3)).text
    settling = object_row(_entry(4, status=TRACK_SETTLING, color=COLOR_RED, v_change=0.031))
    assert "einschwingend" in settling.text and "dv = 31 mm/s" in settling.text
    assert "rot" in settling.text
    assert object_row(_entry(2, picked=1.0)).text.endswith("gepickt")
    assert object_row(_entry(5, oob=1.0)).text.endswith("hinter der Greifebene")


def test_a_block_that_left_the_image_shows_no_position():
    """T1: the values are frozen -- never show them as if the block were there."""
    row = object_row(_entry(6, present=0.0))
    assert row.text.endswith("nicht mehr im Bild") and row.dimmed
    assert "mm/s" not in row.text


def test_long_lists_keep_the_newest_entries():
    rows = object_rows(_world([_entry(i) for i in range(1, 15)]))
    assert len(rows) == LIST_ROWS
    assert rows[0].text.startswith("... 7 aeltere")
    assert "ID   14" in rows[-1].text


def test_all_text_is_ascii():
    """OpenCV's Hershey fonts cannot draw umlauts, degrees or Greek letters."""
    rows = (status_lines(_status(), _world([])) + status_lines(None, None)
            + object_rows(_world([_entry(1), _entry(2, status=TRACK_SETTLING),
                                  _entry(3, picked=1.0), _entry(4, oob=1.0),
                                  _entry(5, present=0.0)])))
    for row in rows:
        assert row.text.isascii(), row.text


# -- Composition ---------------------------------------------------------------------------

def test_panel_size_depends_only_on_the_parameters():
    base = np.zeros((720, 1280, 3), np.uint8)
    robot = np.zeros((480, 848, 3), np.uint8)
    for images in ((base, robot), (None, robot), (base, None), (None, None)):
        panel = compose(*images, status_lines(None, None), object_rows(None), 1280, 360, True)
        assert panel.shape == (panel_height(360, True), 1280, 3)
    small = compose(None, None, status_lines(None, None), [], 640, 180, False)
    assert small.shape == (panel_height(180, False), 640, 3)


def test_images_are_scaled_keeping_their_aspect():
    """848 x 480 into 640 x 360: factor 0.75 -> 636 x 360, centred."""
    image = np.full((480, 848, 3), 255, np.uint8)
    area = fit_image(image, 640, 360, "Roboterkamera")
    white = np.argwhere(area[:, :, 0] == 255)
    assert white[:, 1].min() == 2 and white[:, 1].max() == 637
    assert white[:, 0].min() == 0 and white[:, 0].max() == 359


def test_missing_image_is_grey_with_a_label_and_mono_images_work():
    area = fit_image(None, 640, 360, "Basiskamera")
    assert area.shape == (360, 640, 3) and (area[5, 5] == (80, 80, 80)).all()
    assert (area != 80).any()                             # the label was drawn
    mono = fit_image(np.full((100, 100), 200, np.uint8), 640, 360, "x")
    assert mono.shape == (360, 640, 3)
