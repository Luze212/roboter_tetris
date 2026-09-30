"""Tests for the synthetic objects generator (Phase 3.0).

The scene model is free of ROS imports, so the whole thing is checkable at the
desk -- which is the point of the tool in the first place.
"""

import importlib.util
import os

from roboter_tetris.contracts import (
    OBJECTS_HEADER, OBJECTS_STRIDE, unpack_objects,
)

# The tool lives outside the package (test/tools), so load it by path.
_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "tools", "fake_objects.py")
_spec = importlib.util.spec_from_file_location("fake_objects", _TOOL)
fake_objects = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fake_objects)

FakeBelt = fake_objects.FakeBelt
FakeBlock = fake_objects.FakeBlock
BELT_SURFACE_Z_M = fake_objects.BELT_SURFACE_Z_M


def _belt(**kw):
    return FakeBelt(fake_objects.default_blocks(), **kw)


def test_blocks_appear_one_after_another():
    belt = _belt()
    assert len(belt.objects_at(0.0)) == 1
    assert len(belt.objects_at(4.0)) == 2
    assert len(belt.objects_at(8.0)) == 3


def test_block_travels_at_the_configured_velocity():
    belt = _belt(v_belt_mps=-0.100)
    y0 = belt.y_of(belt.blocks[0], 0.0)
    y1 = belt.y_of(belt.blocks[0], 1.0)
    assert abs((y1 - y0) - (-0.100)) < 1e-9


def test_block_leaves_the_tracker_window():
    belt = _belt(v_belt_mps=-0.100)
    block = belt.blocks[0]
    # Spawn +0.600, despawn -0.800 -> 1.40 m -> 14.0 s at 0.1 m/s.
    assert belt.y_of(block, 13.0) is not None
    assert belt.y_of(block, 15.0) is None




def test_signal_is_a_valid_s1_array():
    belt = _belt()
    arr = belt.signal_at(8.0)
    assert len(arr) == OBJECTS_HEADER + 3 * OBJECTS_STRIDE
    msg = unpack_objects(arr)
    assert msg.t == 8.0
    assert abs(msg.v_belt - (-0.100)) < 1e-9
    assert [o.id for o in msg.objects] == [1.0, 2.0, 3.0]


def test_z_mirrors_what_base_cam_actually_reports():
    # Measured 14.09.: a 100 mm block came out at 103.1 mm, and
    # 53.6 + 100/2 = 103.6 mm matches that within 0.5 mm -- so mid-height,
    # not the top edge the contract table claims.
    belt = _belt()
    obj = belt.objects_at(0.0)[0]
    assert abs(obj.z - (BELT_SURFACE_Z_M + 0.100 / 2.0)) < 1e-9
    assert abs(obj.z - 0.1036) < 1e-9


def test_header_is_sent_even_before_any_block_appears():
    belt = FakeBelt([FakeBlock(id=1, color=0, length=0.05, width=0.05,
                               height=0.1, spawn_t=5.0)])
    arr = belt.signal_at(0.0)
    assert len(arr) == OBJECTS_HEADER
    assert unpack_objects(arr).objects == []


def test_sizes_cover_the_graspability_limits():
    # B15 with 5 mm air (Nachtrag 6 / Z7): below ~30 mm a block is not graspable.
    # B16: the gripper opens 127 mm. The scene must contain both a graspable
    # block and one that is deliberately NOT graspable, as a negative case.
    belt = _belt()
    heights = [b.height for b in belt.blocks]
    widths = [max(b.length, b.width) for b in belt.blocks]
    assert max(widths) < 0.127
    assert any(h >= 0.030 for h in heights)
    assert any(h < 0.020 for h in heights)      # the 15 mm block (L24)


# -- Toppling on placement ------------------------------------------------------

def test_topple_forward_gets_the_geometry_right():
    """The 50 x 50 x 100 reference block pivots about its front edge: its centre
    jumps by (100 - 50) / 2 = 25 mm and it ends up 100 long, 50 high."""
    block = fake_objects.default_blocks()[0]
    t = fake_objects.topple_forward(block, after_s=0.3)
    assert abs(t.shift_along_m - 0.025) < 1e-12
    assert (t.length, t.width, t.height) == (0.100, 0.050, 0.050)


def test_a_block_that_is_not_standing_cannot_topple_forward():
    lying = fake_objects.default_blocks()[1]          # 76 x 50 x 50
    try:
        fake_objects.topple_forward(lying)
    except ValueError:
        return
    raise AssertionError("ein liegender Klotz darf nicht nach vorn umkippen")


def test_toppled_block_jumps_and_changes_shape_at_the_right_time():
    blocks = fake_objects.default_blocks()[:1]
    blocks[0].topple = fake_objects.topple_forward(blocks[0], after_s=0.3)
    belt = FakeBelt(blocks, v_belt_mps=-0.100)
    before = belt.objects_at(0.29)[0]
    after = belt.objects_at(0.31)[0]
    # Along -y the belt travels 2 mm in 0.02 s; the topple adds 25 mm forward.
    assert abs((before.y - after.y) - (0.002 + 0.025)) < 1e-9
    assert before.height == 0.100 and after.height == 0.050
    assert after.length == 0.100
    assert abs(after.z - (BELT_SURFACE_Z_M + 0.025)) < 1e-9


def test_topple_jumps_forward_on_a_belt_running_along_plus_y():
    blocks = fake_objects.default_blocks()[:1]
    blocks[0].topple = fake_objects.topple_forward(blocks[0], after_s=0.3)
    belt = FakeBelt(blocks, v_belt_mps=+0.100, y_spawn_m=-0.300, y_despawn_m=1.080)
    assert belt.objects_at(0.31)[0].y > belt.objects_at(0.29)[0].y + 0.025


# -- Measurement noise ------------------------------------------------------------

def test_noise_is_reproducible_with_a_seed():
    a = FakeBelt(fake_objects.default_blocks(), noise_sigma_m=0.0005, seed=7)
    b = FakeBelt(fake_objects.default_blocks(), noise_sigma_m=0.0005, seed=7)
    assert [a.signal_at(k / 30.0) for k in range(10)] == \
           [b.signal_at(k / 30.0) for k in range(10)]


def test_noise_has_the_requested_spread_and_no_bias():
    belt = FakeBelt(fake_objects.default_blocks()[:1], noise_sigma_m=0.0005, seed=3)
    errs = [belt.objects_at(0.0)[0].x - fake_objects.BELT_CENTER_X_M
            for _ in range(4000)]
    mean = sum(errs) / len(errs)
    sd = (sum((e - mean) ** 2 for e in errs) / len(errs)) ** 0.5
    assert abs(mean) < 0.00005
    assert abs(sd - 0.0005) < 0.00005


def test_without_noise_the_scene_is_exact():
    belt = FakeBelt(fake_objects.default_blocks()[:1])
    assert belt.objects_at(1.0)[0].x == fake_objects.BELT_CENTER_X_M
