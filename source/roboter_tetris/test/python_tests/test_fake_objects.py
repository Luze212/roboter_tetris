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
    # Spawn +0.300, despawn -1.080 -> 1.38 m -> 13.8 s at 0.1 m/s.
    assert belt.y_of(block, 13.0) is not None
    assert belt.y_of(block, 15.0) is None


def test_stalled_block_holds_its_position():
    """The scene that must yield plausibility status 1 in vectoring."""
    blocks = fake_objects.default_blocks()
    blocks[0].stall_at_y_m = -0.700
    belt = FakeBelt(blocks, v_belt_mps=-0.100)
    assert abs(belt.y_of(blocks[0], 10.0) - (-0.700)) < 1e-9
    assert abs(belt.y_of(blocks[0], 30.0) - (-0.700)) < 1e-9
    # and it must not be despawned while it sits there
    assert belt.y_of(blocks[0], 60.0) == -0.700


def test_stall_also_works_for_a_belt_running_the_other_way():
    blocks = [FakeBlock(id=1, color=0, length=0.05, width=0.05, height=0.1,
                        stall_at_y_m=0.100)]
    belt = FakeBelt(blocks, v_belt_mps=+0.100, y_spawn_m=-0.300,
                    y_despawn_m=1.080)
    assert abs(belt.y_of(blocks[0], 10.0) - 0.100) < 1e-9


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
    # B15: below ~24 mm height a block cannot be gripped centrally.
    # B16: the gripper opens 127 mm.
    belt = _belt()
    heights = [b.height for b in belt.blocks]
    widths = [max(b.length, b.width) for b in belt.blocks]
    assert min(heights) > 0.024 and max(widths) < 0.127
