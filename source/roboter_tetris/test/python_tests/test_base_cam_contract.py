"""Tests for the S1 packing of base_cam (Phase 2.1).

The existing test_base_cam_vision.py covers the vision modules, which keep
working in millimetres. This file covers the signal boundary: the conversion to
SI units and the move of the belt velocity into the header.
"""

from roboter_tetris.base_cam import pack_tracked_objects
from roboter_tetris.contracts import (
    OBJECTS_HEADER, OBJECTS_STRIDE, unpack_objects,
)
from roboter_tetris.vision.tracker import TrackedObject


def _track(tid=1, **kw):
    """A tracked object in millimetres, as the vision modules produce it."""
    values = dict(id=tid, color=3, x=814.0, y=-874.0, z=103.1,
                  orientation=0.5, vy=-120.0,
                  length=58.8, width=52.3, height=100.0)
    values.update(kw)
    return TrackedObject(**values)


def test_millimetres_become_metres():
    arr = pack_tracked_objects(12.5, -120.0, [_track()])
    msg = unpack_objects(arr)
    assert msg.t == 12.5
    obj = msg.objects[0]
    assert abs(obj.x - 0.814) < 1e-9
    assert abs(obj.y - (-0.874)) < 1e-9
    assert abs(obj.z - 0.1031) < 1e-9
    assert abs(obj.length - 0.0588) < 1e-9
    assert abs(obj.width - 0.0523) < 1e-9
    assert abs(obj.height - 0.100) < 1e-9
    # An angle is not a length -- it must NOT be scaled.
    assert obj.orientation == 0.5


def test_belt_velocity_sits_in_the_header_in_si():
    arr = pack_tracked_objects(1.0, -120.0, [_track(1), _track(2)])
    msg = unpack_objects(arr)
    assert abs(msg.v_belt - (-0.120)) < 1e-9
    assert len(msg.objects) == 2
    # Stride 9: the per-object vy of the old format is gone.
    assert len(arr) == OBJECTS_HEADER + 2 * OBJECTS_STRIDE


def test_velocity_is_not_duplicated_onto_the_objects():
    # The tracker assigns one global velocity to all tracks, so a jammed block
    # would otherwise be tagged with the full belt speed.
    arr = pack_tracked_objects(1.0, -120.0, [_track(vy=0.0)])
    body = arr[OBJECTS_HEADER:]
    assert len(body) == OBJECTS_STRIDE
    assert -0.120 not in body


def test_header_is_sent_without_objects():
    arr = pack_tracked_objects(7.5, -120.0, [])
    assert len(arr) == OBJECTS_HEADER
    msg = unpack_objects(arr)
    assert msg.t == 7.5
    assert msg.objects == []
    # "sees nothing" still carries a running t -- that is how a receiver tells
    # it apart from "no longer sending".
    assert abs(msg.v_belt - (-0.120)) < 1e-9


def test_ids_and_colors_survive_the_round_trip():
    arr = pack_tracked_objects(0.0, 0.0, [_track(4, color=0), _track(9, color=4)])
    msg = unpack_objects(arr)
    assert [o.id for o in msg.objects] == [4.0, 9.0]
    assert [o.color for o in msg.objects] == [0.0, 4.0]
