"""Binding field layout of every array signal exchanged between the components.

Single source of truth for header lengths, strides and field indices, mirroring
``docs/architektur/datenvertraege.md``. That document is normative: a signal is
changed there first, then here -- never the other way round.

No component may index into a foreign array by hand (contract rule 8); it packs
and unpacks through this module instead.

Deliberately free of ROS, cv2 and numpy imports so the contract stays testable
on its own, without the modulo runtime.

Two conventions inherited from the contract document:

* **Header convention.** List-like signals start with ``[t, n, ...]``; the
  length is verifiable as ``header + n * stride``.
* **Never empty.** The header is sent even with no content (``[t, 0]``), which
  distinguishes "sees nothing" from "no longer sending".

Units are SI throughout (m, m/s, rad, s). Image processing works in mm
internally and converts when packing -- this module never converts, and never
interprets: extrapolation and frame transforms belong to the consumer.
"""

from typing import List, NamedTuple, Optional, Sequence

__all__ = [
    "ContractError",
    # S1
    "OBJECTS_HEADER", "OBJECTS_STRIDE", "ObjectEntry", "ObjectsMsg",
    "pack_objects", "unpack_objects",
    # S2
    "OBJECT_POSITION_LENGTH", "ObjectPosition",
    "pack_object_position", "unpack_object_position",
    # S3
    "TRACKS_HEADER", "TRACKS_STRIDE", "TrackEntry", "TracksMsg",
    "pack_tracks", "unpack_tracks",
    # S4
    "TARGET_LENGTH", "Target", "pack_target", "unpack_target",
    # S5
    "NOT_PICKABLE_HEADER", "NOT_PICKABLE_STRIDE", "NotPickableMsg",
    "pack_not_pickable", "unpack_not_pickable",
    # S7
    "PICKED_ID_LENGTH", "PickedId", "pack_picked_id", "unpack_picked_id",
    # S8
    "FOLLOWER_STATUS_LENGTH", "FollowerStatus",
    "pack_follower_status", "unpack_follower_status",
    # S10
    "WORLD_STATE_HEADER", "WORLD_STATE_STRIDE", "WorldEntry", "WorldStateMsg",
    "pack_world_state", "unpack_world_state",
    # Shared vocabularies
    "COLOR_RED", "COLOR_YELLOW", "COLOR_GREEN", "COLOR_BLUE", "COLOR_WHITE",
    "COLOR_BLACK", "COLOR_UNKNOWN",
    "TRACK_OK", "TRACK_TOO_SLOW", "TRACK_JUMPED", "TRACK_SETTLING",
    "OUTCOME_PLACED", "OUTCOME_MISSED_GRIP", "OUTCOME_LOST",
    "S6_REFERENCE_FRAME",
]


class ContractError(ValueError):
    """An array is present but does not match its contract.

    Raised only for malformed data -- a genuine bug somewhere upstream. An
    input that has simply not been written yet is reported as ``None`` by the
    ``unpack_*`` functions, because AICA hands out an empty list before the
    first message arrives; that is the normal start-up state, not an error.
    """


# -- Shared vocabularies ----------------------------------------------------

#: Colour codes carried in S1/S3/S10 field 1.
COLOR_RED, COLOR_YELLOW, COLOR_GREEN = 0, 1, 2
COLOR_BLUE, COLOR_WHITE, COLOR_BLACK, COLOR_UNKNOWN = 3, 4, 5, 6

#: Plausibility codes carried in S3/S10 field 9.
TRACK_OK = 0          #: plausible, being followed -- candidate for priority_handler
TRACK_TOO_SLOW = 1    #: stalled or stuck -- excluded
TRACK_JUMPED = 2      #: jump / too fast -- misdetection or knocked -- excluded
TRACK_SETTLING = 3    #: too few measurements yet; normal right after placement

#: Outcome codes carried in S7 field 2.
OUTCOME_PLACED = 0
OUTCOME_MISSED_GRIP = 1
OUTCOME_LOST = 2

#: S6 ``target_pose`` is a ``cartesian_pose``, not an array, so it has no
#: pack/unpack here. Its one binding detail is the frame, set explicitly.
S6_REFERENCE_FRAME = "world"

# S9 (gripper_close, gripper_change, motion_done, has_object) are Bool/Int32
# signals, likewise not arrays and therefore not represented here.


# -- Internal helpers -------------------------------------------------------

def _as_floats(arr: Optional[Sequence[float]]) -> Optional[List[float]]:
    """Return ``arr`` as a list of floats, or ``None`` if nothing arrived yet."""
    if arr is None:
        return None
    values = list(arr)
    if not values:
        return None
    return [float(v) for v in values]


def _entry_count(values: List[float], header: int, stride: int, name: str) -> int:
    """Read ``n`` from the header and verify the array length against it."""
    if len(values) < header:
        raise ContractError(
            f"{name}: array of length {len(values)} is shorter than its "
            f"header ({header})"
        )
    count = values[1]
    if count < 0 or count != int(count):
        raise ContractError(f"{name}: n = {count!r} is not a non-negative integer")
    count = int(count)
    expected = header + count * stride
    if len(values) != expected:
        raise ContractError(
            f"{name}: length {len(values)} does not match n = {count} "
            f"(expected {expected} = {header} + {count}*{stride})"
        )
    return count


def _fixed(values: List[float], length: int, name: str) -> List[float]:
    """Verify a fixed-length signal."""
    if len(values) != length:
        raise ContractError(
            f"{name}: expected exactly {length} fields, got {len(values)}"
        )
    return values


# -- S1 `objects` : base_cam -> vectoring, data_tracker ---------------------

OBJECTS_HEADER = 3
OBJECTS_STRIDE = 9

#: Header indices of S1.
OBJECTS_T, OBJECTS_N, OBJECTS_V_BELT = range(OBJECTS_HEADER)

#: Entry field indices of S1, relative to the start of an entry.
(OBJ_ID, OBJ_COLOR, OBJ_X, OBJ_Y, OBJ_Z,
 OBJ_ORI, OBJ_LEN, OBJ_WID, OBJ_HGT) = range(OBJECTS_STRIDE)


class ObjectEntry(NamedTuple):
    """One detection of ``base_cam``, in world coordinates and SI units."""

    id: float
    color: float
    x: float
    y: float
    z: float
    orientation: float
    length: float
    width: float
    height: float


class ObjectsMsg(NamedTuple):
    t: float
    #: Globally estimated belt speed (m/s). A frame-level quantity, NOT a
    #: per-object one: the tracker assigns it to all tracks at once. See the
    #: three caveats under S1 before using it -- notably the 30 mm/s dead zone.
    v_belt: float
    objects: List[ObjectEntry]


def pack_objects(t: float, v_belt: float,
                 objects: Sequence[ObjectEntry]) -> List[float]:
    out = [float(t), float(len(objects)), float(v_belt)]
    for obj in objects:
        out.extend(float(v) for v in obj)
    return out


def unpack_objects(arr: Optional[Sequence[float]]) -> Optional[ObjectsMsg]:
    values = _as_floats(arr)
    if values is None:
        return None
    count = _entry_count(values, OBJECTS_HEADER, OBJECTS_STRIDE, "objects")
    entries = [
        ObjectEntry(*values[i:i + OBJECTS_STRIDE])
        for i in range(OBJECTS_HEADER, len(values), OBJECTS_STRIDE)
    ]
    assert len(entries) == count
    return ObjectsMsg(values[OBJECTS_T], values[OBJECTS_V_BELT], entries)


# -- S2 `object_position` : robot_cam -> object_follower --------------------

OBJECT_POSITION_LENGTH = 6

(OP_T, OP_VALID, OP_X, OP_Y, OP_Z_BELT, OP_ORI) = range(OBJECT_POSITION_LENGTH)


class ObjectPosition(NamedTuple):
    """Fine localisation by the robot camera, in the CAMERA frame (not world).

    With ``valid = 0`` fields 2-5 are meaningless but ``t`` keeps running --
    that is what separates "camera works, sees nothing" (fall back to w = 0)
    from "camera no longer delivers" (abort).

    The consumer MUST scale x and y by ``(z_belt - block_height)/z_belt``
    before use: they are back-projected with the belt distance while the
    measured point sits on the block's top face. ``orientation`` is diagnostic
    only and deliberately unused by the control path.
    """

    t: float
    valid: float
    x: float
    y: float
    z_belt: float
    orientation: float


def pack_object_position(t: float, valid: bool, x: float = 0.0, y: float = 0.0,
                         z_belt: float = 0.0,
                         orientation: float = 0.0) -> List[float]:
    return [float(t), 1.0 if valid else 0.0,
            float(x), float(y), float(z_belt), float(orientation)]


def unpack_object_position(
        arr: Optional[Sequence[float]]) -> Optional[ObjectPosition]:
    values = _as_floats(arr)
    if values is None:
        return None
    return ObjectPosition(
        *_fixed(values, OBJECT_POSITION_LENGTH, "object_position"))


# -- S3 `tracks` : vectoring -> priority_handler, data_tracker --------------

TRACKS_HEADER = 2
TRACKS_STRIDE = 10

TRACKS_T, TRACKS_N = range(TRACKS_HEADER)

(TRK_ID, TRK_COLOR, TRK_X, TRK_Y, TRK_Z, TRK_ORI,
 TRK_LEN, TRK_WID, TRK_HGT, TRK_STATUS) = range(TRACKS_STRIDE)


class TrackEntry(NamedTuple):
    """A smoothed object state plus its plausibility code.

    Aspect-ratio checks belong on these smoothed values, never on a single
    frame: per frame the ratio of an exactly square block scatters from 0.727
    to 0.999.
    """

    id: float
    color: float
    x: float
    y: float
    z: float
    orientation: float
    length: float
    width: float
    height: float
    status: float


class TracksMsg(NamedTuple):
    t: float
    tracks: List[TrackEntry]


def pack_tracks(t: float, tracks: Sequence[TrackEntry]) -> List[float]:
    out = [float(t), float(len(tracks))]
    for track in tracks:
        out.extend(float(v) for v in track)
    return out


def unpack_tracks(arr: Optional[Sequence[float]]) -> Optional[TracksMsg]:
    values = _as_floats(arr)
    if values is None:
        return None
    _entry_count(values, TRACKS_HEADER, TRACKS_STRIDE, "tracks")
    entries = [
        TrackEntry(*values[i:i + TRACKS_STRIDE])
        for i in range(TRACKS_HEADER, len(values), TRACKS_STRIDE)
    ]
    return TracksMsg(values[TRACKS_T], entries)


# -- S4 `target` : priority_handler -> object_follower ----------------------

TARGET_LENGTH = 13

(TGT_T, TGT_HAS_TARGET, TGT_ID, TGT_COLOR, TGT_X, TGT_Y, TGT_Z, TGT_ORI,
 TGT_LEN, TGT_WID, TGT_HGT, TGT_T_REST, TGT_ZONE_UPSTREAM) = range(TARGET_LENGTH)


class Target(NamedTuple):
    """The selected object, or none.

    With ``has_target = 0`` fields 2-11 are meaningless but ``zone_upstream``
    stays valid. Withdrawing the target IS the abort rule: the follower stops
    when ``priority_handler`` sets ``has_target = 0``.
    """

    t: float
    has_target: float
    id: float
    color: float
    x: float
    y: float
    z: float
    orientation: float
    length: float
    width: float
    height: float
    t_rest: float
    #: Upstream longitudinal bound of the grasp zone. The follower clamps its
    #: target pose to this in ANFAHREN -- and to nothing else.
    zone_upstream: float


def pack_target(t: float, has_target: bool, zone_upstream: float,
                track: Optional[TrackEntry] = None,
                t_rest: float = 0.0) -> List[float]:
    """Build S4 from a chosen :class:`TrackEntry`, or an empty selection."""
    if has_target and track is None:
        raise ContractError("target: has_target = 1 requires a track")
    if track is None:
        body = [0.0] * 9
    else:
        body = [track.id, track.color, track.x, track.y, track.z,
                track.orientation, track.length, track.width, track.height]
    return ([float(t), 1.0 if has_target else 0.0]
            + [float(v) for v in body]
            + [float(t_rest), float(zone_upstream)])


def unpack_target(arr: Optional[Sequence[float]]) -> Optional[Target]:
    values = _as_floats(arr)
    if values is None:
        return None
    return Target(*_fixed(values, TARGET_LENGTH, "target"))


# -- S5 `not_pickable` : priority_handler -> data_tracker -------------------

NOT_PICKABLE_HEADER = 2
NOT_PICKABLE_STRIDE = 1

NOT_PICKABLE_T, NOT_PICKABLE_N = range(NOT_PICKABLE_HEADER)


class NotPickableMsg(NamedTuple):
    t: float
    ids: List[float]


def pack_not_pickable(t: float, ids: Sequence[float]) -> List[float]:
    return [float(t), float(len(ids))] + [float(i) for i in ids]


def unpack_not_pickable(
        arr: Optional[Sequence[float]]) -> Optional[NotPickableMsg]:
    values = _as_floats(arr)
    if values is None:
        return None
    _entry_count(values, NOT_PICKABLE_HEADER, NOT_PICKABLE_STRIDE, "not_pickable")
    return NotPickableMsg(values[NOT_PICKABLE_T], values[NOT_PICKABLE_HEADER:])


# -- S7 `picked_id` : object_follower -> priority_handler, data_tracker -----

PICKED_ID_LENGTH = 3

(PICKED_SEQ, PICKED_ID, PICKED_OUTCOME) = range(PICKED_ID_LENGTH)


class PickedId(NamedTuple):
    """Result of one completed attempt.

    Consumers remember the last ``seq`` they saw and react exactly once -- no
    time window, no edge detection, nothing missed under load.
    """

    seq: float
    id: float
    outcome: float


def pack_picked_id(seq: float, object_id: float, outcome: float) -> List[float]:
    return [float(seq), float(object_id), float(outcome)]


def unpack_picked_id(arr: Optional[Sequence[float]]) -> Optional[PickedId]:
    values = _as_floats(arr)
    if values is None:
        return None
    return PickedId(*_fixed(values, PICKED_ID_LENGTH, "picked_id"))


# -- S8 `follower_status` : object_follower -> interface_streamer -----------

FOLLOWER_STATUS_LENGTH = 7

(FS_T, FS_STATE, FS_TARGET_ID, FS_ERR_LONG,
 FS_ERR_LAT, FS_ERR_Z, FS_W_EFFECTIVE) = range(FOLLOWER_STATUS_LENGTH)


class FollowerStatus(NamedTuple):
    """Diagnostics only. ``w_effective`` shows which source the target
    position currently comes from -- the key value for the base_cam vs
    robot_cam comparison."""

    t: float
    state: float
    target_id: float
    err_long: float
    err_lat: float
    err_z: float
    w_effective: float


def pack_follower_status(t: float, state: float, target_id: float,
                         err_long: float, err_lat: float, err_z: float,
                         w_effective: float) -> List[float]:
    return [float(t), float(state), float(target_id), float(err_long),
            float(err_lat), float(err_z), float(w_effective)]


def unpack_follower_status(
        arr: Optional[Sequence[float]]) -> Optional[FollowerStatus]:
    values = _as_floats(arr)
    if values is None:
        return None
    return FollowerStatus(
        *_fixed(values, FOLLOWER_STATUS_LENGTH, "follower_status"))


# -- S10 `world_state` : data_tracker -> interface_streamer -----------------

WORLD_STATE_HEADER = 2
WORLD_STATE_STRIDE = 12

WORLD_STATE_T, WORLD_STATE_N = range(WORLD_STATE_HEADER)

(WS_ID, WS_COLOR, WS_X, WS_Y, WS_Z, WS_ORI, WS_LEN, WS_WID, WS_HGT,
 WS_STATUS, WS_PICKED, WS_OUT_OF_BOUNDS) = range(WORLD_STATE_STRIDE)


class WorldEntry(NamedTuple):
    """Fields 0-9 as in :class:`TrackEntry`, plus the two bookkeeping flags.

    Entries flagged ``picked`` or ``out_of_bounds`` drop out of the array
    after a configurable grace period -- without that rule the signal grows
    monotonically over a run. History belongs in the log, not in the signal.
    """

    id: float
    color: float
    x: float
    y: float
    z: float
    orientation: float
    length: float
    width: float
    height: float
    status: float
    picked: float
    out_of_bounds: float


class WorldStateMsg(NamedTuple):
    t: float
    entries: List[WorldEntry]


def pack_world_state(t: float, entries: Sequence[WorldEntry]) -> List[float]:
    out = [float(t), float(len(entries))]
    for entry in entries:
        out.extend(float(v) for v in entry)
    return out


def unpack_world_state(
        arr: Optional[Sequence[float]]) -> Optional[WorldStateMsg]:
    values = _as_floats(arr)
    if values is None:
        return None
    _entry_count(values, WORLD_STATE_HEADER, WORLD_STATE_STRIDE, "world_state")
    entries = [
        WorldEntry(*values[i:i + WORLD_STATE_STRIDE])
        for i in range(WORLD_STATE_HEADER, len(values), WORLD_STATE_STRIDE)
    ]
    return WorldStateMsg(values[WORLD_STATE_T], entries)
