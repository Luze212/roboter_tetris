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
interprets: extrapolation and frame transforms belong to the consumer. Two
exceptions define contract semantics that every side must share:
:func:`along_belt` (the coordinate of S4 fields 12 and 15) and
:class:`AttemptWatcher` (the S7 rule "react once per seq, never to 0").
"""

import math
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
    "TARGET_LENGTH", "Target", "pack_target", "unpack_target", "along_belt",
    # S5
    "NOT_PICKABLE_HEADER", "NOT_PICKABLE_STRIDE", "NotPickableMsg",
    "pack_not_pickable", "unpack_not_pickable",
    # S7
    "PICKED_ID_LENGTH", "PickedId", "pack_picked_id", "unpack_picked_id",
    "AttemptWatcher",
    # S8
    "FOLLOWER_STATUS_LENGTH", "FollowerStatus",
    "pack_follower_status", "unpack_follower_status",
    # S10
    "WORLD_STATE_HEADER", "WORLD_STATE_STRIDE", "WorldEntry", "WorldStateMsg",
    "pack_world_state", "unpack_world_state",
    # Shared vocabularies
    "COLOR_RED", "COLOR_YELLOW", "COLOR_GREEN", "COLOR_BLUE", "COLOR_WHITE",
    "COLOR_BLACK", "COLOR_UNKNOWN",
    "TRACK_FINAL", "TRACK_SETTLING", "TRACK_PREDICTED", "TRACK_SELECTABLE",
    "OUTCOME_PLACED", "OUTCOME_MISSED_GRIP", "OUTCOME_LOST", "OUTCOME_TOO_LATE",
    "OUTCOME_ABORTED",
    "FOLLOWER_STATES",
    "STATE_WAIT", "STATE_APPROACH", "STATE_FOLLOW", "STATE_DESCEND",
    "STATE_GRASP", "STATE_LIFT", "STATE_PLACE", "STATE_RELEASE", "STATE_ABORT",
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

#: Status codes carried in S3/S10 field 9.
#:
#: Codes 1 and 2 ("stuck" / "knocked") are RETIRED and must never be reused:
#: blocks move freely with the belt, so neither case exists. Leaving the numbers
#: unassigned means an old note about "status 1" can never mean something else.
TRACK_FINAL = 0       #: velocity measured as constant; selectable, feeds the pool
TRACK_SETTLING = 3    #: just placed, may still be toppling; not yet selectable
#: Final, but no longer measured (left the base camera image, or missed a frame):
#: the position is carried on with the pooled belt velocity. Selectable -- the
#: grasp zone lies behind the image (entscheidungen.md Nachtrag 13 / L4, L10).
TRACK_PREDICTED = 4
#: Statuses a consumer may choose a target from.
TRACK_SELECTABLE = (TRACK_FINAL, TRACK_PREDICTED)

#: Outcome codes carried in S7 field 2.
OUTCOME_PLACED = 0       #: block is in the bin (also after an abort while holding it)
OUTCOME_MISSED_GRIP = 1  #: gripper closed without an object
OUTCOME_LOST = 2         #: object vanished, or dropped from the gripper
OUTCOME_TOO_LATE = 3     #: block crossed the grasp plane before descending began
OUTCOME_ABORTED = 4      #: attempt aborted before gripping; the reason is logged

# S8 field 1: follower state codes, in the order of the state machine.
STATE_WAIT = 0       #: WARTEN   -- observation pose
STATE_APPROACH = 1   #: ANFAHREN
STATE_FOLLOW = 2     #: FOLGEN
STATE_DESCEND = 3    #: ABSENKEN
STATE_GRASP = 4      #: GREIFEN
STATE_LIFT = 5       #: HEBEN
STATE_PLACE = 6      #: ABLEGEN
STATE_RELEASE = 7    #: LOESEN
STATE_ABORT = 8      #: ABBRUCH  -- also the start state
#: Code -> the German state name used throughout the documentation.
FOLLOWER_STATES = {
    STATE_WAIT: "WARTEN", STATE_APPROACH: "ANFAHREN", STATE_FOLLOW: "FOLGEN",
    STATE_DESCEND: "ABSENKEN", STATE_GRASP: "GREIFEN", STATE_LIFT: "HEBEN",
    STATE_PLACE: "ABLEGEN", STATE_RELEASE: "LOESEN", STATE_ABORT: "ABBRUCH",
}

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


# -- S2 `object_position` : robot_cam -> (no consumer since 24.09.2026, L22) -

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

TRACKS_HEADER = 5
TRACKS_STRIDE = 14

#: Header indices of S3. The belt velocity is the pooled estimate over every
#: block that settled in the current run; ``n_pool = 0`` means "no estimate yet"
#: and the velocity must not be used -- it is a state, not a value.
(TRACKS_T, TRACKS_N, TRACKS_V_BELT_X, TRACKS_V_BELT_Y,
 TRACKS_N_POOL) = range(TRACKS_HEADER)

(TRK_ID, TRK_COLOR, TRK_X, TRK_Y, TRK_Z, TRK_ORI,
 TRK_LEN, TRK_WID, TRK_HGT, TRK_STATUS,
 TRK_VX, TRK_VY, TRK_V_CHANGE, TRK_ORI_QUALITY) = range(TRACKS_STRIDE)


class TrackEntry(NamedTuple):
    """A smoothed object state, its settling status and its own velocity.

    Position, geometry and orientation are averaged only once the block is
    final (status 0) -- a block that toppled on placement must not mix its
    standing and lying shape. Aspect-ratio checks belong on these smoothed
    values, never on a single frame: per frame the ratio of an exactly square
    block scatters from 0.727 to 0.999.
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
    #: This block's OWN velocity estimate -- diagnostics and display. Control
    #: uses the pooled belt velocity from the header instead.
    vx: float
    vy: float
    #: |v_new - v_old| between the two half windows; decides status 3 -> 0.
    v_change: float
    #: Orientation quality 0..1; judged by the consumer, not here.
    ori_quality: float


class TracksMsg(NamedTuple):
    t: float
    v_belt_x: float
    v_belt_y: float
    #: Number of settled blocks behind the belt estimate; 0 = no estimate yet.
    n_pool: float
    tracks: List[TrackEntry]


def pack_tracks(t: float, v_belt: Sequence[float], n_pool: int,
                tracks: Sequence[TrackEntry]) -> List[float]:
    out = [float(t), float(len(tracks)),
           float(v_belt[0]), float(v_belt[1]), float(n_pool)]
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
    return TracksMsg(values[TRACKS_T], values[TRACKS_V_BELT_X],
                     values[TRACKS_V_BELT_Y], values[TRACKS_N_POOL], entries)


# -- S4 `target` : priority_handler -> object_follower ----------------------

TARGET_LENGTH = 17

(TGT_T, TGT_HAS_TARGET, TGT_ID, TGT_COLOR, TGT_X, TGT_Y, TGT_Z, TGT_ORI,
 TGT_LEN, TGT_WID, TGT_HGT, TGT_T_REST, TGT_ZONE_UPSTREAM,
 TGT_VX, TGT_VY, TGT_GRASP_PLANE, TGT_ORI_QUALITY) = range(TARGET_LENGTH)


class Target(NamedTuple):
    """The selected object, or none.

    With ``has_target = 0`` the object fields are meaningless, but
    ``zone_upstream`` and ``grasp_plane`` stay valid. Withdrawing the target is
    the abort rule -- until the gripper holds the block; from ``has_object`` on
    the follower owns it and a withdrawn target no longer aborts anything.
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
    #: Time until the block reaches the grasp plane -- the deadline by which
    #: descending must have started.
    t_rest: float
    #: Upstream longitudinal bound of the grasp zone. The follower clamps its
    #: target pose to this in ANFAHREN -- and to nothing else.
    zone_upstream: float
    #: POOLED belt velocity, not the track's own estimate.
    vx: float
    vy: float
    #: Last longitudinal position from which the whole grasp still finishes
    #: before the zone end. A gate for starting ABSENKEN, not a pose clamp.
    grasp_plane: float
    #: Orientation quality of the target; the follower judges it.
    ori_quality: float


def pack_target(t: float, has_target: bool, zone_upstream: float,
                grasp_plane: float, v_belt: Sequence[float],
                track: Optional[TrackEntry] = None,
                t_rest: float = 0.0) -> List[float]:
    """Build S4 from a chosen :class:`TrackEntry`, or an empty selection.

    ``v_belt`` is passed separately on purpose: S4 carries the POOLED belt
    velocity from the S3 header, not the track's own estimate. Copying it from
    the track would silently put the wrong value into the contract.
    """
    if has_target and track is None:
        raise ContractError("target: has_target = 1 requires a track")
    if track is None:
        body = [0.0] * 9
        quality = 0.0
    else:
        body = [track.id, track.color, track.x, track.y, track.z,
                track.orientation, track.length, track.width, track.height]
        quality = track.ori_quality
    return ([float(t), 1.0 if has_target else 0.0]
            + [float(v) for v in body]
            + [float(t_rest), float(zone_upstream),
               float(v_belt[0]), float(v_belt[1]),
               float(grasp_plane), float(quality)])


def unpack_target(arr: Optional[Sequence[float]]) -> Optional[Target]:
    values = _as_floats(arr)
    if values is None:
        return None
    return Target(*_fixed(values, TARGET_LENGTH, "target"))


def along_belt(x: float, y: float, v_belt: Sequence[float]) -> float:
    """Longitudinal coordinate ``s`` of the point (x, y) in ``world``.

    The coordinate of S4 fields 12 and 15: the projection onto the belt
    direction taken from ``v_belt`` (S4 fields 13/14 of the same message),
    origin at the world origin, increasing downstream. ``zone_upstream`` and
    ``grasp_plane`` mean nothing without it -- which is why both sides call
    this one function instead of each writing the dot product.
    """
    vx, vy = float(v_belt[0]), float(v_belt[1])
    speed = math.hypot(vx, vy)
    if speed == 0.0:
        raise ContractError("along_belt: v_belt = 0 gives no belt direction")
    return (float(x) * vx + float(y) * vy) / speed


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


class AttemptWatcher:
    """The S7 consumer rule: report each completed attempt exactly once.

    ``seq = 0`` means the follower was (re)activated and has no attempt yet --
    it is remembered but never reported. Remembering it matters: after a
    reactivation the follower counts from 1 again, and those numbers must count
    as new even if they were seen before.
    """

    def __init__(self) -> None:
        self._last_seq: Optional[float] = None

    def reset(self) -> None:
        self._last_seq = None

    def is_new(self, picked: PickedId) -> bool:
        if picked.seq == self._last_seq:
            return False
        self._last_seq = picked.seq
        return picked.seq != 0.0


# -- S8 `follower_status` : object_follower -> interface_streamer -----------

#: 6 since 24.09.2026: the 7th field ``w_effective`` (weight of the robot
#: camera) went with stage 4c (Nachtrag 13 / L22).
FOLLOWER_STATUS_LENGTH = 6

(FS_T, FS_STATE, FS_TARGET_ID, FS_ERR_LONG,
 FS_ERR_LAT, FS_ERR_Z) = range(FOLLOWER_STATUS_LENGTH)


class FollowerStatus(NamedTuple):
    """Diagnostics only."""

    t: float
    state: float
    target_id: float
    err_long: float
    err_lat: float
    err_z: float


def pack_follower_status(t: float, state: float, target_id: float,
                         err_long: float, err_lat: float, err_z: float) -> List[float]:
    return [float(t), float(state), float(target_id), float(err_long),
            float(err_lat), float(err_z)]


def unpack_follower_status(
        arr: Optional[Sequence[float]]) -> Optional[FollowerStatus]:
    values = _as_floats(arr)
    if values is None:
        return None
    return FollowerStatus(
        *_fixed(values, FOLLOWER_STATUS_LENGTH, "follower_status"))


# -- S10 `world_state` : data_tracker -> interface_streamer -----------------

WORLD_STATE_HEADER = 5
WORLD_STATE_STRIDE = 17

#: Header as in S3 -- passed through so the display can show the belt estimate.
(WORLD_STATE_T, WORLD_STATE_N, WORLD_STATE_V_BELT_X, WORLD_STATE_V_BELT_Y,
 WORLD_STATE_N_POOL) = range(WORLD_STATE_HEADER)

(WS_ID, WS_COLOR, WS_X, WS_Y, WS_Z, WS_ORI, WS_LEN, WS_WID, WS_HGT,
 WS_STATUS, WS_VX, WS_VY, WS_V_CHANGE, WS_ORI_QUALITY,
 WS_PICKED, WS_OUT_OF_BOUNDS, WS_PRESENT) = range(WORLD_STATE_STRIDE)


class WorldEntry(NamedTuple):
    """Fields 0-13 as in :class:`TrackEntry`, plus three bookkeeping fields.

    An entry is done once it is ``picked``, ``out_of_bounds`` or no longer
    ``present``; it drops out of the array a configurable time after the last
    of these changes -- without that rule the signal grows monotonically over
    a run. History belongs in the log, not in the signal.
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
    vx: float
    vy: float
    v_change: float
    ori_quality: float
    picked: float
    #: The block crossed the grasp plane without being gripped.
    out_of_bounds: float
    #: 1 = in the current tracks; 0 = gone, fields 0-13 are the last known
    #: values (the block is in the gripper, or lost).
    present: float


class WorldStateMsg(NamedTuple):
    t: float
    v_belt_x: float
    v_belt_y: float
    n_pool: float
    entries: List[WorldEntry]


def pack_world_state(t: float, v_belt: Sequence[float], n_pool: int,
                     entries: Sequence[WorldEntry]) -> List[float]:
    out = [float(t), float(len(entries)),
           float(v_belt[0]), float(v_belt[1]), float(n_pool)]
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
    return WorldStateMsg(values[WORLD_STATE_T], values[WORLD_STATE_V_BELT_X],
                         values[WORLD_STATE_V_BELT_Y], values[WORLD_STATE_N_POOL],
                         entries)
