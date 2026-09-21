"""Tests for the signal contracts (no ROS, no numpy, no hardware required).

Every array signal is checked for a clean round trip, for the "header only"
case (n = 0, which is valid and means "sees nothing"), for the not-yet-written
input (empty list -> None) and for malformed data (-> ContractError).
"""

from roboter_tetris.contracts import (
    ContractError,
    OBJECTS_HEADER, OBJECTS_STRIDE, ObjectEntry, pack_objects, unpack_objects,
    OBJECT_POSITION_LENGTH, pack_object_position, unpack_object_position,
    TRACKS_HEADER, TRACKS_STRIDE, TrackEntry, pack_tracks, unpack_tracks,
    TARGET_LENGTH, pack_target, unpack_target, along_belt,
    NOT_PICKABLE_HEADER, pack_not_pickable, unpack_not_pickable,
    PICKED_ID_LENGTH, pack_picked_id, unpack_picked_id, AttemptWatcher, PickedId,
    FOLLOWER_STATUS_LENGTH, pack_follower_status, unpack_follower_status,
    WORLD_STATE_HEADER, WORLD_STATE_STRIDE, WorldEntry,
    pack_world_state, unpack_world_state,
    OBJ_ID, OBJ_COLOR, OBJ_X, OBJ_Y, OBJ_Z, OBJ_ORI, OBJ_LEN, OBJ_WID, OBJ_HGT,
    TRK_STATUS, TRK_VX, TRK_V_CHANGE, TRK_ORI_QUALITY,
    TGT_ZONE_UPSTREAM, TGT_VX, TGT_GRASP_PLANE, TGT_ORI_QUALITY, OP_VALID,
    TRACK_FINAL, TRACK_SETTLING,
    OUTCOME_PLACED, OUTCOME_MISSED_GRIP, OUTCOME_LOST, OUTCOME_TOO_LATE,
)
import roboter_tetris.contracts as contracts


def _expect_contract_error(fn, *args):
    """Assert that ``fn(*args)`` rejects the data instead of computing on it."""
    try:
        fn(*args)
    except ContractError:
        return
    raise AssertionError(f"{fn.__name__} accepted malformed data: {args!r}")


def _object(oid=1.0):
    return ObjectEntry(id=oid, color=3.0, x=0.10, y=-0.20, z=0.15,
                       orientation=0.5, length=0.05, width=0.05, height=0.10)


POOL_V = (0.002, -0.101)     # pooled belt velocity (m/s), as in the S3 header


def _track(tid=1.0, status=TRACK_FINAL, vx=0.004, vy=-0.097):
    """A track whose OWN velocity differs from the pool -- on purpose, so tests
    can tell which of the two ended up where."""
    return TrackEntry(id=tid, color=3.0, x=0.10, y=-0.20, z=0.15,
                      orientation=0.5, length=0.05, width=0.05, height=0.10,
                      status=float(status), vx=vx, vy=vy, v_change=0.002,
                      ori_quality=0.93)


def _world(wid=1.0):
    return WorldEntry(*_track(wid), picked=1.0, out_of_bounds=0.0, present=0.0)


# -- S1 objects -------------------------------------------------------------

def test_objects_round_trip():
    objs = [_object(1.0), _object(2.0)]
    arr = pack_objects(12.5, 0.12, objs)
    assert len(arr) == OBJECTS_HEADER + 2 * OBJECTS_STRIDE
    msg = unpack_objects(arr)
    assert msg.t == 12.5
    assert msg.v_belt == 0.12
    assert msg.objects == objs


def test_objects_header_only_is_valid():
    msg = unpack_objects(pack_objects(3.0, 0.0, []))
    assert msg.t == 3.0
    assert msg.objects == []


def test_objects_empty_input_is_none_not_an_error():
    # AICA hands out an empty list before the first message arrives.
    assert unpack_objects([]) is None
    assert unpack_objects(None) is None


def test_objects_truncated_is_rejected():
    arr = pack_objects(1.0, 0.1, [_object()])
    _expect_contract_error(unpack_objects, arr[:-1])


def test_objects_count_mismatch_is_rejected():
    arr = pack_objects(1.0, 0.1, [_object()])
    arr[1] = 2.0  # claims two objects, carries one
    _expect_contract_error(unpack_objects, arr)


def test_objects_non_integral_count_is_rejected():
    arr = pack_objects(1.0, 0.1, [])
    arr[1] = 1.5
    _expect_contract_error(unpack_objects, arr)


def test_object_field_order_matches_the_contract():
    obj = _object()
    body = pack_objects(0.0, 0.0, [obj])[OBJECTS_HEADER:]
    assert body[OBJ_ID] == obj.id
    assert body[OBJ_COLOR] == obj.color
    assert body[OBJ_X] == obj.x
    assert body[OBJ_Y] == obj.y
    assert body[OBJ_Z] == obj.z
    assert body[OBJ_ORI] == obj.orientation
    assert body[OBJ_LEN] == obj.length
    assert body[OBJ_WID] == obj.width
    assert body[OBJ_HGT] == obj.height


# -- S2 object_position -----------------------------------------------------

def test_object_position_round_trip():
    arr = pack_object_position(7.0, True, 0.01, -0.02, 0.54, 1.2)
    assert len(arr) == OBJECT_POSITION_LENGTH
    pos = unpack_object_position(arr)
    assert pos.t == 7.0
    assert pos.valid == 1.0
    assert pos.z_belt == 0.54


def test_object_position_invalid_keeps_the_timestamp_running():
    # The whole point of the valid flag: "works, sees nothing" must stay
    # distinguishable from "no longer delivering".
    pos = unpack_object_position(pack_object_position(9.5, False))
    assert pos.t == 9.5
    assert pos.valid == 0.0


def test_object_position_valid_index_is_field_one():
    arr = pack_object_position(0.0, True)
    assert arr[OP_VALID] == 1.0


def test_object_position_wrong_length_is_rejected():
    _expect_contract_error(unpack_object_position, [1.0, 1.0, 0.0])
    assert unpack_object_position([]) is None


# -- S3 tracks --------------------------------------------------------------

def test_tracks_round_trip_with_the_pooled_header():
    tracks = [_track(1.0), _track(2.0, status=TRACK_SETTLING)]
    arr = pack_tracks(4.0, POOL_V, 3, tracks)
    assert len(arr) == TRACKS_HEADER + 2 * TRACKS_STRIDE
    msg = unpack_tracks(arr)
    assert msg.t == 4.0
    assert (msg.v_belt_x, msg.v_belt_y) == POOL_V
    assert msg.n_pool == 3.0
    assert msg.tracks == tracks
    assert msg.tracks[1].status == float(TRACK_SETTLING)


def test_tracks_field_indices_match_the_contract():
    body = pack_tracks(0.0, POOL_V, 1, [_track()])[TRACKS_HEADER:]
    assert body[TRK_STATUS] == float(TRACK_FINAL)
    assert body[TRK_VX] == 0.004            # the track's own estimate
    assert body[TRK_V_CHANGE] == 0.002
    assert body[TRK_ORI_QUALITY] == 0.93
    assert TRK_ORI_QUALITY == TRACKS_STRIDE - 1


def test_tracks_without_a_pool_say_so_explicitly():
    # n_pool = 0 is a state, not a value: "no belt estimate yet".
    msg = unpack_tracks(pack_tracks(1.0, (0.0, 0.0), 0, []))
    assert msg.tracks == []
    assert msg.n_pool == 0.0


def test_tracks_empty_and_malformed():
    assert unpack_tracks([]) is None
    _expect_contract_error(unpack_tracks, [1.0, 1.0, 0.0, 0.0, 0.0])


def test_retired_status_codes_are_gone():
    # Codes 1 and 2 ("stuck" / "knocked") do not exist -- blocks move freely
    # with the belt. They stay unassigned so they can never change meaning.
    assert TRACK_FINAL == 0 and TRACK_SETTLING == 3
    assert not hasattr(contracts, "TRACK_TOO_SLOW")
    assert not hasattr(contracts, "TRACK_JUMPED")
    assert not hasattr(contracts, "TRACK_OK")


# -- S4 target --------------------------------------------------------------

def test_target_round_trip():
    arr = pack_target(5.0, True, zone_upstream=-0.35, grasp_plane=-0.65,
                      v_belt=POOL_V, track=_track(7.0), t_rest=1.25)
    assert len(arr) == TARGET_LENGTH
    tgt = unpack_target(arr)
    assert tgt.has_target == 1.0
    assert tgt.id == 7.0
    assert tgt.height == 0.10
    assert tgt.t_rest == 1.25
    assert tgt.zone_upstream == -0.35
    assert tgt.grasp_plane == -0.65
    assert tgt.ori_quality == 0.93          # taken from the track


def test_target_velocity_is_the_pool_not_the_track():
    """S4 carries the POOLED belt velocity. The track's own estimate differs
    here on purpose; it must not leak into the contract."""
    tgt = unpack_target(pack_target(0.0, True, -0.35, -0.65, POOL_V,
                                    track=_track()))
    assert (tgt.vx, tgt.vy) == POOL_V
    assert (tgt.vx, tgt.vy) != (0.004, -0.097)


def test_target_without_selection_keeps_both_zone_values_valid():
    # has_target = 0: object fields meaningless, fields 12 and 15 are not.
    tgt = unpack_target(pack_target(6.0, False, zone_upstream=-0.35,
                                    grasp_plane=-0.65, v_belt=(0.0, 0.0)))
    assert tgt.has_target == 0.0
    assert tgt.zone_upstream == -0.35
    assert tgt.grasp_plane == -0.65


def test_target_field_indices_match_the_contract():
    arr = pack_target(0.0, True, -0.35, -0.65, POOL_V, track=_track())
    assert arr[TGT_ZONE_UPSTREAM] == -0.35 and TGT_ZONE_UPSTREAM == 12
    assert arr[TGT_VX] == POOL_V[0] and TGT_VX == 13
    assert arr[TGT_GRASP_PLANE] == -0.65 and TGT_GRASP_PLANE == 15
    assert arr[TGT_ORI_QUALITY] == 0.93 and TGT_ORI_QUALITY == TARGET_LENGTH - 1


def test_target_claiming_a_selection_without_a_track_is_rejected():
    _expect_contract_error(pack_target, 0.0, True, -0.35, -0.65, POOL_V)


def test_target_wrong_length_is_rejected():
    _expect_contract_error(unpack_target, [0.0] * (TARGET_LENGTH - 1))


def test_along_belt_grows_downstream_and_ignores_the_speed():
    # Belt along -y: downstream means smaller y, larger s.
    assert along_belt(0.8, -0.9, (0.0, -0.1)) > along_belt(0.8, -0.5, (0.0, -0.1))
    assert abs(along_belt(0.8, -0.5, (0.0, -0.1)) - 0.5) < 1e-12
    assert abs(along_belt(0.8, -0.5, (0.0, -0.3)) - 0.5) < 1e-12
    # Perpendicular offsets do not change s.
    assert abs(along_belt(0.3, -0.5, (0.0, -0.1)) - 0.5) < 1e-12


def test_along_belt_without_a_direction_is_rejected():
    _expect_contract_error(along_belt, 0.8, -0.5, (0.0, 0.0))


# -- S5 not_pickable --------------------------------------------------------

def test_not_pickable_round_trip():
    arr = pack_not_pickable(2.0, [3.0, 9.0])
    assert len(arr) == NOT_PICKABLE_HEADER + 2
    msg = unpack_not_pickable(arr)
    assert msg.t == 2.0
    assert msg.ids == [3.0, 9.0]


def test_not_pickable_empty_and_malformed():
    assert unpack_not_pickable(pack_not_pickable(2.0, [])).ids == []
    assert unpack_not_pickable([]) is None
    _expect_contract_error(unpack_not_pickable, [2.0, 3.0, 1.0])


# -- S7 picked_id -----------------------------------------------------------

def test_picked_id_round_trip():
    arr = pack_picked_id(4, 11, OUTCOME_MISSED_GRIP)
    assert len(arr) == PICKED_ID_LENGTH
    picked = unpack_picked_id(arr)
    assert picked.seq == 4.0
    assert picked.id == 11.0
    assert picked.outcome == float(OUTCOME_MISSED_GRIP)


def test_outcome_codes_match_the_contract():
    assert (OUTCOME_PLACED, OUTCOME_MISSED_GRIP, OUTCOME_LOST,
            OUTCOME_TOO_LATE) == (0, 1, 2, 3)
    picked = unpack_picked_id(pack_picked_id(1, 5, OUTCOME_TOO_LATE))
    assert picked.outcome == 3.0


def test_picked_id_wrong_length_is_rejected():
    _expect_contract_error(unpack_picked_id, [0.0, 1.0])
    assert unpack_picked_id([]) is None


# -- S8 follower_status -----------------------------------------------------

def test_follower_status_round_trip():
    arr = pack_follower_status(8.0, 3, 11, 0.01, -0.002, 0.03, 0.75)
    assert len(arr) == FOLLOWER_STATUS_LENGTH
    status = unpack_follower_status(arr)
    assert status.state == 3.0
    assert status.target_id == 11.0
    assert status.w_effective == 0.75


def test_follower_status_wrong_length_is_rejected():
    _expect_contract_error(unpack_follower_status, [0.0] * 6)


# -- S10 world_state --------------------------------------------------------

def test_world_state_round_trip_with_the_pooled_header():
    entries = [_world(1.0), _world(2.0)]
    arr = pack_world_state(9.0, POOL_V, 2, entries)
    assert len(arr) == WORLD_STATE_HEADER + 2 * WORLD_STATE_STRIDE
    msg = unpack_world_state(arr)
    assert msg.t == 9.0
    assert (msg.v_belt_x, msg.v_belt_y) == POOL_V
    assert msg.n_pool == 2.0
    assert msg.entries == entries
    assert msg.entries[0].picked == 1.0
    assert msg.entries[0].out_of_bounds == 0.0
    assert msg.entries[0].present == 0.0


def test_world_state_extends_tracks_by_exactly_three_fields():
    assert WORLD_STATE_HEADER == TRACKS_HEADER
    assert WORLD_STATE_STRIDE == TRACKS_STRIDE + 3
    assert tuple(_world())[:TRACKS_STRIDE] == tuple(_track())


def test_world_state_empty_and_malformed():
    assert unpack_world_state(pack_world_state(1.0, (0.0, 0.0), 0, [])).entries == []
    assert unpack_world_state([]) is None
    _expect_contract_error(unpack_world_state, [1.0, 1.0, 0.0, 0.0, 0.0])


# -- S7 consumer rule ---------------------------------------------------------

def test_attempt_watcher_reports_each_seq_once_and_never_zero():
    watcher = AttemptWatcher()
    seen = [watcher.is_new(PickedId(float(seq), 4.0, 0.0))
            for seq in (0, 0, 1, 1, 2, 2)]
    assert seen == [False, False, True, False, True, False]


def test_attempt_watcher_counts_again_after_a_follower_restart():
    watcher = AttemptWatcher()
    assert watcher.is_new(PickedId(1.0, 4.0, 0.0))
    assert not watcher.is_new(PickedId(0.0, 4.0, 0.0))   # reactivated
    assert watcher.is_new(PickedId(1.0, 5.0, 0.0))        # same number, new attempt
