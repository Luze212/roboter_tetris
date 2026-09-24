"""Target selection behind `priority_handler` (project goal 4).

Picks the block the robot grasps next and holds on to it until it is done. No
ROS, no numpy: `priority_handler.py` only wires this to S3/S7 in and S4/S5 out,
so the whole rule set is testable with plain python3.

Positions along the belt are the coordinate ``s`` of
:func:`roboter_tetris.contracts.along_belt`, increasing downstream. Three of
them carry the logic (Nachtrag 6 / Z11)::

    zone_upstream ............ grasp_plane ............ zone_end
    |<---- approach: checked here ---->|<-- descend, grasp, lift -->|

The reachability test covers only the approach; the grasp itself is secured
by where the plane lies. A target may be chosen upstream of the zone -- the
follower then waits at ``zone_upstream`` on the block's lane.
"""

import math
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Sequence, Set, Tuple

from .contracts import (
    TRACK_SELECTABLE, AttemptWatcher, PickedId, TrackEntry, TracksMsg, along_belt,
    pack_not_pickable, pack_target,
)

#: Below this the belt counts as standing: no direction, hence no grasp plane.
MIN_BELT_SPEED_MPS = 0.01


@dataclass
class SelectorParams:
    # Grasp zone in world = the workspace driven by hand (B10/B19): x -1.0 ...
    # -0.53, y -0.32 ... +0.445. The margins of L14 (5 cm / 10 cm in y, x only
    # -0.95 ... -0.68) are gone since 24.09.2026 (Nachtrag 13 / L21): the values
    # driven by hand already hold the safety distance (user).
    zone_x_min: float = -1.0
    zone_x_max: float = -0.53
    zone_y_min: float = -0.32
    zone_y_max: float = 0.445
    #: Binding speed limit of the approach: the lower of attractor and IK
    #: controller (A1). 0.30 since 24.09.2026: the IK controller runs at 0.30
    #: for the belt (setup guide §2, Nachtrag 13 / L18). Throttled runs: set lower.
    attractor_v_max_mps: float = 0.30
    #: Attractor gain K; settling takes about 3/K (setup guide: K ~ 5).
    attractor_gain: float = 5.0
    #: (observe_z - grip height) / descend_speed_mps of the follower, plus
    #: settling (Nachtrag 10 / J2). Was 2.0 for observe_z 0.60; with 0.45 since
    #: L14/L15: 0.11...0.14 m / 0.15 m/s + settling -> 1.2, measured ~1.0 s at
    #: the robot on 24.09.2026 (Nachtrag 13 / L18).
    t_descend_s: float = 1.2
    t_grasp_s: float = 1.0
    t_lift_s: float = 0.5
    grasp_time_margin: float = 1.2
    reach_safety_factor: float = 1.5
    min_graspable_height_m: float = 0.030
    max_gripper_opening_m: float = 0.127
    gripper_margin_m: float = 0.010


class BeltFrame(NamedTuple):
    """The belt of one S3 frame: direction, speed and the three positions."""

    v_belt: Tuple[float, float]
    speed: float
    zone_upstream: float
    grasp_plane: float
    zone_end: float

    @property
    def feasible(self) -> bool:
        """False: the zone is too short for this belt speed, nothing is ever
        graspable -- the grasp plane lies upstream of the zone."""
        return self.grasp_plane > self.zone_upstream

    def s(self, x: float, y: float) -> float:
        return along_belt(x, y, self.v_belt)

    def shift(self, x: float, y: float, ds: float) -> Tuple[float, float]:
        """(x, y) moved by ``ds`` along the belt."""
        return (x + self.v_belt[0] / self.speed * ds,
                y + self.v_belt[1] / self.speed * ds)


def belt_frame(params: SelectorParams, msg: TracksMsg) -> Optional[BeltFrame]:
    """The belt as far as S3 knows it, or None without a usable estimate."""
    if msg.n_pool < 1:
        return None
    v_belt = (msg.v_belt_x, msg.v_belt_y)
    speed = math.hypot(*v_belt)
    if speed < MIN_BELT_SPEED_MPS:
        return None
    corners = sorted(along_belt(x, y, v_belt)
                     for x in (params.zone_x_min, params.zone_x_max)
                     for y in (params.zone_y_min, params.zone_y_max))
    # The two middle corners: for a belt along an axis exactly the zone edges,
    # for a slanted belt the conservative, inner ones.
    zone_upstream, zone_end = corners[1], corners[2]
    t_process = params.t_descend_s + params.t_grasp_s + params.t_lift_s
    grasp_plane = zone_end - speed * t_process * params.grasp_time_margin
    return BeltFrame(v_belt, speed, zone_upstream, grasp_plane, zone_end)


def in_zone(params: SelectorParams, x: float, y: float) -> bool:
    return (params.zone_x_min <= x <= params.zone_x_max
            and params.zone_y_min <= y <= params.zone_y_max)


def fits_gripper(params: SelectorParams, track: TrackEntry) -> bool:
    """The diagonal must fit between the open jaws (Nachtrag 7 / H2).

    Which side ends up between the jaws depends on the follower's yaw, which
    this component does not know. The diagonal is the largest extent in any
    direction, so the check holds for every yaw -- and needs no special case
    for nearly square blocks whose axes may be swapped.
    """
    return (math.hypot(track.length, track.width)
            <= params.max_gripper_opening_m - params.gripper_margin_m)


def tall_enough(params: SelectorParams, track: TrackEntry) -> bool:
    return track.height >= params.min_graspable_height_m


def passes_zone(params: SelectorParams, frame: BeltFrame,
                track: TrackEntry) -> bool:
    """Does the block's lane cross the zone? Checked where it meets the plane."""
    x, y = frame.shift(track.x, track.y,
                       frame.grasp_plane - frame.s(track.x, track.y))
    return in_zone(params, x, y)


def t_available(frame: BeltFrame, track: TrackEntry) -> float:
    """Time until the block reaches the grasp plane; negative once past it."""
    return (frame.grasp_plane - frame.s(track.x, track.y)) / frame.speed


def approach_point(frame: BeltFrame, track: TrackEntry) -> Tuple[float, float]:
    """Where ANFAHREN actually goes: the block, but no further upstream than
    ``zone_upstream`` -- there the robot waits on the block's lane."""
    ds = max(0.0, frame.zone_upstream - frame.s(track.x, track.y))
    return frame.shift(track.x, track.y, ds)


def t_needed(params: SelectorParams, frame: BeltFrame, track: TrackEntry,
             flange_xy: Tuple[float, float]) -> float:
    """Approach time: travel plus attractor settling (3/K).

    Horizontal distance only (Nachtrag 7 / H1): the flange approaches at
    observation height and does not change height before ABSENKEN.
    """
    ax, ay = approach_point(frame, track)
    distance = math.hypot(ax - flange_xy[0], ay - flange_xy[1])
    return distance / params.attractor_v_max_mps + 3.0 / params.attractor_gain


class Selection(NamedTuple):
    target: List[float]         # S4
    not_pickable: List[float]   # S5
    frame: Optional[BeltFrame]
    #: No block at all inside the zone box, whatever its status.
    zone_empty: bool


class TargetSelector:
    """Choose, lock and release the target.

    Once chosen, a target stays chosen. It is withdrawn only when it is done
    (``picked_id``, any outcome), lost from ``tracks``, back to settling, or
    by the component (stale input). Reachability is deliberately NOT a reason:
    a grasp that is nearly done must not be aborted because the block crossed
    the zone end (P4). Whether a grasp may still *start* is decided by the
    follower at the grasp plane.
    """

    def __init__(self, params: Optional[SelectorParams] = None):
        self.params = params or SelectorParams()
        self.reset()

    def reset(self) -> None:
        """A new run: no lock, nothing done, no ``seq`` seen."""
        self.locked_id: Optional[float] = None
        self._done: Set[float] = set()
        self._attempts = AttemptWatcher()
        self._reported: Set[Tuple[float, str]] = set()
        self._events: List[str] = []

    def pop_events(self) -> List[str]:
        """Log lines since the last call: every choice and every withdrawal."""
        events, self._events = self._events, []
        return events

    # -- Withdrawal -------------------------------------------------------------

    def withdraw(self, reason: str) -> None:
        if self.locked_id is not None:
            self._events.append(
                f"Ziel {self.locked_id:.0f} zurückgezogen: {reason}")
            self.locked_id = None

    def on_picked(self, picked: PickedId) -> None:
        """React exactly once per attempt, whatever its outcome."""
        if not self._attempts.is_new(picked):
            return
        self._done.add(picked.id)
        if picked.id == self.locked_id:
            self.withdraw(f"erledigt, outcome {picked.outcome:.0f}")

    # -- One frame --------------------------------------------------------------

    def step(self, msg: TracksMsg,
             flange_xy: Optional[Tuple[float, float]]) -> Selection:
        """Update the lock, choose if free, and build the outputs.

        ``flange_xy`` is the flange position from ``robot_state``; without it
        nothing new is chosen (reachability is unknown), but a lock is kept.
        """
        frame = belt_frame(self.params, msg)
        by_id = {track.id: track for track in msg.tracks}

        if self.locked_id is not None:
            track = by_id.get(self.locked_id)
            if track is None:
                self.withdraw("verloren, ID nicht mehr in tracks")
            elif track.status not in TRACK_SELECTABLE:
                self.withdraw("schwingt neu ein (Status 3)")
            elif frame is None:
                self.withdraw("keine Bandschätzung")

        if (self.locked_id is None and frame is not None and frame.feasible
                and flange_xy is not None):
            self._choose(frame, msg.tracks, flange_xy)

        return self.output(msg, frame)

    def output(self, msg: TracksMsg,
               frame: Optional[BeltFrame] = None) -> Selection:
        """The outputs for the current lock, without choosing anything."""
        if frame is None:
            frame = belt_frame(self.params, msg)
        by_id = {track.id: track for track in msg.tracks}
        locked = by_id.get(self.locked_id) if self.locked_id is not None else None

        if frame is None:
            target = pack_target(msg.t, False, 0.0, 0.0, (0.0, 0.0))
            past_plane: List[float] = []
        else:
            target = pack_target(
                msg.t, locked is not None, frame.zone_upstream,
                frame.grasp_plane, frame.v_belt, locked,
                t_rest=t_available(frame, locked) if locked is not None else 0.0)
            # Not the ones already attempted: a gripped block is no longer seen
            # and vectoring carries it on as predicted (status 4) -- it is not
            # "missed", it is in the bin.
            past_plane = [track.id for track in msg.tracks
                          if track.id != self.locked_id and track.id not in self._done
                          and frame.s(track.x, track.y) > frame.grasp_plane]

        return Selection(
            target=target,
            not_pickable=pack_not_pickable(msg.t, past_plane),
            frame=frame,
            zone_empty=not any(in_zone(self.params, track.x, track.y)
                               for track in msg.tracks),
        )

    # -- Helpers ----------------------------------------------------------------

    def _choose(self, frame: BeltFrame, tracks: Sequence[TrackEntry],
                flange_xy: Tuple[float, float]) -> None:
        """The most urgent candidate: smallest time left to the grasp plane."""
        params = self.params
        best = None
        for track in tracks:
            if track.status not in TRACK_SELECTABLE or track.id in self._done:
                continue
            if not self._graspable(track):
                continue
            if not passes_zone(params, frame, track):
                continue
            available = t_available(frame, track)
            needed = t_needed(params, frame, track, flange_xy)
            if available <= params.reach_safety_factor * needed:
                continue
            if best is None or available < best[0]:
                best = (available, needed, track)
        if best is not None:
            available, needed, track = best
            self.locked_id = track.id
            self._events.append(
                f"Ziel {track.id:.0f} gewählt: {available:.2f} s bis zur "
                f"Greifebene, Anfahrt {needed:.2f} s")

    def _graspable(self, track: TrackEntry) -> bool:
        """Height and width check; each rejection is logged once per block."""
        params = self.params
        if not tall_enough(params, track):
            self._report(track.id, "flach",
                         f"Klotz {track.id:.0f} nicht greifbar: Höhe "
                         f"{track.height * 1000:.0f} mm < "
                         f"{params.min_graspable_height_m * 1000:.0f} mm")
            return False
        if not fits_gripper(params, track):
            limit = params.max_gripper_opening_m - params.gripper_margin_m
            self._report(track.id, "breit",
                         f"Klotz {track.id:.0f} nicht greifbar: Diagonale "
                         f"{math.hypot(track.length, track.width) * 1000:.0f} mm"
                         f" > {limit * 1000:.0f} mm")
            return False
        return True

    def _report(self, track_id: float, kind: str, line: str) -> None:
        if (track_id, kind) not in self._reported:
            self._reported.add((track_id, kind))
            self._events.append(line)
