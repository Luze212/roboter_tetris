"""Bookkeeping behind `data_tracker` -- the list of every block, for display.

A leaf of the graph: nothing here feeds back into control, so nothing here can
block, delay or corrupt a grasp. No ROS, no numpy: `data_tracker.py` only wires
this to S3/S5/S7 in and S10 out.

One entry per block ID, in order of first appearance. Besides the S3 fields it
carries three facts (Nachtrag 7 / T1):

* ``picked``        -- ``picked_id`` reported ``outcome = 0`` for it
* ``out_of_bounds`` -- it appeared in ``not_pickable``: past the grasp plane
* ``present``       -- it is in the current ``tracks``. A lifted block leaves
  the image seconds before ``picked_id`` arrives, so entries outlive their
  track; ``present = 0`` marks the frozen values.

All times are S3 header times, never the wall clock.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

from .contracts import (
    OUTCOME_PLACED, AttemptWatcher, PickedId, TrackEntry, TracksMsg,
    WorldEntry, pack_world_state,
)


@dataclass
class _Entry:
    track: TrackEntry            # current values, or the last known ones
    present: bool = True
    picked: bool = False
    out_of_bounds: bool = False
    #: S3 time of the last change towards "done"; None while not done.
    done_since: Optional[float] = None

    @property
    def flagged(self) -> bool:
        return self.picked or self.out_of_bounds


class WorldBook:
    """The list behind S10, with the expiry rule that keeps it bounded."""

    def __init__(self, expiry_after_done_s: float = 10.0):
        self.expiry_after_done_s = expiry_after_done_s
        self.reset()

    def reset(self) -> None:
        """A new run: empty list, no ``seq`` seen."""
        self._entries: Dict[float, _Entry] = {}
        #: IDs that expired while still tracked -- kept out until they leave.
        self._expired: Set[float] = set()
        self._attempts = AttemptWatcher()
        self._t: Optional[float] = None
        self._header: Tuple[float, Tuple[float, float], float] = (0.0, (0.0, 0.0), 0.0)

    def __len__(self) -> int:
        return len(self._entries)

    # -- Inputs -----------------------------------------------------------------

    def update_tracks(self, msg: TracksMsg) -> None:
        """Take a new S3 frame: add, refresh, mark as gone, expire."""
        t = msg.t
        self._t = t
        self._header = (t, (msg.v_belt_x, msg.v_belt_y), msg.n_pool)
        seen = set()
        for track in msg.tracks:
            seen.add(track.id)
            if track.id in self._expired:
                continue
            entry = self._entries.get(track.id)
            if entry is None:
                self._entries[track.id] = _Entry(track)
                continue
            entry.track = track
            if not entry.present:            # back before it expired
                entry.present = True
                if not entry.flagged:
                    entry.done_since = None
        for track_id, entry in self._entries.items():
            if entry.present and track_id not in seen:
                entry.present = False
                entry.done_since = t
        self._expired &= seen                # forget IDs that finally left
        self._expire(t)

    def mark_out_of_bounds(self, ids: Sequence[float]) -> None:
        """S5: these blocks crossed the grasp plane. The flag sticks."""
        for track_id in ids:
            entry = self._entries.get(track_id)
            if entry is not None and not entry.out_of_bounds:
                entry.out_of_bounds = True
                entry.done_since = self._t

    def on_picked(self, picked: PickedId) -> None:
        """S7: only ``outcome = 0`` is a pick. A miss or a lost block ends up
        as ``out_of_bounds`` or gone by the other two routes."""
        if not self._attempts.is_new(picked) or picked.outcome != OUTCOME_PLACED:
            return
        entry = self._entries.get(picked.id)
        if entry is not None and not entry.picked:
            entry.picked = True
            entry.done_since = self._t

    # -- Output -----------------------------------------------------------------

    def pack(self) -> List[float]:
        """S10: the S3 header unchanged, then one entry per block."""
        t, v_belt, n_pool = self._header
        return pack_world_state(t, v_belt, n_pool, [
            WorldEntry(*entry.track,
                       picked=float(entry.picked),
                       out_of_bounds=float(entry.out_of_bounds),
                       present=float(entry.present))
            for entry in self._entries.values()
        ])

    # -- Helpers ----------------------------------------------------------------

    def _expire(self, t: float) -> None:
        for track_id in [track_id for track_id, entry in self._entries.items()
                         if entry.done_since is not None
                         and t - entry.done_since >= self.expiry_after_done_s]:
            if self._entries.pop(track_id).present:
                self._expired.add(track_id)
