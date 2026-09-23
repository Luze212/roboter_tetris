"""Velocity estimation and track smoothing -- the computing core of `vectoring`.

This is the procedure behind project goal 3, "Geschwindigkeitsschätzung und
Positionsberechnung der Gegenstände". Binding design:
``docs/architektur/entscheidungen.md`` Nachtrag 6 (Z2-Z4, Z9, Z13) and the
`vectoring` spec.

Pure Python -- no ROS, no numpy -- so it is testable with plain ``python3``.
The `Vectoring` component is only a thin shell around :class:`TrackEstimator`.

The model in one sentence
-------------------------
The belt runs at one constant velocity and the blocks move freely with it, so
every block that has settled measures the *same* velocity: one common slope,
and per block only its own position on the belt.

Per track
---------
1. **Settling.** A freshly placed block may still topple. It becomes *final*
   (status 0) once its velocity is measured as constant: the slopes of the
   older and the newer half window agree within ``settle_v_tolerance``. A
   topple puts a jump into one of the halves, so they disagree until the jump
   has left both. (The first design used the standard error of the slope
   instead; a line through a jump swallows it as extra slope, and a block
   toppling by 25 mm was declared final with a 31 % wrong velocity in every
   simulated case -- Z9.)
2. **"Final" is a status, not a frozen value.** The block is selectable from
   then on; its measurements keep feeding the belt estimate.
3. **Outliers.** A measurement too far from the prediction is dropped; several
   in a row are a real change of position (a topple) and restart the track.

Behind the image
----------------
A **final** track that misses a frame is not forgotten but carried on with the
pooled belt velocity and reported as status 4 (predicted), for at most
``predict_max_s``. The base camera sees only the first ~0.6 m of the belt; the
grasp zone lies behind it (Nachtrag 13 / L4, L9). The prediction is the same
projection a final track always uses -- its clean samples moved to the frame
time with the belt velocity -- just without new samples. ``base_cam`` no longer
carries tracks on itself (its measuring region spans the whole belt), so S1
holds measurements only. Should the camera see the block again under a new ID
(a short occlusion), the predicted track is dropped once a measurement lies
within ``handover_distance_m`` of it. Settling tracks are still forgotten after
``track_expiry_s``: without a settled velocity there is nothing to predict with.

Pool
----
All clean sections of all tracks of one run form a joint least-squares fit:
common slope, one intercept per section. That equals the mean of the section
slopes weighted by their ``S_tt`` -- a long, clean track counts more than a short
one. Compared with averaging frozen snapshots this is 160 to 280 times more
precise over a full pass (Z9).

Only **moving** sections count: a section whose own speed is below
``pool_min_speed_mps`` stays out. Standing blocks -- or the gripper jaws seen
as blocks -- give long, perfectly clean sections with v = 0, and weighted by
``S_tt`` they buried the moving ones: at the setup the pool reported 0 mm/s
while every single block measured -127 mm/s (Nachtrag 13, Block 3). The belt
runs at a fixed ~0.13 m/s, so a floor far below that costs nothing.

Each section keeps five running sums per coordinate. Times and positions are
stored **relative to the section start**: with timestamps around 1.7e9 s the
term ``sum(t^2) - sum(t)^2 / n`` would otherwise cancel to noise.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from .contracts import (
    ObjectEntry, TRACK_FINAL, TRACK_PREDICTED, TRACK_SETTLING, TrackEntry,
    pack_tracks,
)

Velocity = Tuple[float, float]


@dataclass
class EstimatorParams:
    """Tunables; operator-facing descriptions live in the component JSON."""

    #: Measurements per half window of the settling test (0.5 s at 30 Hz).
    settle_half_window: int = 15
    #: How far the two half-window velocities may differ and still count as
    #: "constant" (m/s). D20.
    settle_v_tolerance: float = 0.010
    #: Distance to the prediction beyond which one measurement is dropped (m). D21.
    outlier_distance_m: float = 0.02
    #: That many outliers in a row are a real change of position -> restart. D21.
    outlier_persist_frames: int = 3
    #: Measurements averaged for position, orientation and geometry.
    smoothing_window: int = 30
    #: A SETTLING track not seen for this long is forgotten (its pool share
    #: stays). At ~7 measurements/s, 0.5 s were barely three frames.
    track_expiry_s: float = 1.0
    #: A FINAL track not seen is predicted for at most this long (s): from the
    #: image edge (y ~ 0.55) to the belt end (-0.375) at 0.13 m/s are ~7 s.
    predict_max_s: float = 8.0
    #: A measurement this close to a predicted track (m) ends the prediction:
    #: the camera sees that block again, under a new ID.
    handover_distance_m: float = 0.05
    #: Sections slower than this stay out of the pool (m/s): standing objects.
    #: 0.05 as the previous group's "belt not running" floor; belt ~0.13 m/s.
    pool_min_speed_mps: float = 0.05


@dataclass
class _Sample:
    t: float
    x: float
    y: float
    z: float
    orientation: float
    length: float
    width: float
    height: float

    @classmethod
    def from_object(cls, t: float, obj: ObjectEntry) -> "_Sample":
        return cls(t, obj.x, obj.y, obj.z, obj.orientation,
                   obj.length, obj.width, obj.height)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.t, self.x, self.y, self.z,
                                               self.orientation, self.length,
                                               self.width, self.height))


class _Section:
    """Running sums of one clean section of one track.

    Enough for its own slope and for its share of the pooled slope, with O(1)
    memory. Everything relative to the first sample, see the module docstring.
    """

    def __init__(self, first: _Sample) -> None:
        self._t0, self._x0, self._y0 = first.t, first.x, first.y
        self.n = 0
        self._st = self._stt = 0.0
        self._sx = self._stx = 0.0
        self._sy = self._sty = 0.0

    def add(self, s: _Sample) -> None:
        u, px, py = s.t - self._t0, s.x - self._x0, s.y - self._y0
        self.n += 1
        self._st += u
        self._stt += u * u
        self._sx += px
        self._stx += u * px
        self._sy += py
        self._sty += u * py

    def s_tt(self) -> float:
        return self._stt - self._st * self._st / self.n if self.n else 0.0

    def s_tx(self) -> float:
        return self._stx - self._st * self._sx / self.n if self.n else 0.0

    def s_ty(self) -> float:
        return self._sty - self._st * self._sy / self.n if self.n else 0.0

    def velocity(self) -> Optional[Velocity]:
        stt = self.s_tt()
        if self.n < 2 or stt <= 0.0:
            return None
        return self.s_tx() / stt, self.s_ty() / stt


def _slope(samples: Sequence[_Sample]) -> Velocity:
    """Least-squares velocity through ``samples``, centred on the mean time."""
    n = len(samples)
    tm = sum(s.t for s in samples) / n
    xm = sum(s.x for s in samples) / n
    ym = sum(s.y for s in samples) / n
    den = sum((s.t - tm) ** 2 for s in samples)
    if den <= 0.0:
        return 0.0, 0.0
    return (sum((s.t - tm) * (s.x - xm) for s in samples) / den,
            sum((s.t - tm) * (s.y - ym) for s in samples) / den)


def mean_orientation(angles: Sequence[float]) -> Tuple[float, float]:
    """Average of pi-periodic angles over the doubled angle.

    Returns ``(mean in [0, pi), quality 0..1)``. The quality is the resultant
    length: 1 for identical angles, near 0 for angles spread evenly. An
    arithmetic mean of 0 and 90 degrees would be 45 -- exactly where the gripper
    catches the corners.
    """
    if not angles:
        return 0.0, 0.0
    s = sum(math.sin(2.0 * a) for a in angles) / len(angles)
    c = sum(math.cos(2.0 * a) for a in angles) / len(angles)
    return (0.5 * math.atan2(s, c)) % math.pi, math.hypot(s, c)


class _Track:
    def __init__(self, tid: float, color: float, maxlen: int) -> None:
        self.id = tid
        self.color = color
        self.status = TRACK_SETTLING
        self.samples: Deque[_Sample] = deque(maxlen=maxlen)
        self.pending: List[_Sample] = []      # consecutive outliers
        self.section: Optional[_Section] = None
        self.clean_start_t: Optional[float] = None
        self.v_change = 0.0
        self.last_seen: Optional[float] = None


class TrackEstimator:
    """Per-object velocity, settling, pooled belt velocity and smoothing.

    Feed it every new S1 frame with :meth:`update`; read the S3 state with
    :meth:`snapshot` or directly as a packed array with :meth:`pack`. One
    instance is one run: the pool starts empty.
    """

    def __init__(self, params: Optional[EstimatorParams] = None) -> None:
        self.params = params or EstimatorParams()
        self._tracks: Dict[float, _Track] = {}
        self._closed_sections: List[_Section] = []
        #: Time of the latest S1 frame: "not seen" means "not in that frame".
        self._frame_t: Optional[float] = None

    # -- Pool -----------------------------------------------------------------

    def _sections(self) -> List[_Section]:
        """Sections that feed the pool: at least two samples, and moving."""
        open_ = [tr.section for tr in self._tracks.values()
                 if tr.section is not None]
        floor = self.params.pool_min_speed_mps
        pooled = []
        for s in self._closed_sections + open_:
            v = s.velocity()
            if v is not None and math.hypot(*v) >= floor:
                pooled.append(s)
        return pooled

    def belt_velocity(self) -> Tuple[Optional[Velocity], int]:
        """Pooled belt velocity and the number of sections behind it.

        ``(None, 0)`` before the first block has settled -- a state, not a
        value: consumers must not use a belt velocity then.
        """
        sections = self._sections()
        stt = sum(s.s_tt() for s in sections)
        if not sections or stt <= 0.0:
            return None, 0
        return ((sum(s.s_tx() for s in sections) / stt,
                 sum(s.s_ty() for s in sections) / stt), len(sections))

    # -- Input ----------------------------------------------------------------

    def update(self, t: float, objects: Sequence[ObjectEntry]) -> None:
        """Take one S1 frame (header time ``t``, objects in SI units)."""
        p = self.params
        self._frame_t = t
        maxlen = max(2 * p.settle_half_window, p.smoothing_window)
        v_belt, _ = self.belt_velocity()
        for obj in objects:
            sample = _Sample.from_object(t, obj)
            if not sample.is_finite():
                continue                      # drop the measurement, keep the track
            tr = self._tracks.get(obj.id)
            if tr is None:
                tr = self._tracks[obj.id] = _Track(obj.id, obj.color, maxlen)
            elif tr.samples.maxlen != maxlen:
                # A window was changed live; without resizing, a track whose
                # buffer is shorter than two half windows would never settle.
                tr.samples = deque(tr.samples, maxlen=maxlen)
            tr.last_seen = t
            self._take(tr, sample, v_belt)
        self._hand_over(t, v_belt)
        self._expire(t)

    def _hand_over(self, t: float, v_belt: Optional[Velocity]) -> None:
        """Drop predicted tracks that a measurement of this frame lies on."""
        if v_belt is None:
            return
        measured = [tr.samples[-1] for tr in self._tracks.values()
                    if tr.last_seen == t and tr.samples]
        if not measured:
            return
        limit = self.params.handover_distance_m
        for tid in [tid for tid, tr in self._tracks.items()
                    if self._is_predicted(tr, t, v_belt)]:
            p = self._predict(self._tracks[tid], t, v_belt)
            if p is not None and any(math.hypot(s.x - p[0], s.y - p[1]) <= limit
                                     for s in measured):
                self._drop(tid)

    @staticmethod
    def _is_predicted(tr: "_Track", t: float, v_belt: Optional[Velocity]) -> bool:
        return (tr.status == TRACK_FINAL and v_belt is not None
                and tr.last_seen is not None and tr.last_seen < t)

    def _drop(self, tid: float) -> None:
        tr = self._tracks.pop(tid)
        if tr.section is not None:
            self._closed_sections.append(tr.section)

    def _predict(self, tr: _Track, t: float,
                 v_belt: Optional[Velocity]) -> Optional[Tuple[float, float]]:
        if not tr.samples:
            return None
        last = tr.samples[-1]
        if v_belt is not None:
            v = v_belt
        elif len(tr.samples) >= 3:
            v = _slope(tr.samples)
        else:
            return None                       # too little to judge an outlier
        return last.x + v[0] * (t - last.t), last.y + v[1] * (t - last.t)

    def _take(self, tr: _Track, s: _Sample, v_belt: Optional[Velocity]) -> None:
        p = self.params
        pred = self._predict(tr, s.t, v_belt)
        if pred is not None and math.hypot(s.x - pred[0], s.y - pred[1]) \
                > p.outlier_distance_m:
            tr.pending.append(s)
            if len(tr.pending) >= p.outlier_persist_frames:
                self._restart(tr)             # a real change: the block toppled
            return
        tr.pending.clear()
        tr.samples.append(s)
        if tr.section is not None:
            tr.section.add(s)
        self._update_settling(tr)

    def _restart(self, tr: _Track) -> None:
        """Start over with the deviating measurements.

        A clean section is closed and **stays in the pool**: it was measured
        while the block ran evenly.
        """
        if tr.section is not None:
            self._closed_sections.append(tr.section)
        tr.section = None
        tr.clean_start_t = None
        tr.status = TRACK_SETTLING
        tr.samples.clear()
        tr.samples.extend(tr.pending)
        tr.pending.clear()
        tr.v_change = 0.0

    def _update_settling(self, tr: _Track) -> None:
        h = self.params.settle_half_window
        if len(tr.samples) < 2 * h:
            return
        window = list(tr.samples)[-2 * h:]
        v_old, v_new = _slope(window[:h]), _slope(window[h:])
        tr.v_change = math.hypot(v_new[0] - v_old[0], v_new[1] - v_old[1])
        if tr.status == TRACK_SETTLING \
                and tr.v_change <= self.params.settle_v_tolerance:
            tr.status = TRACK_FINAL
            tr.clean_start_t = window[0].t
            tr.section = _Section(window[0])
            for s in window:
                tr.section.add(s)

    def _expire(self, t: float) -> None:
        """Settling tracks after ``track_expiry_s``; final ones are predicted
        up to ``predict_max_s`` -- if there is a belt velocity to predict with."""
        p = self.params
        v_belt, _ = self.belt_velocity()
        for tid in list(self._tracks):
            tr = self._tracks[tid]
            unseen = t - tr.last_seen
            predictable = tr.status == TRACK_FINAL and v_belt is not None
            if unseen > (p.predict_max_s if predictable else p.track_expiry_s):
                self._drop(tid)

    # -- Output ---------------------------------------------------------------

    def _entry(self, tr: _Track, t: float,
               v_belt: Optional[Velocity]) -> TrackEntry:
        last = tr.samples[-1] if tr.samples else tr.pending[-1]
        if tr.status != TRACK_FINAL or v_belt is None:
            v = _slope(tr.samples) if len(tr.samples) >= 2 else (0.0, 0.0)
            return TrackEntry(
                id=tr.id, color=tr.color, x=last.x, y=last.y, z=last.z,
                orientation=last.orientation, length=last.length,
                width=last.width, height=last.height, status=float(tr.status),
                vx=v[0], vy=v[1], v_change=tr.v_change, ori_quality=0.0)

        # Final: average only the clean section, projected to t with the belt.
        # Seen in this frame: final. Not seen: predicted -- the same projection,
        # carried on without new samples.
        frame_t = t if self._frame_t is None else self._frame_t
        status = TRACK_PREDICTED if tr.last_seen < frame_t else TRACK_FINAL
        clean = [s for s in tr.samples if s.t >= tr.clean_start_t]
        clean = clean[-self.params.smoothing_window:]
        n = len(clean)
        x = sum(s.x + v_belt[0] * (t - s.t) for s in clean) / n
        y = sum(s.y + v_belt[1] * (t - s.t) for s in clean) / n
        ori, quality = mean_orientation([s.orientation for s in clean])
        own = tr.section.velocity() or (0.0, 0.0)
        return TrackEntry(
            id=tr.id, color=tr.color, x=x, y=y,
            z=sum(s.z for s in clean) / n, orientation=ori,
            length=sum(s.length for s in clean) / n,
            width=sum(s.width for s in clean) / n,
            height=sum(s.height for s in clean) / n,
            status=float(status), vx=own[0], vy=own[1],
            v_change=tr.v_change, ori_quality=quality)

    def snapshot(self, t: float) -> Tuple[Optional[Velocity], int, List[TrackEntry]]:
        """Belt velocity, pool size and the tracks, valid for time ``t``."""
        v_belt, n_pool = self.belt_velocity()
        entries = [self._entry(tr, t, v_belt)
                   for _, tr in sorted(self._tracks.items())
                   if tr.samples or tr.pending]
        return v_belt, n_pool, entries

    def pack(self, t: float) -> List[float]:
        """The full S3 array for time ``t``."""
        v_belt, n_pool, entries = self.snapshot(t)
        return pack_tracks(t, v_belt or (0.0, 0.0), n_pool, entries)
