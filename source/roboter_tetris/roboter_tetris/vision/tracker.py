"""Nearest-neighbor 3D tracker with timestamp-based velocity and prediction.

Port of ``FuE_Greifen-main/cameras/tracker.{hpp,cpp}`` (read-only reference).
Pure Python — no ROS/cv2 imports, fully unit-testable.

Behavioural notes carried over from the C++ original:
- Objects move along the conveyor (y direction) only; prediction shifts y.
- New detections first become *candidates*; they are promoted to tracks (and get
  an id) only when re-detected in a later frame.
- vy is only measured inside the trusted velocity region and when the frame gap
  is sane (0 < dt < 2 s) and the speed is significant (|vy| >= 30 mm/s).
- Unmatched tracks coast with the prediction velocity; deletion is strict inside
  the velocity region (missed frames) and lenient outside (conveyor borders).
- For near-square objects the previous orientation is kept (a square's minAreaRect
  angle flaps, so the first stable estimate wins).
"""

import math
from dataclasses import dataclass, field, replace
from typing import List, Optional


@dataclass
class TrackedObject:
    """Detection/track state. Positions in mm (robot frame), vy in mm/s."""

    id: int = 0
    color: int = 6
    x: float = float("nan")
    y: float = float("nan")
    z: float = float("nan")
    orientation: float = float("nan")
    vy: float = 0.0
    length: float = float("nan")
    width: float = float("nan")
    height: float = float("nan")
    square: bool = False


def _finite_xyz(obj: TrackedObject) -> bool:
    return math.isfinite(obj.x) and math.isfinite(obj.y) and math.isfinite(obj.z)


def _resolve_prediction_vy(obj: TrackedObject, fallback_vy: float) -> float:
    if math.isfinite(obj.vy) and abs(obj.vy) > 1e-6:
        return obj.vy
    return fallback_vy


def _resolve_orientation(detection: TrackedObject, previous: TrackedObject) -> float:
    if detection.square and math.isfinite(previous.orientation):
        return previous.orientation
    return detection.orientation


@dataclass
class _Track:
    object: TrackedObject
    last_ts: float
    missed: int = 0
    age: int = 1


@dataclass
class _Candidate:
    object: TrackedObject
    detected_ts: float


class VisionTracker:
    """Greedy NN tracker; parameter defaults match the C++ header."""

    def __init__(self) -> None:
        self.max_match_distance_mm = 300.0
        self.min_tracked_y_mm = -1080.0
        self.max_tracked_y_mm = 375.0
        self.max_missed_in_region = 3
        self.velocity_region_y_min = -1000.0
        self.velocity_region_y_max = -500.0

        self._tracks: List[_Track] = []
        self._candidates: List[_Candidate] = []
        self._next_id = 1
        self._avg_velocity_y = 0.0
        self._prediction_velocity_y = 0.0

    # -- public API (mirrors the C++ class) -----------------------------------

    def set_prediction_velocity(self, vy_mm_s: float) -> None:
        self._prediction_velocity_y = vy_mm_s

    def set_global_velocity(self, vy_mm_s: float) -> None:
        for track in self._tracks:
            track.object.vy = vy_mm_s
        self._avg_velocity_y = vy_mm_s
        self._prediction_velocity_y = vy_mm_s

    def get_average_velocity(self) -> float:
        return self._avg_velocity_y

    def get_active_objects(self) -> List[TrackedObject]:
        """Active tracks incl. coasting (predicted invisible) ones, as copies."""
        return [replace(track.object) for track in self._tracks]

    def _in_velocity_region(self, y: float) -> bool:
        return self.velocity_region_y_min <= y <= self.velocity_region_y_max

    def update(self, detections: List[TrackedObject], timestamp: float) -> float:
        """Match detections to tracks at ``timestamp`` (s); returns avg y-velocity."""
        n = len(detections)
        matched_track_index = [-1] * n
        original_track_count = len(self._tracks)
        track_used = [False] * original_track_count

        # Greedy nearest-neighbor matching against motion-predicted track positions.
        for i in range(n):
            det = detections[i]
            if not _finite_xyz(det):
                continue
            best_dist = self.max_match_distance_mm + 1.0
            best_j = -1
            for j in range(original_track_count):
                if track_used[j]:
                    continue
                track = self._tracks[j]
                dt = max(timestamp - track.last_ts, 0.0)
                pred_vy = _resolve_prediction_vy(track.object, self._prediction_velocity_y)
                dx = det.x - track.object.x
                dy = det.y - (track.object.y + pred_vy * dt)
                dz = det.z - track.object.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < best_dist:
                    best_dist = dist
                    best_j = j
            if best_j != -1 and best_dist <= self.max_match_distance_mm:
                matched_track_index[i] = best_j
                track_used[best_j] = True

        # Update matched tracks; collect measured y-velocities.
        sum_vy = 0.0
        vel_count = 0
        for i in range(n):
            j = matched_track_index[i]
            if j == -1:
                continue
            track = self._tracks[j]
            det = detections[i]
            det.id = track.object.id
            det.orientation = _resolve_orientation(det, track.object)
            dt = timestamp - track.last_ts
            det_in_region = self._in_velocity_region(det.y)

            if 1e-6 < dt < 2.0 and det_in_region:
                vy = (det.y - track.object.y) / dt
                if abs(vy) >= 30.0:
                    det.vy = vy
                    sum_vy += vy
                    vel_count += 1
                else:
                    det.vy = 0.0
            else:
                det.vy = _resolve_prediction_vy(track.object, self._prediction_velocity_y)

            track.last_ts = timestamp
            track.missed = 0
            track.age += 1

            if not det_in_region and det.vy != 0.0:
                # Outside the trusted region the measured y is unreliable; coast
                # the previous y with the velocity instead (C++ behaviour).
                prev_y = track.object.y
                track.object = replace(det)
                track.object.y = prev_y + det.vy * dt
            else:
                track.object = replace(det)

        # Match remaining detections against candidates (promote on re-detection).
        candidate_matched = [False] * len(self._candidates)
        promoted = [False] * n
        for i in range(n):
            if matched_track_index[i] != -1:
                continue
            det = detections[i]
            if not _finite_xyz(det):
                continue
            best_dist = self.max_match_distance_mm + 1.0
            best_c = -1
            for c, cand in enumerate(self._candidates):
                if candidate_matched[c]:
                    continue
                dx = det.x - cand.object.x
                dy = det.y - cand.object.y
                dz = det.z - cand.object.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < best_dist:
                    best_dist = dist
                    best_c = c
            if best_c != -1 and best_dist <= self.max_match_distance_mm:
                det.id = self._next_id
                self._next_id += 1
                det.vy = 0.0  # new tracks start with zero velocity
                det.orientation = _resolve_orientation(det, self._candidates[best_c].object)
                self._tracks.append(_Track(object=replace(det), last_ts=timestamp))
                candidate_matched[best_c] = True
                promoted[i] = True

        self._candidates = [
            cand for c, cand in enumerate(self._candidates) if not candidate_matched[c]
        ]

        # Still-unmatched detections become new candidates (no id yet).
        for i in range(n):
            det = detections[i]
            if matched_track_index[i] == -1 and not promoted[i] and _finite_xyz(det):
                det.id = 0
                self._candidates.append(_Candidate(object=replace(det), detected_ts=timestamp))
                det.vy = 0.0

        # Coast unmatched tracks with the prediction velocity.
        for j in range(original_track_count):
            if track_used[j]:
                continue
            track = self._tracks[j]
            dt = max(timestamp - track.last_ts, 0.0)
            pred_vy = _resolve_prediction_vy(track.object, self._prediction_velocity_y)
            track.object.y += pred_vy * dt
            track.object.vy = pred_vy
            track.last_ts = timestamp
            track.missed += 1

        # Prune: strict inside the velocity region, lenient (border-based) outside.
        kept: List[_Track] = []
        for track in self._tracks:
            y = track.object.y
            if self.velocity_region_y_min <= y <= self.velocity_region_y_max:
                keep = track.missed <= self.max_missed_in_region
            else:
                keep = self.min_tracked_y_mm <= y <= self.max_tracked_y_mm
            if keep:
                kept.append(track)
        self._tracks = kept

        if vel_count > 0:
            self._avg_velocity_y = sum_vy / vel_count
        return self._avg_velocity_y
