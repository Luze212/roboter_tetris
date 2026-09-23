"""Logic behind `object_follower` -- stages 4a to 4d.

No ROS, no numpy: `object_follower.py` only wires this to its signals. Built in
stages (Umsetzungsplan Phase 4):

* :class:`FollowerCore` -- the state machine. 4a: ``WARTEN`` (the observation
  pose) and ``ABBRUCH`` without a block (straight up, then ``WARTEN``); the
  follower starts in ``ABBRUCH``, because the arm may be anywhere and a
  straight line to the observation pose could cut through the belt. 4b:
  ``ANFAHREN`` and ``FOLGEN`` with the base camera alone -- prediction, lead,
  clamp to ``zone_upstream``, yaw. 4d: ``ABSENKEN``, ``GREIFEN``, ``HEBEN``,
  ``ABLEGEN``, ``LOESEN`` and ``ABBRUCH`` with a block in the gripper.
* :class:`CameraBlend` -- 4c: the robot camera as a filtered correction on
  top of the base camera prediction, ramped in and out, frozen for the grasp.
* :class:`SafetyGate` -- the last function before every output (Thema 7).
* :class:`PoseHistory` -- the flange ring buffer that 4c uses to place a robot
  camera measurement at its image time.

Every pose here is a **flange** pose (``ur_tool0``) in ``world``: the gripper is
not in the URDF, and the IK controller moves the flange.
"""

import math
from collections import deque
from dataclasses import dataclass, fields
from typing import Deque, List, NamedTuple, Optional, Set, Tuple

from .contracts import (
    FOLLOWER_STATES, OUTCOME_ABORTED, OUTCOME_LOST, OUTCOME_MISSED_GRIP,
    OUTCOME_PLACED, OUTCOME_TOO_LATE, STATE_ABORT, STATE_APPROACH,
    STATE_DESCEND, STATE_FOLLOW, STATE_GRASP, STATE_LIFT, STATE_PLACE,
    STATE_RELEASE, STATE_WAIT, ObjectPosition, Target, along_belt,
)


class Pose(NamedTuple):
    """Flange pose in world: position (m) and quaternion (w, x, y, z)."""

    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float

    @property
    def position(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def orientation(self) -> Tuple[float, float, float, float]:
        return (self.qw, self.qx, self.qy, self.qz)


def vertical_orientation(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) of a tool pointing straight down.

    A half turn about the horizontal axis at ``yaw/2``: tool z maps to world
    -z, tool x to (cos yaw, sin yaw, 0). The same form as measured at the setup
    (B8: w = 0, x = cos(yaw/2), y = sin(yaw/2), z = 0).
    """
    return (0.0, math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0), 0.0)


def distance(a: Pose, b: Pose) -> float:
    return math.dist(a.position, b.position)


def yaw_of(orientation: Tuple[float, float, float, float]) -> float:
    """Heading of the tool x axis in the horizontal plane (rad)."""
    w, x, y, z = orientation
    return math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def rotate(q, v) -> Tuple[float, float, float]:
    """Rotate vector ``v`` by the unit quaternion ``q`` = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll) -- the convention of the hand-eye
    result (``Calibration_results_final.yaml``: its rpy reproduce its
    quaternion to five digits this way)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def yaw_error(measured: float, commanded: float) -> float:
    """Yaw difference modulo a half turn (the gripper is symmetric)."""
    return (measured - commanded + math.pi / 2) % math.pi - math.pi / 2


def nearest_equivalent(yaw: float, reference: float) -> float:
    """``yaw + k*pi`` closest to ``reference``.

    The gripper is symmetric under a half turn, so both are the same grasp.
    Taking the one closest to the previous command keeps the wrist from
    turning half a revolution -- and keeps the command continuous when the
    block angle wraps at the edge of [0, pi).
    """
    return yaw + math.pi * round((reference - yaw) / math.pi)


# -- Parameters ---------------------------------------------------------------------

#: No default on purpose (Nachtrag 8 / F3): the workspace is a safety boundary
#: (B10, Safety/README), the observation pose is still open (B8).
REQUIRED = ("ws_x_min", "ws_x_max", "ws_y_min", "ws_y_max", "ws_z_min",
            "ws_z_max", "observe_x", "observe_y", "observe_z", "observe_yaw_deg")


@dataclass
class FollowerParams:
    ws_x_min: Optional[float] = None
    ws_x_max: Optional[float] = None
    ws_y_min: Optional[float] = None
    ws_y_max: Optional[float] = None
    ws_z_min: Optional[float] = None
    ws_z_max: Optional[float] = None
    observe_x: Optional[float] = None
    observe_y: Optional[float] = None
    observe_z: Optional[float] = None
    observe_yaw_deg: Optional[float] = None
    #: Free height of the flange: transfer, and where the abort path rises to.
    #: D12 plus the held block: belt 53.6 + standing block 100 + lower half of
    #: a held block 50 + grip point 235 + air 50 mm -> 0.49 m (Nachtrag 10 / J1).
    transfer_height_m: float = 0.49
    #: Counts as "arrived" for static targets (like the attractor's precision).
    pose_tolerance_m: float = 0.01
    #: Safety gate check 4 (D15, provisional).
    max_target_jump_m: float = 0.05
    # -- 4b: tracking --------------------------------------------------------
    #: Lead as a time: target = prediction + v * lead_time_s. Theory 1/K
    #: (K = 5 -> 0.2 s); calibrated in 4b (B4).
    lead_time_s: float = 0.2
    #: Added to the prediction horizon (D7).
    latency_compensation_s: float = 0.0
    #: Cap of the prediction horizon, gate check 3 (D14). Measured at the
    #: setup: S4 is up to ~0.5 s old before the next one arrives (Nachtrag 13 /
    #: L2). A cap below that froze the prediction and left the flange 10-60 mm
    #: behind the block, unseen by err_along (L1). Was 0.2.
    max_extrapolation_s: float = 0.6
    #: S4 timestamp standing still this long -> abort, gate check 2. Was 0.5,
    #: barely above the measured gap between two base camera measurements.
    target_timeout_s: float = 1.0
    #: ANFAHREN may take until the block reaches the zone, plus this (D5).
    timeout_approach_s: float = 2.0
    #: Time in FOLGEN before the grasp must begin (D5, Thema 6: 2-3 s).
    timeout_track_s: float = 3.0
    #: Mode 2: grip along the block's own angle, if its quality allows.
    use_block_orientation: bool = False
    orientation_quality_min: float = 0.7
    #: Mounting angle of the jaws relative to the flange x axis (D23).
    gripper_yaw_offset_deg: float = 0.0
    # -- 4c: robot camera ----------------------------------------------------
    #: Weight of the robot camera along / across the belt, 0...1 (X/Y of the plan).
    weight_along: float = 0.0
    weight_across: float = 0.0
    #: Fade in / out time of the weights (D8).
    w_ramp_s: float = 0.2
    #: Moving average over this many accepted corrections.
    correction_filter_window: int = 10
    #: Larger correction -> another object in view (R4, D16).
    max_correction_m: float = 0.05
    #: Older measurement -> fall back to w = 0 (D6).
    robot_cam_max_age_s: float = 0.3
    #: Start ABSENKEN only with the robot camera fully blended in (D9/D10).
    require_robot_cam_for_grasp: bool = False
    #: Hand-eye flange -> camera, R = Rz * Ry * Rx. Re-measured 23.09.2026 from
    #: five views of one touched block, residual <= 4.4 mm (Nachtrag 13 / L11).
    #: The previous group's values (C1: 0.1087 / -0.03436 / -0.05987, rpy
    #: 1.66 / 1.56 / 91.50) put the camera 6 cm ABOVE the flange; it sits 7 cm below.
    handeye_x: float = 0.0783
    handeye_y: float = -0.0326
    handeye_z: float = 0.0720
    handeye_roll_deg: float = 4.26
    handeye_pitch_deg: float = 0.08
    handeye_yaw_deg: float = 90.95
    # -- 4d: grasp cycle -----------------------------------------------------
    belt_surface_z_m: float = 0.0536          # B17
    flange_to_grip_point_m: float = 0.235     # Nachtrag 6 / Z7
    #: Lowest grip point above the belt (pad centre). 0.021 since 23.09.2026: the
    #: workspace floor keeps the closed jaw tip 10 mm above the belt (flange
    #: 0.3086), and 53.6 + 21 + 235 = 309.6 mm clears it (Nachtrag 13 / L14).
    #: Pad 11...31 mm: a 30 mm block is still gripped. Was 0.015 (B15).
    min_grip_height_m: float = 0.021
    #: 0.15 m/s: observation height 0.60 -> grip height 0.34 in 1.7 s. Couples
    #: to t_descend_s of the priority_handler (Nachtrag 10 / J2).
    descend_speed_mps: float = 0.15
    #: Rise above the grip height while still moving with the belt.
    lift_clearance_m: float = 0.10
    #: Grasp release in belt coordinates, as err_laengs/err_quer of S8 (D3, B18).
    tol_along_m: float = 0.005
    tol_across_m: float = 0.005
    tol_z_m: float = 0.01
    tol_yaw_rad: float = 0.05
    stable_cycles: int = 10
    timeout_grasp_s: float = 2.0
    timeout_place_s: float = 8.0
    timeout_release_s: float = 2.0
    #: Place pose over the bin (B9, measured 15.09.2026, cross-check pending).
    place_x: float = -0.31649
    place_y: float = 0.47621
    place_z: float = 0.41971
    place_yaw_deg: float = 94.2

    def missing(self) -> List[str]:
        return [name for name in REQUIRED if getattr(self, name) is None]

    def problems(self) -> List[str]:
        """Everything that makes this set unusable; empty means usable."""
        missing = self.missing()
        if missing:
            return [f"nicht gesetzt: {', '.join(missing)}"]
        found = [f"{f.name} ist nicht endlich" for f in fields(self)
                 if not math.isfinite(getattr(self, f.name))]
        if found:
            return found
        for axis in "xyz":
            if getattr(self, f"ws_{axis}_min") >= getattr(self, f"ws_{axis}_max"):
                found.append(f"Arbeitsraum leer in {axis}")
        if found:
            return found
        if self.workspace().clamp(self.observe_pose())[1]:
            found.append("Beobachtungspose liegt außerhalb des Arbeitsraums")
        if not self.ws_z_min <= self.transfer_height_m <= self.ws_z_max:
            found.append("transfer_height_m liegt außerhalb des Arbeitsraums")
        if self.workspace().clamp(self.place_pose())[1]:
            found.append("Ablagepose liegt außerhalb des Arbeitsraums")
        return found

    def workspace(self) -> "Workspace":
        return Workspace(self.ws_x_min, self.ws_x_max, self.ws_y_min,
                         self.ws_y_max, self.ws_z_min, self.ws_z_max)

    def observe_pose(self) -> Pose:
        return Pose(self.observe_x, self.observe_y, self.observe_z,
                    *vertical_orientation(math.radians(self.observe_yaw_deg)))

    def place_pose(self) -> Pose:
        return Pose(self.place_x, self.place_y, self.place_z,
                    *vertical_orientation(math.radians(self.place_yaw_deg)))

    def grip_flange_z(self, block_height: float) -> float:
        """Flange height for the grasp: mid-height of the block, never lower
        than ``min_grip_height_m`` above the belt (Thema 6, B15)."""
        return (self.belt_surface_z_m
                + max(block_height / 2.0, self.min_grip_height_m)
                + self.flange_to_grip_point_m)


class Workspace(NamedTuple):
    """Box the flange target must stay in (B10) -- not the grasp zone."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def clamp(self, pose: Pose) -> Tuple[Pose, bool]:
        x = min(max(pose.x, self.x_min), self.x_max)
        y = min(max(pose.y, self.y_min), self.y_max)
        z = min(max(pose.z, self.z_min), self.z_max)
        clamped = (x, y, z) != pose.position
        return pose._replace(x=x, y=y, z=z), clamped


# -- Safety gate --------------------------------------------------------------------

class GateResult(NamedTuple):
    #: The pose to publish, or None: publish nothing and abort.
    pose: Optional[Pose]
    abort_reason: Optional[str]
    clamped: bool


class SafetyGate:
    """The last function before the output. Nothing bypasses it (Thema 7).

    Checks 1 (finite), 4 (jump) and 5 (workspace) live here. Check 2 (input
    timestamps advance) and 3 (extrapolation cap) belong to the inputs of the
    tracking states and come with 4b.
    """

    def __init__(self, params: FollowerParams):
        self.params = params
        self._last: Optional[Pose] = None

    def reset(self) -> None:
        """At every state or phase change: legitimate jumps happen there."""
        self._last = None

    def check(self, pose: Pose) -> GateResult:
        if not all(math.isfinite(v) for v in pose):
            return GateResult(None, "Zielpose nicht endlich", False)
        if (self._last is not None
                and distance(pose, self._last) > self.params.max_target_jump_m):
            return GateResult(
                None, f"Zielpose springt um {distance(pose, self._last):.3f} m", False)
        # The jump check compares raw targets: comparing against the clamped
        # one would read the clamp itself as a jump.
        self._last = pose
        pose, clamped = self.params.workspace().clamp(pose)
        return GateResult(pose, None, clamped)


# -- Flange history -----------------------------------------------------------------

class PoseHistory:
    """Ring buffer of (t, flange pose), about one second.

    4c places a robot camera measurement at the flange pose of its *image*
    time, not the current one: at 0.2 m/s and 40 ms latency that alone is 8 mm,
    exactly in the fine positioning.
    """

    def __init__(self, max_age_s: float = 1.0):
        self.max_age_s = max_age_s
        self._samples: Deque[Tuple[float, Pose]] = deque()

    def __len__(self) -> int:
        return len(self._samples)

    def clear(self) -> None:
        self._samples.clear()

    def add(self, t: float, pose: Pose) -> None:
        if self._samples and t <= self._samples[-1][0]:
            return                            # out of order or repeated: drop
        self._samples.append((t, pose))
        while self._samples and t - self._samples[0][0] > self.max_age_s:
            self._samples.popleft()

    def at(self, t: float) -> Optional[Pose]:
        """Pose at time ``t``: position linear between neighbours, orientation
        from the nearer one. None outside the stored span -- no extrapolation."""
        if not self._samples or not self._samples[0][0] <= t <= self._samples[-1][0]:
            return None
        previous = self._samples[0]
        for sample in self._samples:
            if sample[0] >= t:
                t0, p0 = previous
                t1, p1 = sample
                if t1 == t0:
                    return p1
                a = (t - t0) / (t1 - t0)
                orientation = p0.orientation if a < 0.5 else p1.orientation
                return Pose(p0.x + a * (p1.x - p0.x), p0.y + a * (p1.y - p0.y),
                            p0.z + a * (p1.z - p0.z), *orientation)
            previous = sample
        return None


# -- State machine ------------------------------------------------------------------

#: Once in FOLGEN this long, the mean along-belt error is checked (lead).
LEAD_CHECK_AFTER_S = 2.0
#: Mean along-belt error above this -> lead_time_s does not fit (B4).
LEAD_WARN_M = 0.01
#: Time constant of that mean (s).
LEAD_MEAN_TAU_S = 0.5
#: ABSENKEN goes back to FOLGEN once the error exceeds this many tolerances.
DESCEND_RETREAT_FACTOR = 2.0
#: has_object must stay false this long before a held block counts as lost.
OBJECT_LOSS_S = 0.1


class TrackingPoint(NamedTuple):
    """Where the block is now and where the flange should go."""

    block_x: float            # predicted block position, with correction
    block_y: float
    target_x: float           # block + lead, clamped if asked
    target_y: float
    capped: bool              # the prediction horizon hit its cap


def tracking_point(target: Target, now: float, params: FollowerParams,
                   clamp_upstream: bool,
                   correction: Tuple[float, float] = (0.0, 0.0)) -> TrackingPoint:
    """Prediction, robot camera correction, lead and -- in ANFAHREN -- the
    clamp to ``zone_upstream``.

    ``p(t) = p + v * (t - t_target + latency)`` with ``t - t_target`` capped to
    ``[0, max_extrapolation_s]`` (Thema 7); plus the correction (4c); then
    ``+ v * lead_time_s``. On a standing target v = 0 and the lead vanishes by
    itself. Only the upstream side is clamped: across stays free (the robot
    waits on the block's lane), downstream stays free (P4).
    """
    age = now - target.t
    horizon = min(max(age, 0.0), params.max_extrapolation_s)
    capped = age != horizon
    dt = horizon + params.latency_compensation_s
    bx = target.x + target.vx * dt + correction[0]
    by = target.y + target.vy * dt + correction[1]
    tx = bx + target.vx * params.lead_time_s
    ty = by + target.vy * params.lead_time_s
    if clamp_upstream:
        v = (target.vx, target.vy)
        short = target.zone_upstream - along_belt(tx, ty, v)
        if short > 0.0:
            speed = math.hypot(*v)
            tx += target.vx / speed * short
            ty += target.vy / speed * short
    return TrackingPoint(bx, by, tx, ty, capped)


def desired_yaw(target: Target, params: FollowerParams) -> float:
    """Block angle in mode 2 if its quality allows, else the belt direction.

    Mode 1 is the special case ``use_block_orientation = False`` -- one code
    path, so mode 2 is not first tested at the very end.
    """
    if (params.use_block_orientation
            and target.ori_quality >= params.orientation_quality_min):
        base = target.orientation
    else:
        base = math.atan2(target.vy, target.vx)
    return base + math.radians(params.gripper_yaw_offset_deg)


def _usable(target: Target) -> bool:
    """Finite, and with a belt velocity -- has_target = 1 guarantees one."""
    return (all(math.isfinite(v) for v in target)
            and math.hypot(target.vx, target.vy) > 0.0)


# -- 4c: robot camera ---------------------------------------------------------------

def camera_to_world(flange: Pose, point_cam: Tuple[float, float, float],
                    params: FollowerParams) -> Tuple[float, float, float]:
    """Camera point -> world, via the flange pose. Direction **flange -> camera**
    (C1): the hand-eye pose is the camera expressed in the flange frame."""
    q_he = quat_from_rpy(math.radians(params.handeye_roll_deg),
                         math.radians(params.handeye_pitch_deg),
                         math.radians(params.handeye_yaw_deg))
    px, py, pz = rotate(q_he, point_cam)
    in_flange = (px + params.handeye_x, py + params.handeye_y, pz + params.handeye_z)
    wx, wy, wz = rotate(flange.orientation, in_flange)
    return (wx + flange.x, wy + flange.y, wz + flange.z)


def robot_cam_world_xy(measurement: ObjectPosition, block_height: float,
                       flange: Pose, params: FollowerParams
                       ) -> Optional[Tuple[float, float]]:
    """S2 -> world (x, y) of the block. The height correction comes FIRST
    (N1): robot_cam back-projects with the belt distance, but the point lies
    on the top face, so x and y are too large by z_band / (z_band - h)."""
    depth = measurement.z_belt - block_height
    if measurement.z_belt <= 0.0 or depth <= 0.0:
        return None
    scale = depth / measurement.z_belt
    wx, wy, _ = camera_to_world(
        flange, (measurement.x * scale, measurement.y * scale, depth), params)
    return (wx, wy)


class CameraBlend:
    """The robot camera as a correction on top of the base camera prediction.

    ``correction = robot_cam_world - prediction`` at the image time, averaged
    over ``correction_filter_window`` accepted measurements -- nearly constant,
    so it can be smoothed; the absolute position could not be without lag.
    Applied per belt axis with ``weight_along`` / ``weight_across``, faded by
    ``ramp`` (0...1) over ``w_ramp_s``. Back to 0 on: ``valid = 0``, a
    measurement older than ``robot_cam_max_age_s``, a correction above
    ``max_correction_m`` (R4: another block), or an image time outside the
    flange history. Frozen from ABSENKEN to GREIFEN (F2).
    """

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self._samples: Deque[Tuple[float, float]] = deque()
        self._last_t: Optional[float] = None
        self._last_ok_t: Optional[float] = None
        self._latest_ok = False
        self.ramp = 0.0
        self._frozen: Optional[Tuple[float, float]] = None
        self.events: List[str] = []
        self._reported: Set[str] = set()

    @property
    def frozen(self) -> bool:
        return self._frozen is not None

    def available(self, now: float, params: FollowerParams) -> bool:
        return (self._latest_ok and self._last_ok_t is not None
                and now - self._last_ok_t <= params.robot_cam_max_age_s)

    def update(self, now: float, dt: float,
               measurement: Optional[ObjectPosition], target: Target,
               history: PoseHistory, params: FollowerParams) -> Tuple[float, float]:
        if self._frozen is not None:
            return self._frozen
        if measurement is not None and measurement.t != self._last_t:
            self._last_t = measurement.t
            self._take(measurement, target, history, params)
        goal = 1.0 if (self.available(now, params)
                       and max(params.weight_along, params.weight_across) > 0.0) else 0.0
        step = dt / params.w_ramp_s if params.w_ramp_s > 0.0 else 1.0
        self.ramp = (min(goal, self.ramp + step) if goal > self.ramp
                     else max(goal, self.ramp - step))
        return self.applied(target, params)

    def applied(self, target: Target, params: FollowerParams) -> Tuple[float, float]:
        if self._frozen is not None:
            return self._frozen
        if not self._samples or self.ramp == 0.0:
            return (0.0, 0.0)
        cx = sum(c[0] for c in self._samples) / len(self._samples)
        cy = sum(c[1] for c in self._samples) / len(self._samples)
        speed = math.hypot(target.vx, target.vy)
        ux, uy = target.vx / speed, target.vy / speed
        along = (cx * ux + cy * uy) * params.weight_along * self.ramp
        across = (-cx * uy + cy * ux) * params.weight_across * self.ramp
        return (along * ux - across * uy, along * uy + across * ux)

    def freeze(self, target: Target, params: FollowerParams) -> None:
        self._frozen = self.applied(target, params)

    def unfreeze(self) -> None:
        self._frozen = None

    def _take(self, m: ObjectPosition, target: Target, history: PoseHistory,
              params: FollowerParams) -> None:
        self._latest_ok = False
        if not m.valid or not all(math.isfinite(v) for v in m):
            return
        flange = history.at(m.t)
        if flange is None:
            self._report("history", "robot_cam: Bildzeit liegt nicht im Ringpuffer "
                         "der Flanschposen - Zeitdomäne der Roboterkamera prüfen")
            return
        point = robot_cam_world_xy(m, target.height, flange, params)
        if point is None:
            return
        age = m.t - target.t
        correction = (point[0] - (target.x + target.vx * age),
                      point[1] - (target.y + target.vy * age))
        if math.hypot(*correction) > params.max_correction_m:
            self._report("identity", "robot_cam: Korrektur über max_correction_m "
                         "verworfen - anderer Klotz im Bild? (R4)")
            return
        self._samples.append(correction)
        while len(self._samples) > max(int(params.correction_filter_window), 1):
            self._samples.popleft()
        self._last_ok_t = m.t
        self._latest_ok = True

    def _report(self, key: str, line: str) -> None:
        if key not in self._reported:
            self._reported.add(key)
            self.events.append(line)


# -- State machine ------------------------------------------------------------------

class GripperFeedback(NamedTuple):
    """S9 from the gripper: motion_done covers closing and opening."""

    motion_done: bool = False
    has_object: bool = False


class FollowerStatus(NamedTuple):
    """Fields 2-6 of S8: errors are 0 in states without a block."""

    target_id: float = 0.0
    err_along: float = 0.0
    err_across: float = 0.0
    err_z: float = 0.0
    w_effective: float = 0.0


class FollowerOutput(NamedTuple):
    #: Flange target after the gate, or None: publish nothing new this step.
    target: Optional[Pose]
    gripper_close: bool
    state: int
    status: FollowerStatus = FollowerStatus()


_TRACKING = (STATE_APPROACH, STATE_FOLLOW, STATE_DESCEND, STATE_GRASP)
_HOLDING = (STATE_LIFT, STATE_PLACE, STATE_RELEASE)


class FollowerCore:
    """The state machine of Thema 6, built in stages 4a to 4d.

    Until the gripper holds the block (``ANFAHREN`` to ``GREIFEN``), a
    withdrawn or changed target, a standing S4, a timeout, a jump or the
    workspace ends the attempt, and so does the grasp plane before descending
    (``outcome = 3``). From ``has_object`` on (``HEBEN`` to ``LOESEN``) the
    block is meant to leave the belt: only a timeout, the gate or a lost block
    ends it -- and an abort with the block in the gripper still places it in
    the bin (Nachtrag 6 / Z12). Every attempt ends with ``picked_id``.
    """

    def __init__(self, params: FollowerParams):
        self.params = params
        self.gate = SafetyGate(params)
        self.history = PoseHistory()
        self.blend = CameraBlend()
        self.reset()

    def reset(self) -> None:
        """Activation: start in ABBRUCH, the rise begins at the first pose."""
        self.state = STATE_ABORT
        self._rise_target: Optional[Pose] = None
        self._hold_pose: Optional[Pose] = None
        self._events: List[str] = []
        self._was_clamped = False
        self._capped_reported = False
        #: (seq, id, outcome) of the last completed attempt; seq 0 = none.
        self.picked: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.has_aborted = False
        self.holding = False
        self.gripper_close = False
        self._attempt_id: Optional[float] = None
        #: IDs whose attempt ended. S4 may still show one for a few cycles
        #: until the priority_handler has read picked_id -- never retake it.
        self._finished: Set[float] = set()
        self._state_since = 0.0
        self._approach_deadline = 0.0
        self._last_target_t: Optional[float] = None
        self._last_target_change = 0.0
        self._last_target: Optional[Target] = None
        self._yaw_ref = 0.0
        self._yaw_frozen: Optional[float] = None
        self._lead_mean = 0.0
        self._lead_warned = False
        self._last_now: Optional[float] = None
        self._stable = 0
        self._descend_deadline = 0.0
        self._grip_z = 0.0
        self._last_cmd: Optional[Pose] = None
        self._saw_motion = False
        self._open_wait_since: Optional[float] = None
        self._abort_with_block = False
        self._place_retry = False
        self._place_stuck_reported = False
        self._object_lost_since: Optional[float] = None
        self._lift_phase = 1
        self._lift_start: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._lift_v: Tuple[float, float] = (0.0, 0.0)
        self.history.clear()
        self.blend.reset()
        self.gate.reset()

    def pop_events(self) -> List[str]:
        events = self._events + self.blend.events
        self._events, self.blend.events = [], []
        return events

    # -- One cycle ------------------------------------------------------------------

    def step(self, flange: Optional[Pose], target: Optional[Target] = None,
             now: float = 0.0, measurement: Optional[ObjectPosition] = None,
             gripper: GripperFeedback = GripperFeedback()) -> FollowerOutput:
        """One cycle.

        ``flange``: current flange pose, None if unknown or stale -- then
        nothing new is published and the attractor holds. ``target``: the
        latest S4. ``measurement``: the latest S2. ``now``: clock time (s), in
        the domain of the S4 and S2 timestamps.
        """
        self.gate.params = self.params
        dt = 0.0 if self._last_now is None else max(now - self._last_now, 0.0)
        self._last_now = now
        if flange is None or not all(math.isfinite(v) for v in flange):
            return FollowerOutput(None, self.gripper_close, self.state)
        if target is not None and target.t != self._last_target_t:
            self._last_target_t = target.t
            self._last_target_change = now

        if (self.state == STATE_WAIT and target is not None and target.has_target
                and target.id not in self._finished and _usable(target)):
            self._start_attempt(target, flange, now)

        if self.state in _TRACKING:
            if self.state == STATE_GRASP and gripper.has_object:
                self._enter_lift(now)
            else:
                problem = self._tracking_problem(target, now)
                if problem is None:
                    return self._track(target, flange, now, dt, measurement, gripper)
                self._fail_attempt(problem[0], problem[1], flange)

        if self.state in _HOLDING:
            output = self._hold(flange, now, gripper)
            if output is not None:
                return output

        if self.state == STATE_ABORT:
            pose = self._abort_step(flange, now, gripper)
        else:
            pose = self.params.observe_pose()
        return self._publish(pose, flange, FollowerStatus(), tracking=False)

    def abort(self, reason: str, flange: Optional[Pose] = None,
              now: Optional[float] = None) -> None:
        """Enter ABBRUCH. With a block in the gripper it stays closed and the
        path ends at the place pose; without, the gripper opens first if the
        follower had closed it, then the flange rises."""
        self._abort_with_block = self.holding
        self._enter(STATE_ABORT, reason, now)
        self._rise_target = None
        self._hold_pose = flange
        if flange is not None:
            self._rise_target = flange._replace(
                z=max(flange.z, self.params.transfer_height_m))
        if not self.holding and self.gripper_close:
            self.gripper_close = False
            self._open_wait_since = self._state_since
            self._saw_motion = False
        else:
            self._open_wait_since = None

    # -- Tracking: ANFAHREN to GREIFEN ------------------------------------------------

    def _start_attempt(self, target: Target, flange: Pose, now: float) -> None:
        self._attempt_id = target.id
        self._yaw_ref = yaw_of(flange.orientation)
        self._yaw_frozen = None
        self._lead_mean = 0.0
        self._lead_warned = False
        self._capped_reported = False
        self._stable = 0
        self._place_retry = False
        self._place_stuck_reported = False
        self.blend.reset()
        point = tracking_point(target, now, self.params, clamp_upstream=False)
        speed = math.hypot(target.vx, target.vy)
        wait = max(0.0, target.zone_upstream
                   - along_belt(point.block_x, point.block_y,
                                (target.vx, target.vy))) / speed
        self._approach_deadline = now + wait + self.params.timeout_approach_s
        self._last_target_change = now
        self._enter(STATE_APPROACH, f"Ziel {target.id:.0f}", now)

    def _tracking_problem(self, target: Optional[Target],
                          now: float) -> Optional[Tuple[str, int]]:
        """Abort reasons while no block is held (Thema 6, Nachtrag 6 / Z12)."""
        if target is None or not target.has_target:
            return "Ziel zurückgezogen", OUTCOME_ABORTED
        if target.id != self._attempt_id:
            return f"Ziel-ID gewechselt auf {target.id:.0f}", OUTCOME_ABORTED
        if not _usable(target):
            return "Zielsatz nicht endlich oder ohne Bandgeschwindigkeit", OUTCOME_ABORTED
        if now - self._last_target_change > self.params.target_timeout_s:
            return "Zielsatz steht still (S4-Zeitstempel)", OUTCOME_ABORTED
        elapsed = now - self._state_since
        if self.state == STATE_APPROACH and now > self._approach_deadline:
            return "Zeitüberschreitung in ANFAHREN", OUTCOME_ABORTED
        if self.state == STATE_FOLLOW and elapsed > self.params.timeout_track_s:
            return "Zeitüberschreitung in FOLGEN", OUTCOME_ABORTED
        if self.state == STATE_DESCEND and now > self._descend_deadline:
            return "Zeitüberschreitung in ABSENKEN", OUTCOME_ABORTED
        if self.state == STATE_GRASP and elapsed > self.params.timeout_grasp_s:
            return "Greifer meldet keinen Klotz (Zeitüberschreitung)", OUTCOME_MISSED_GRIP
        return None

    def _track(self, target: Target, flange: Pose, now: float, dt: float,
               measurement: Optional[ObjectPosition],
               gripper: GripperFeedback) -> FollowerOutput:
        self._last_target = target
        params = self.params
        v = (target.vx, target.vy)
        speed = math.hypot(*v)
        correction = self.blend.update(now, dt, measurement, target,
                                       self.history, params)
        point = tracking_point(target, now, params,
                               clamp_upstream=self.state == STATE_APPROACH,
                               correction=correction)
        if point.capped and not self._capped_reported:
            self._capped_reported = True
            self._events.append(
                f"Vorhersage gedeckelt: S4 ist {now - target.t:.3f} s alt "
                f"(max_extrapolation_s {params.max_extrapolation_s} s)")
        s_block = along_belt(point.block_x, point.block_y, v)

        # The grasp plane: a gate before descending, nothing after (Z11).
        if self.state in (STATE_APPROACH, STATE_FOLLOW) and s_block > target.grasp_plane:
            self._fail_attempt("Greifebene überschritten, bevor abgesenkt wurde",
                               OUTCOME_TOO_LATE, flange)
            return self._publish(self._abort_step(flange, now, gripper), flange,
                                 FollowerStatus(), tracking=False)
        if self.state == STATE_APPROACH and s_block >= target.zone_upstream:
            self._enter(STATE_FOLLOW, "Klotz in der Greifzone", now)

        if self._yaw_frozen is not None:
            yaw = self._yaw_frozen
        else:
            yaw = nearest_equivalent(desired_yaw(target, params), self._yaw_ref)
            self._yaw_ref = yaw

        dx, dy = flange.x - point.block_x, flange.y - point.block_y
        err_along = (dx * v[0] + dy * v[1]) / speed
        err_across = (-dx * v[1] + dy * v[0]) / speed

        if self.state == STATE_FOLLOW:
            self._check_lead(err_along, now, dt)
            z_cmd = params.observe_z
            if self._release_ok(err_along, err_across, flange, yaw, now):
                self._stable += 1
            else:
                self._stable = 0
            if self._stable >= params.stable_cycles:
                self._enter_descend(target, now, yaw)
        if self.state == STATE_DESCEND:
            z_cmd = max(self._grip_z, params.observe_z
                        - params.descend_speed_mps * (now - self._state_since))
            if (abs(err_along) > DESCEND_RETREAT_FACTOR * params.tol_along_m
                    or abs(err_across) > DESCEND_RETREAT_FACTOR * params.tol_across_m):
                self._back_to_follow(now)
                z_cmd = params.observe_z
            elif flange.z <= self._grip_z + params.tol_z_m:
                self._enter(STATE_GRASP, "Greifhöhe erreicht", now)
                self.gripper_close = True
                self._saw_motion = False
        if self.state == STATE_GRASP:
            z_cmd = self._grip_z
            if not gripper.motion_done:
                self._saw_motion = True
            elif self._saw_motion and not gripper.has_object:
                self._fail_attempt("Greifer geschlossen, kein Klotz (Fehlgriff)",
                                   OUTCOME_MISSED_GRIP, flange)
                return self._publish(self._abort_step(flange, now, gripper), flange,
                                     FollowerStatus(), tracking=False)
        if self.state == STATE_APPROACH:
            z_cmd = params.observe_z

        pose = Pose(point.target_x, point.target_y, z_cmd, *vertical_orientation(yaw))
        self._last_cmd = pose
        ramp = self.blend.ramp
        status = FollowerStatus(target.id, err_along, err_across,
                                flange.z - z_cmd, ramp)
        return self._publish(pose, flange, status, tracking=True)

    def _release_ok(self, err_along: float, err_across: float, flange: Pose,
                    yaw: float, now: float) -> bool:
        """Grasp release: four tolerances (F1), relative to the predicted
        block -- the lead keeps the absolute distance to the target non-zero
        on purpose, so the attractor's is_in_range is no use here."""
        params = self.params
        ok = (abs(err_along) <= params.tol_along_m
              and abs(err_across) <= params.tol_across_m
              and abs(flange.z - params.observe_z) <= params.tol_z_m
              and abs(yaw_error(yaw_of(flange.orientation), yaw)) <= params.tol_yaw_rad)
        if ok and params.require_robot_cam_for_grasp:
            ok = self.blend.available(now, params) and self.blend.ramp >= 0.999
        return ok

    def _enter_descend(self, target: Target, now: float, yaw: float) -> None:
        self._grip_z = self.params.grip_flange_z(target.height)
        self._yaw_frozen = yaw
        self.blend.freeze(target, self.params)
        drop = max(self.params.observe_z - self._grip_z, 0.0)
        self._descend_deadline = (now + drop / self.params.descend_speed_mps
                                  + self.params.timeout_grasp_s)
        self._enter(STATE_DESCEND, "Toleranz gehalten, Klotz vor der Greifebene", now)

    def _back_to_follow(self, now: float) -> None:
        """F3: back up to observation height, so the camera is valid again and
        the next attempt starts under the same conditions as the first."""
        self._yaw_frozen = None
        self.blend.unfreeze()
        self._stable = 0
        self._enter(STATE_FOLLOW, "Abweichung wächst beim Absenken", now)

    def _check_lead(self, err_along: float, now: float, dt: float) -> None:
        """Selbstüberwachung des Vorhalts: in steady state the flange sits on
        the predicted block. A lasting offset means lead_time_s no longer fits
        the attractor gain -- the grasp would miss silently."""
        alpha = min(dt / LEAD_MEAN_TAU_S, 1.0)
        self._lead_mean += alpha * (err_along - self._lead_mean)
        if (not self._lead_warned and now - self._state_since >= LEAD_CHECK_AFTER_S
                and abs(self._lead_mean) > LEAD_WARN_M):
            self._lead_warned = True
            side = "vor" if self._lead_mean > 0 else "hinter"
            self._events.append(
                f"Vorhalt passt nicht: Flansch im Mittel "
                f"{abs(self._lead_mean) * 1000:.0f} mm {side} dem Klotz - "
                f"lead_time_s prüfen (B4)")

    def _fail_attempt(self, reason: str, outcome: int, flange: Pose) -> None:
        """End an attempt without a block: picked_id now, then the abort path."""
        seq = self.picked[0] + 1.0
        self.picked = (seq, float(self._attempt_id), float(outcome))
        self.has_aborted = True
        self._finished.add(self._attempt_id)
        self.blend.unfreeze()
        self.abort(reason, flange)

    # -- Holding: HEBEN, ABLEGEN, LOESEN -------------------------------------------------

    def _enter_lift(self, now: float) -> None:
        self.holding = True
        cmd = self._last_cmd
        self._lift_start = (cmd.x, cmd.y, now) if cmd is not None else (0.0, 0.0, now)
        target = self._last_target
        self._lift_v = (target.vx, target.vy) if target is not None else (0.0, 0.0)
        self._lift_phase = 1
        self._object_lost_since = None
        self._enter(STATE_LIFT, "Klotz im Greifer", now)

    def _hold(self, flange: Pose, now: float,
              gripper: GripperFeedback) -> Optional[FollowerOutput]:
        """HEBEN, ABLEGEN, LOESEN. Returns None when the state was left
        towards ABBRUCH, so the caller handles that."""
        params = self.params
        if self.state in (STATE_LIFT, STATE_PLACE):
            if gripper.has_object:
                self._object_lost_since = None
            elif self._object_lost_since is None:
                self._object_lost_since = now
            elif now - self._object_lost_since >= OBJECT_LOSS_S:
                self.holding = False
                self._fail_attempt("Klotz aus dem Greifer verloren", OUTCOME_LOST, flange)
                return None
        yaw = self._yaw_frozen if self._yaw_frozen is not None else yaw_of(flange.orientation)

        if self.state == STATE_LIFT:
            if now - self._state_since > params.timeout_place_s:
                self.abort("Zeitüberschreitung in HEBEN", flange, now)
                return None
            x0, y0, t0 = self._lift_start
            lift_z = self._grip_z + params.lift_clearance_m
            if self._lift_phase == 1:
                # Keep moving with the last belt velocity -- no block prediction
                # any more, the block is in the gripper.
                x = x0 + self._lift_v[0] * (now - t0)
                y = y0 + self._lift_v[1] * (now - t0)
                pose = Pose(x, y, lift_z, *vertical_orientation(yaw))
                if flange.z >= lift_z - params.tol_z_m:
                    self._lift_phase = 2
                    self._lift_start = (x, y, now)
                    self.gate.reset()
            if self._lift_phase == 2:
                x, y, _ = self._lift_start
                pose = Pose(x, y, max(params.transfer_height_m, lift_z),
                            *vertical_orientation(yaw))
                if flange.z >= pose.z - params.pose_tolerance_m:
                    self._enter(STATE_PLACE, "Freihöhe erreicht", now)
            if self.state == STATE_LIFT:
                return self._publish(pose, flange, FollowerStatus(self._attempt_id or 0.0),
                                     tracking=False)

        if self.state == STATE_PLACE:
            place = params.place_pose()
            if distance(flange, place) <= params.pose_tolerance_m:
                self._enter(STATE_RELEASE, "Ablagepose erreicht", now)
                self.gripper_close = False
                self._saw_motion = False
            elif now - self._state_since > params.timeout_place_s:
                if not self._place_retry:
                    self._place_retry = True
                    self.abort("Zeitüberschreitung in ABLEGEN", flange, now)
                    return None
                if not self._place_stuck_reported:
                    self._place_stuck_reported = True
                    self._events.append(
                        "Ablagepose wieder nicht erreicht - der Klotz bleibt im "
                        "Greifer, Eingriff nötig")
            if self.state == STATE_PLACE:
                return self._publish(place, flange, FollowerStatus(), tracking=False)

        if self.state == STATE_RELEASE:
            if not gripper.motion_done:
                self._saw_motion = True
            done = self._saw_motion and gripper.motion_done
            timed_out = now - self._state_since > params.timeout_release_s
            if done or timed_out:
                if timed_out and not done:
                    self._events.append("Öffnen nicht bestätigt (motion_done) - "
                                        "Klotz gilt als abgelegt")
                self._finish_placed(now)
            else:
                return self._publish(params.place_pose(), flange, FollowerStatus(),
                                     tracking=False)
        return None

    def _finish_placed(self, now: float) -> None:
        seq = self.picked[0] + 1.0
        self.picked = (seq, float(self._attempt_id), float(OUTCOME_PLACED))
        self.has_aborted = self._abort_with_block
        self._finished.add(self._attempt_id)
        self.holding = False
        self._abort_with_block = False
        self._yaw_frozen = None
        self.blend.unfreeze()
        self._enter(STATE_WAIT, "Klotz abgelegt", now)

    # -- ABBRUCH ----------------------------------------------------------------------

    def _abort_step(self, flange: Pose, now: float,
                    gripper: GripperFeedback) -> Pose:
        """Open first (if the follower had closed), then straight up; then
        WARTEN -- or ABLEGEN with a block in the gripper."""
        if self._rise_target is None:
            self._hold_pose = flange
            self._rise_target = flange._replace(
                z=max(flange.z, self.params.transfer_height_m))
            self.gate.reset()
        if self._open_wait_since is not None:
            if not gripper.motion_done:
                self._saw_motion = True
            opened = self._saw_motion and gripper.motion_done
            if opened or now - self._open_wait_since > self.params.timeout_release_s:
                self._open_wait_since = None
                self.gate.reset()
            else:
                return self._hold_pose
        if flange.z >= self._rise_target.z - self.params.pose_tolerance_m:
            if self._abort_with_block:
                self._enter(STATE_PLACE, "angehoben, Klotz noch im Greifer", now)
                return self.params.place_pose()
            self._enter(STATE_WAIT, "angehoben", now)
            return self.params.observe_pose()
        return self._rise_target

    # -- Output ---------------------------------------------------------------------

    def _publish(self, pose: Pose, flange: Pose, status: FollowerStatus,
                 tracking: bool) -> FollowerOutput:
        """The gate, last. In a tracking state a clamp is also an abort."""
        result = self.gate.check(pose)
        if result.abort_reason is not None:
            if tracking:
                self._fail_attempt(result.abort_reason, OUTCOME_ABORTED, flange)
            else:
                self.abort(result.abort_reason, flange)
            return FollowerOutput(None, self.gripper_close, self.state)
        if result.clamped and tracking:
            self._fail_attempt("Zielpose außerhalb des Arbeitsraums",
                               OUTCOME_ABORTED, flange)
            return FollowerOutput(result.pose, self.gripper_close, self.state)
        if result.clamped and not self._was_clamped:
            self._events.append(
                f"Zielpose in {FOLLOWER_STATES[self.state]} auf den Arbeitsraum "
                f"gedeckelt: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
        self._was_clamped = result.clamped
        return FollowerOutput(result.pose, self.gripper_close, self.state, status)

    def _enter(self, state: int, reason: str, now: Optional[float] = None) -> None:
        self._events.append(f"{FOLLOWER_STATES[self.state]} -> "
                            f"{FOLLOWER_STATES[state]}: {reason}")
        self.state = state
        self._state_since = self._last_now if now is None else now
        if self._state_since is None:
            self._state_since = 0.0
        self.gate.reset()
        self._was_clamped = False
