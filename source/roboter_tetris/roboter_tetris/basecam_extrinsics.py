"""Extrinsic calibration of the base camera: camera -> ``world``, without ROS.

Replaces the transitional calibration of Nachtrag 13 / L6 (``cal_*`` of
``base_cam``) by a measured one that can be repeated at any time. Only the base
camera is calibrated; the robot camera is not part of the chain (L22), and the
calibration project under ``Calibration/`` stays untouched.

Stage 1 -- robot moves (rare, about two minutes). The gripper holds the AprilGrid
board flat, tool axis horizontal, so the printed face looks up into the static
base camera. The robot drives a pose plan around a start pose driven by hand.
Two transforms are unknown and solved together (eye-to-hand)::

    cam_T_board_i = inv(world_T_cam) @ world_T_flange_i @ flange_T_board

``world_T_cam`` is what ``base_cam`` needs; ``flange_T_board`` (how the board
sits in the jaws) comes for free and never has to be measured. Start values come
from ``cv2.calibrateHandEye``; a Levenberg-Marquardt refinement then minimises the
pixel error of every detected tag corner over all poses. Held-out poses give the
error in mm at points the solution has not seen.

Stage 2 -- no robot motion (seconds). Single AprilTags fixed to the conveyor
frame beside the belt are located in ``world`` during stage 1. From then on one
camera image of them is enough to solve the camera pose again, and the same
comparison serves as the start-up check: has the camera moved?

Conventions shared with ``vision.detection.build_cam_to_robot``: camera = color
optical frame of the L515 (aligned depth lives in the same pixel grid), angles in
degrees with R = Rz(yaw) @ Ry(pitch) @ Rx(roll), translations in meters.
Transforms are 4x4 numpy arrays named ``a_T_b`` (maps points from b into a).
"""

import datetime
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

#: Tag detections of one image: tag id -> 4x2 pixel corners in OpenCV order
#: (top-left, top-right, bottom-right, bottom-left of the upright tag).
Detections = Dict[int, np.ndarray]

SCHEMA = "roboter_tetris.base_cam_extrinsics"
SCHEMA_VERSION = 1

#: The calibration file shipped with the package, relative to this directory.
#: A new calibration is copied here from the container and built in (all data
#: of the project lives in the repository).
DEFAULT_CALIBRATION_FILE = "Extrinsics/base_cam_extrinsics.json"
PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

#: Transitional calibration of Nachtrag 13 / L6 (the cal_* defaults of base_cam),
#: in force whenever no valid file is.
L6_CAL = (-0.7787, 0.7934, 0.9163, 179.46, 0.45, 179.76)


# ---------------------------------------------------------------------------
# Transforms
# ---------------------------------------------------------------------------

def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    return make_transform(R.T, -R.T @ T[:3, 3])


def rotvec_to_matrix(rotvec: Sequence[float]) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64).reshape(3, 1))
    return R


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    return rvec.reshape(3)


def rotation_angle_deg(R: np.ndarray) -> float:
    """Angle of a rotation matrix, 0 ... 180 degrees."""
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def orthonormalize(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    R_o = U @ Vt
    if np.linalg.det(R_o) < 0:
        U[:, -1] *= -1
        R_o = U @ Vt
    return R_o


def average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Mean of transforms that lie close together (rotations projected to SO(3))."""
    R = orthonormalize(np.mean([T[:3, :3] for T in transforms], axis=0))
    return make_transform(R, np.mean([T[:3, 3] for T in transforms], axis=0))


def axis_rotation(axis: Sequence[float], angle_deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    return rotvec_to_matrix(a / np.linalg.norm(a) * math.radians(angle_deg))


def pose_from_quaternion(position: Sequence[float], wxyz: Sequence[float]) -> np.ndarray:
    """4x4 from a position and a quaternion (w, x, y, z) as state_representation gives it."""
    w, x, y, z = (float(v) for v in wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("Quaternion der Länge 0")
    w, x, y, z = w / n, x / n, y / n, z / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return make_transform(R, position)


def quaternion_from_matrix(R: np.ndarray) -> Tuple[float, float, float, float]:
    """(w, x, y, z) of a rotation matrix, w >= 0."""
    angle = math.radians(rotation_angle_deg(R))
    rvec = matrix_to_rotvec(R)
    norm = np.linalg.norm(rvec)
    axis = rvec / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0])
    q = (math.cos(angle / 2), *(axis * math.sin(angle / 2)))
    return tuple(float(v) for v in q)


def matrix_from_cal(x: float, y: float, z: float,
                    roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """``world_T_cam`` from the six ``cal_*`` values of ``base_cam``."""
    r, p, yv = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(yv), -math.sin(yv), 0], [math.sin(yv), math.cos(yv), 0], [0, 0, 1]])
    return make_transform(rz @ ry @ rx, (x, y, z))


def cal_from_matrix(T: np.ndarray) -> Dict[str, float]:
    """The six ``cal_*`` values (m, degrees) of ``world_T_cam``."""
    R = T[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock, never near the setup (camera looks straight down)
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return {
        "cal_x": float(T[0, 3]), "cal_y": float(T[1, 3]), "cal_z": float(T[2, 3]),
        "cal_roll": math.degrees(roll), "cal_pitch": math.degrees(pitch),
        "cal_yaw": math.degrees(yaw),
    }


# ---------------------------------------------------------------------------
# AprilGrid board and tag detection
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GridSpec:
    """AprilGrid in the Kalibr layout (calib.io, family 36h11).

    Board frame: x along the columns, y along the rows, z = x cross y out of the
    printed face. ``origin`` names the board corner at tag ``first_id``; ids run
    along a row first. ``tag_size_m`` is the outer edge of the black tag border.
    ``tag_rotation_deg`` turns every tag against the grid: calib.io prints the
    tags upright in the AprilTag library's sense, which OpenCV reads turned by
    180 degrees -- measured on the board at the setup, 25.09.2026 (45 px error
    with 0, 1.2 px with 180). Which layout a printed board has is checked on the
    first image with :func:`identify_layout`.
    """

    rows: int = 7
    cols: int = 11
    tag_size_m: float = 0.020
    tag_spacing_m: float = 0.006
    first_id: int = 0
    origin: str = "bottom_left"  # or "top_left"
    tag_rotation_deg: int = 180

    @property
    def pitch_m(self) -> float:
        return self.tag_size_m + self.tag_spacing_m

    @property
    def size_m(self) -> Tuple[float, float]:
        """Extent of the tag field (x, y)."""
        return (self.cols * self.pitch_m - self.tag_spacing_m,
                self.rows * self.pitch_m - self.tag_spacing_m)

    def tag_corners(self, tag_id: int) -> Optional[np.ndarray]:
        """4x3 board coordinates of a tag's corners in OpenCV order, or None."""
        k = tag_id - self.first_id
        if not 0 <= k < self.rows * self.cols:
            return None
        row, col = divmod(k, self.cols)
        if self.origin == "top_left":
            row = self.rows - 1 - row
        elif self.origin != "bottom_left":
            raise ValueError(f"unbekannter Ursprung {self.origin!r}")
        if self.tag_rotation_deg % 90:
            raise ValueError(f"Tag-Drehung {self.tag_rotation_deg} kein Vielfaches von 90")
        x0, y0, s = col * self.pitch_m, row * self.pitch_m, self.tag_size_m
        upright = np.array([[x0, y0 + s, 0.0], [x0 + s, y0 + s, 0.0],
                            [x0 + s, y0, 0.0], [x0, y0, 0.0]])
        # detected corner k lies where the upright model has corner k + turns
        return np.roll(upright, -(self.tag_rotation_deg // 90), axis=0)

    def to_dict(self) -> dict:
        return {"rows": self.rows, "cols": self.cols, "tag_size_m": self.tag_size_m,
                "tag_spacing_m": self.tag_spacing_m, "first_id": self.first_id,
                "origin": self.origin, "tag_rotation_deg": self.tag_rotation_deg}


def make_tag_detector(dictionary: str = "DICT_APRILTAG_36h11", sharpen: float = 2.0):
    """Callable gray image -> :data:`Detections`, for OpenCV 4.6 and 4.7+.

    Tuned on the base camera at the setup (25.09.2026): a 20 mm tag at 0.55 m
    is about 30 px, 3.7 px per bit, and the L515 colour image is soft. Plain
    settings read 1 of 77 tags. An unsharp mask (``sharpen``) and a finer
    bit sampling read 57, without a false id. The corners found here are only
    coarse (about 1 px inside, some snapped onto the AprilGrid's corner squares);
    :func:`refine_grid_corners` and :func:`refine_tag_corners` set them exactly.
    """
    if not hasattr(cv2.aruco, dictionary):
        raise ValueError(f"unbekanntes Tag-Wörterbuch {dictionary!r}")
    tag_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    # 4.6 needs the factory (its bare constructor crashes), 4.7+ dropped it
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    params.perspectiveRemovePixelPerCell = 14
    params.perspectiveRemoveIgnoredMarginPerCell = 0.35
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(tag_dict, params)
        detect = detector.detectMarkers
    else:
        def detect(gray):
            return cv2.aruco.detectMarkers(gray, tag_dict, parameters=params)

    def run(gray: np.ndarray) -> Detections:
        if sharpen > 0:
            blur = cv2.GaussianBlur(gray, (0, 0), 1.0)
            gray = cv2.addWeighted(gray, 1.0 + sharpen, blur, -sharpen, 0)
        corners, ids, _ = detect(gray)
        if ids is None:
            return {}
        return {int(i): np.asarray(c, dtype=np.float64).reshape(4, 2)
                for i, c in zip(ids.ravel(), corners)}

    return run


_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.005)


def refine_grid_corners(gray: np.ndarray, spec: GridSpec, detections: Detections,
                        K: np.ndarray, D: np.ndarray, max_move_px: float = 1.5,
                        coarse_limit_px: float = 5.0) -> Tuple[Detections, float]:
    """Set the corners of the grid tags exactly; returns them and the pixel RMS.

    A coarse board pose from all detected corners (outliers dropped) predicts
    every tag corner to about a pixel; ``cornerSubPix`` in the original image
    then settles on the corner, where tag and AprilGrid square meet. A tag
    whose corner moves more than ``max_move_px`` drops out. Not the grid (or a
    wrong layout): the detections come back unchanged with RMS inf.
    Measured at the setup: 2.15 px -> 0.16 px over 228 corners.
    """
    board = {i: c for i, c in detections.items() if spec.tag_corners(i) is not None}
    obj, img = grid_points(spec, board)
    if len(board) < 4:
        return detections, float("inf")
    keep = np.ones(len(obj), dtype=bool)
    try:
        for _ in range(3):
            T, _ = solve_pnp(obj[keep], img[keep], K, D)
            err = reprojection_errors(obj, img, T, K, D)
            keep = err < max(2.5, 2.0 * float(np.median(err)))
            if keep.sum() < 16:
                return detections, float("inf")
    except (ValueError, cv2.error):
        return detections, float("inf")
    if rms(err[keep]) > coarse_limit_px:
        return detections, float("inf")
    rvec = matrix_to_rotvec(T[:3, :3])
    guess, _ = cv2.projectPoints(obj, rvec, T[:3, 3], K, D)
    refined = guess.astype(np.float32).copy()
    cv2.cornerSubPix(gray, refined, (3, 3), (-1, -1), _SUBPIX)
    refined = refined.reshape(-1, 4, 2).astype(np.float64)
    moved = np.linalg.norm(refined - guess.reshape(-1, 4, 2), axis=2)
    out = {i: c for i, c in detections.items() if i not in board}
    for tag_id, corners, shift in zip(sorted(board), refined, moved):
        if np.all(shift <= max_move_px):
            out[tag_id] = corners
    obj2, img2 = grid_points(spec, {i: c for i, c in out.items() if i in board})
    if len(obj2) < 16:
        return detections, float("inf")
    T2, err2 = solve_pnp(obj2, img2, K, D)
    return out, err2


def refine_tag_corners(gray: np.ndarray, detections: Detections,
                       max_move_px: float = 2.0) -> Detections:
    """Exact corners of single tags (reference tags): outer corners of a black
    square on white, where ``cornerSubPix`` converges from the coarse corner."""
    out = {}
    for tag_id, corners in detections.items():
        c = corners.astype(np.float32).reshape(-1, 1, 2).copy()
        cv2.cornerSubPix(gray, c, (3, 3), (-1, -1), _SUBPIX)
        c = c.reshape(4, 2).astype(np.float64)
        if np.all(np.linalg.norm(c - corners, axis=1) <= max_move_px):
            out[tag_id] = c
    return out


def average_detections(frames: Sequence[Detections], min_fraction: float = 0.8) -> Detections:
    """Mean corners over frames of a standing scene; tags seen too rarely drop out."""
    if not frames:
        return {}
    seen: Dict[int, List[np.ndarray]] = {}
    for frame in frames:
        for tag_id, corners in frame.items():
            seen.setdefault(tag_id, []).append(corners)
    need = max(1, math.ceil(min_fraction * len(frames)))
    return {tag_id: np.mean(c, axis=0) for tag_id, c in seen.items() if len(c) >= need}


def grid_points(spec: GridSpec, detections: Detections) -> Tuple[np.ndarray, np.ndarray]:
    """Matching board points (Nx3) and pixels (Nx2) of the grid tags in view."""
    obj, img = [], []
    for tag_id in sorted(detections):
        corners = spec.tag_corners(tag_id)
        if corners is not None:
            obj.append(corners)
            img.append(detections[tag_id])
    if not obj:
        return np.zeros((0, 3)), np.zeros((0, 2))
    return np.vstack(obj), np.vstack(img)


# ---------------------------------------------------------------------------
# Single-view pose
# ---------------------------------------------------------------------------

def reprojection_errors(obj: np.ndarray, img: np.ndarray, cam_T_obj: np.ndarray,
                        K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Pixel distance per point."""
    rvec = matrix_to_rotvec(cam_T_obj[:3, :3])
    proj, _ = cv2.projectPoints(obj, rvec, cam_T_obj[:3, 3], K, D)
    return np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else float("nan")


def solve_pnp(obj: np.ndarray, img: np.ndarray, K: np.ndarray, D: np.ndarray,
              guess: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float]:
    """``cam_T_obj`` and its pixel RMS. Planar ``obj`` (z = 0) uses IPPE."""
    obj = np.ascontiguousarray(obj, dtype=np.float64)
    img = np.ascontiguousarray(img, dtype=np.float64)
    if len(obj) < 4:
        raise ValueError(f"zu wenige Punkte für die Pose: {len(obj)}")
    if guess is not None:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, matrix_to_rotvec(guess[:3, :3]).copy(),
                                      guess[:3, 3].copy(), True, cv2.SOLVEPNP_ITERATIVE)
    elif np.allclose(obj[:, 2], 0.0):
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
    else:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError("Pose nicht lösbar")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
    T = make_transform(rotvec_to_matrix(rvec), tvec)
    return T, rms(reprojection_errors(obj, img, T, K, D))


def identify_layout(detections: Detections, candidates: Sequence[GridSpec],
                    K: np.ndarray, D: np.ndarray) -> List[Tuple[float, GridSpec]]:
    """Pixel RMS of each candidate layout, best first. Only a right layout fits."""
    ranked = []
    for spec in candidates:
        obj, img = grid_points(spec, detections)
        if len(obj) < 8 or len(obj) < 4 * len(detections):
            continue  # ids outside this layout: cannot be it
        try:
            _, err = solve_pnp(obj, img, K, D)
        except (ValueError, cv2.error):
            continue
        ranked.append((err, spec))
    return sorted(ranked, key=lambda r: r[0])


def layout_candidates(spec: GridSpec) -> List[GridSpec]:
    """Every way the ids can lie: rows/cols swapped, both origins, four tag turns."""
    out = []
    for rows, cols in ((spec.rows, spec.cols), (spec.cols, spec.rows)):
        for origin in ("bottom_left", "top_left"):
            for turn in (spec.tag_rotation_deg, *(r for r in (0, 90, 180, 270)
                                                  if r != spec.tag_rotation_deg)):
                out.append(GridSpec(rows, cols, spec.tag_size_m, spec.tag_spacing_m,
                                    spec.first_id, origin, turn))
    return out


# ---------------------------------------------------------------------------
# Stage 1: pose plan
# ---------------------------------------------------------------------------

@dataclass
class PlanParams:
    #: Board centre in the flange frame, along the tool axis: flange -> jaws
    #: (0.235 m, Z7) plus about half the board held out beyond them.
    pivot_along_tool_m: float = 0.37
    #: Kept small on purpose: every turn about the board centre swings the
    #: flange too, and with ±35 degrees and 15 cm it wandered 42 cm and came
    #: near the reach limit (setup, 25.09.2026). 8 cm with ±20 degrees keep it
    #: within 24 cm of the start.
    max_shift_m: float = 0.08
    height_step_m: float = 0.08
    #: Tilt about the tool axis (roll) and across it (pitch). Off by default:
    #: the board hangs in the jaws by a few degrees and moves in the grip as
    #: soon as gravity pulls from another side -- 0.5 degrees in the first run
    #: at the setup (25.09.2026), and the grip may not be changed. Turning about
    #: the vertical and shifting keep gravity where it is. Without tilt the
    #: camera height is not observable; it comes from the belt in the depth
    #: image instead (``camera_z_m`` of :func:`solve_stage1`). Simulated: 0.4 mm
    #: position on the belt, height within about 2 mm.
    max_tilt_deg: float = 0.0
    max_pitch_deg: float = 0.0
    max_yaw_deg: float = 20.0
    #: Passes over the pose pattern; each further pass is mirrored and turns the
    #: tilt axes by 90 degrees. The robot reports its pose with a small error per
    #: pose, not the camera its pixels -- that error dominates, and only more and
    #: wider spread poses average it out. Synthetic, 0.1 mm / 0.02 deg per pose,
    #: start at the lowest allowed height: at most 0.6 mm on the belt (ROI) with
    #: 40 poses; 20 poses give about twice that.
    passes: int = 2
    #: Belt surface in world (B17) and the free distance the lowest board edge
    #: must keep, with the board seen as a disc of ``board_radius_m``.
    belt_z_m: float = 0.0536
    min_clearance_m: float = 0.10
    board_radius_m: float = 0.17
    #: Wrist and gripper as a cylinder along the tool axis, flange to jaw tips
    #: (245 mm, CLAUDE.md §4).
    gripper_length_m: float = 0.245
    gripper_radius_m: float = 0.06
    #: Lowest flange z in world (0 = off). A pose that would take the flange
    #: lower is lifted to it, turns kept -- they carry the accuracy. At the setup
    #: the wrist posture allows about 6 cm below the start pose (25.09.2026).
    min_flange_z_m: float = 0.0
    #: Largest horizontal distance of the flange from the start pose.
    max_flange_offset_m: float = 0.25


# Each entry: shift (x, y, z in world) in units of max_shift / max_shift /
# height_step; tilt 0...1 about a horizontal axis at the given azimuth from the
# tool direction (0 = roll about the tool axis, scaled by max_tilt; 90 = pitch
# across it, scaled by max_pitch); yaw in units of max_yaw. Chosen for rotation
# diversity about several axes -- what the hand-eye solution needs, and what the
# old orbit lacked.
_CALIBRATION_POSES = (
    ((0, 0, 0), 0.0, 0, 0.0),
    ((1, 0, 0), 1.0, 0, 0.0),
    ((-1, 0, 0), 1.0, 180, 0.0),
    ((0, 1, 0), 1.0, 90, 0.0),
    ((0, -1, 0), 1.0, 270, 0.0),
    ((1, 1, 1), 0.5, 45, 1.0),
    ((-1, -1, 1), 0.5, 225, -1.0),
    ((1, -1, 1), 1.0, 315, 0.5),
    ((-1, 1, 1), 1.0, 135, -0.5),
    ((0, 0, 1), 0.0, 0, 1.0),
    ((0, 0, 1), 0.0, 0, -1.0),
    ((0.5, 0, -1), 1.0, 90, 1.0),
    ((-0.5, 0, -1), 1.0, 270, -1.0),
    ((0, 0.5, -1), 1.0, 0, -1.0),
    ((0, -0.5, -1), 1.0, 180, 1.0),
    ((1, 0, 1), 1.0, 200, -1.0),
    ((-1, 0, 1), 1.0, 20, 1.0),
    ((0, 1, -0.5), 1.0, 110, 0.5),
    ((0, -1, -0.5), 1.0, 290, -0.5),
    ((0, 0, 0), 1.0, 60, 1.0),
)
_HOLDOUT_POSES = (
    ((0.5, 0.5, 0.5), 0.5, 30, 0.33),
    ((-0.5, -0.5, -0.5), 0.5, 210, -0.33),
    ((0.5, -0.5, 0), 0.5, 300, 0.0),
    ((-0.5, 0.5, 0.5), 0.5, 120, 0.5),
)


@dataclass
class PlannedPose:
    world_T_flange: np.ndarray
    role: str  # "kalibrieren", "pruefen" or "wiederholen" (slip check)
    tilt_deg: float


def _mirrored(entry):
    """The same pose idea from the other side, tilted about the crossing axis."""
    (sx, sy, sz), tilt, azimuth, yaw = entry
    return (-sx, -sy, sz), tilt, (azimuth + 90) % 360, -yaw


def _plan_pose(start: np.ndarray, p: PlanParams, entry) -> Tuple[np.ndarray, float]:
    (sx, sy, sz), tilt, azimuth, yaw = entry
    along = np.array([start[0, 2], start[1, 2], 0.0])
    along /= max(np.linalg.norm(along), 1e-9)
    across = np.cross((0.0, 0.0, 1.0), along)
    az = math.radians(azimuth)
    rotvec_deg = tilt * (math.cos(az) * p.max_tilt_deg * along
                         + math.sin(az) * p.max_pitch_deg * across)
    tilt_deg = float(np.linalg.norm(rotvec_deg))
    R = axis_rotation((0, 0, 1), yaw * p.max_yaw_deg)
    if tilt_deg > 1e-9:
        R = R @ axis_rotation(rotvec_deg, tilt_deg)
    pivot = start[:3, :3] @ np.array([0.0, 0.0, p.pivot_along_tool_m]) + start[:3, 3]
    shift = np.array([sx * p.max_shift_m, sy * p.max_shift_m, sz * p.height_step_m])
    # rotate about the board centre, then shift, all in world
    T = make_transform(R, pivot - R @ pivot + shift) @ start
    if p.min_flange_z_m > 0 and T[2, 3] < p.min_flange_z_m:
        T[2, 3] = p.min_flange_z_m
    return T, tilt_deg


def _nearest_first(poses: List[PlannedPose]) -> List[PlannedPose]:
    """Drive order: always the nearest pose next (1 rad counted as 0.3 m), from
    the first. Short moves between poses instead of back-and-forth sweeps."""
    left = list(poses[1:])
    out = poses[:1]
    while left:
        last = out[-1].world_T_flange
        k = min(range(len(left)), key=lambda i: (
            np.linalg.norm(left[i].world_T_flange[:3, 3] - last[:3, 3])
            + 0.3 * math.radians(rotation_angle_deg(left[i].world_T_flange[:3, :3] @ last[:3, :3].T))))
        out.append(left.pop(k))
    return out


def plan_poses(start: np.ndarray, params: PlanParams) -> List[PlannedPose]:
    """Calibration poses, then held-out poses, then the first pose again."""
    out: List[PlannedPose] = []
    for n in range(max(1, params.passes)):
        for e in _CALIBRATION_POSES:
            if n % 2:
                e = _mirrored(e)
            if n >= 2:  # third and later passes: half the tilt, a different mix
                (s, tilt, az, yaw) = e
                e = (s, tilt * 0.5, (az + 45 * (n - 1)) % 360, yaw)
            T, tilt = _plan_pose(start, params, e)
            out.append(PlannedPose(T, "kalibrieren", tilt))
    out = _nearest_first(out)
    for e in _HOLDOUT_POSES:
        T, tilt = _plan_pose(start, params, e)
        out.append(PlannedPose(T, "pruefen", tilt))
    out.append(PlannedPose(out[0].world_T_flange.copy(), "wiederholen", 0.0))
    return out


def plan_problems(start: np.ndarray, poses: Sequence[PlannedPose],
                  params: PlanParams) -> List[str]:
    """Everything that forbids driving this plan; empty means drivable."""
    found = []
    tool_z = start[:3, 2]
    if abs(tool_z[2]) > math.sin(math.radians(20.0)):
        found.append("Werkzeugachse der Startpose nicht waagerecht (Board zeigt nicht zur Kamera)")
    for n, pose in enumerate(poses, 1):
        T = pose.world_T_flange
        pivot = T[:3, :3] @ np.array([0.0, 0.0, params.pivot_along_tool_m]) + T[:3, 3]
        # yaw keeps the board level; only the tilt lowers its edge
        lowest = pivot[2] - params.board_radius_m * math.sin(math.radians(pose.tilt_deg))
        floor = params.belt_z_m + params.min_clearance_m
        if lowest < floor:
            found.append(f"Pose {n}: Board-Kante {1000 * (floor - lowest):.0f} mm unter der Mindesthöhe")
        tip = T[:3, :3] @ np.array([0.0, 0.0, params.gripper_length_m]) + T[:3, 3]
        gripper = min(T[2, 3], tip[2]) - params.gripper_radius_m
        offset = float(np.linalg.norm(T[:2, 3] - start[:2, 3]))
        if offset > params.max_flange_offset_m:
            found.append(f"Pose {n}: Flansch {100 * offset:.0f} cm von der Startpose "
                         f"(höchstens {100 * params.max_flange_offset_m:.0f})")
        if params.min_flange_z_m > 0 and T[2, 3] < params.min_flange_z_m - 1e-9:
            found.append(f"Pose {n}: Flansch unter der Untergrenze {params.min_flange_z_m:.2f} m")
        if gripper < floor:
            found.append(f"Pose {n}: Greifer {1000 * (floor - gripper):.0f} mm unter der Mindesthöhe")
    return found


def required_start_height(params: PlanParams) -> float:
    """Lowest world z of the board centre at the start pose for which the plan
    keeps every clearance (the start itself taken level)."""
    if params.min_flange_z_m > 0:  # the floor lifts poses; the start must lie above it
        return max(params.min_flange_z_m, params.belt_z_m + params.min_clearance_m
                   + params.board_radius_m * math.sin(math.radians(
                       max(params.max_tilt_deg, params.max_pitch_deg))))
    tilt = math.sin(math.radians(max(params.max_tilt_deg, params.max_pitch_deg)))
    pitch = math.sin(math.radians(params.max_pitch_deg))
    below = max(params.board_radius_m * tilt,
                params.pivot_along_tool_m * pitch + params.gripper_radius_m)
    return params.belt_z_m + params.min_clearance_m + params.height_step_m + below


# ---------------------------------------------------------------------------
# Stage 1: solution
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    """One standing pose: robot flange and the averaged tag pixels there."""

    world_T_flange: np.ndarray
    obj: np.ndarray  # Nx3 board points
    img: np.ndarray  # Nx2 pixels
    #: The board surface as the depth image sees it (camera frame, base_cam's
    #: deprojection); None without a depth image at this pose.
    depth_plane: Optional["Plane"] = None


@dataclass
class Stage1Result:
    world_T_cam: np.ndarray
    flange_T_board: np.ndarray
    rms_px: float
    points_used: int
    points_rejected: int
    poses_dropped: int
    init_method: str
    #: 1-sigma of the camera position / orientation from the fit covariance.
    position_std_mm: float
    rotation_std_deg: float
    per_sample_rms_px: List[float] = field(default_factory=list)


def _residuals(X: np.ndarray, Y: np.ndarray, samples: Sequence[Sample],
               masks: Sequence[np.ndarray], K, D) -> np.ndarray:
    X_inv = invert(X)
    parts = []
    for s, m in zip(samples, masks):
        if not m.any():
            continue  # pose dropped as a whole
        C = X_inv @ s.world_T_flange @ Y
        proj, _ = cv2.projectPoints(s.obj[m], matrix_to_rotvec(C[:3, :3]), C[:3, 3], K, D)
        parts.append((proj.reshape(-1, 2) - s.img[m]).ravel())
    return np.concatenate(parts)


def _perturb(T: np.ndarray, delta: np.ndarray) -> np.ndarray:
    return T @ make_transform(rotvec_to_matrix(delta[:3]), delta[3:])


def _refine(X, Y, samples, masks, K, D, max_iter: int = 60):
    """Levenberg-Marquardt over (X, Y) with local perturbations; returns X, Y, J, r."""
    r = _residuals(X, Y, samples, masks, K, D)
    cost, lam, eps = float(r @ r), 1e-3, 1e-7
    J = None
    for _ in range(max_iter):
        J = np.empty((len(r), 12))
        for k in range(12):
            d = np.zeros(12)
            d[k] = eps
            J[:, k] = (_residuals(_perturb(X, d[:6]), _perturb(Y, d[6:]),
                                  samples, masks, K, D) - r) / eps
        A, g = J.T @ J, J.T @ r
        improved = False
        while lam < 1e10:
            step = -np.linalg.solve(A + lam * np.diag(np.diag(A)), g)
            Xn, Yn = _perturb(X, step[:6]), _perturb(Y, step[6:])
            rn = _residuals(Xn, Yn, samples, masks, K, D)
            if rn @ rn < cost:
                X, Y, r, cost = Xn, Yn, rn, float(rn @ rn)
                lam = max(lam / 10.0, 1e-9)
                improved = True
                break
            lam *= 10.0
        if not improved or np.linalg.norm(step) < 1e-10:
            break
    return X, Y, J, r


_INIT_METHODS = (("Park", cv2.CALIB_HAND_EYE_PARK), ("Tsai", cv2.CALIB_HAND_EYE_TSAI),
                 ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS))


def initial_guess(samples: Sequence[Sample], K, D,
                  prior: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, str]:
    """Start values: calibrateHandEye in the eye-to-hand form, or the prior.

    Fed with flange_T_world instead of world_T_flange, OpenCV's "cam2gripper"
    is world_T_cam, since inv(F_i) @ X @ C_i = Y must hold for every pose.
    Without tilt the closed forms are degenerate; then the calibration in
    force (``prior``) is the better start. The lowest pixel error wins.
    """
    views = [solve_pnp(s.obj, s.img, K, D)[0] for s in samples]
    flange_T_world = [invert(s.world_T_flange) for s in samples]
    masks = [np.ones(len(s.obj), dtype=bool) for s in samples]
    best = None
    if prior is not None:
        Y = average_transforms([Fi @ prior @ C for Fi, C in zip(flange_T_world, views)])
        best = (rms(_residuals(prior, Y, samples, masks, K, D)), prior.copy(), Y, "bisherige Kalibrierung")
    for name, method in _INIT_METHODS:
        try:
            R, t = cv2.calibrateHandEye([F[:3, :3] for F in flange_T_world],
                                        [F[:3, 3] for F in flange_T_world],
                                        [C[:3, :3] for C in views], [C[:3, 3] for C in views],
                                        method=method)
        except cv2.error:
            continue
        if not np.all(np.isfinite(R)) or not np.all(np.isfinite(t)):
            continue
        X = make_transform(orthonormalize(R), t)
        Y = average_transforms([Fi @ X @ C for Fi, C in zip(flange_T_world, views)])
        err = rms(_residuals(X, Y, samples, masks, K, D))
        if best is None or err < best[0]:
            best = (err, X, Y, name)
    if best is None:
        raise ValueError("Hand-Auge-Startwert nicht lösbar")
    return best[1], best[2], best[3]


def _all_turns_vertical(samples: Sequence[Sample], tol_deg: float = 1.0) -> bool:
    R0 = samples[0].world_T_flange[:3, :3]
    for s in samples[1:]:
        R = s.world_T_flange[:3, :3] @ R0.T
        axis = matrix_to_rotvec(R)
        if np.linalg.norm(axis) > 1e-9:
            if math.degrees(math.acos(min(1.0, abs(axis[2]) / np.linalg.norm(axis)))) > tol_deg \
                    and rotation_angle_deg(R) > tol_deg:
                return False
    return True


def solve_stage1(samples: Sequence[Sample], K: np.ndarray, D: np.ndarray,
                 outlier_px: float = 1.5, min_points_per_pose: int = 16,
                 prior: Optional[np.ndarray] = None,
                 camera_z_m: Optional[float] = None) -> Stage1Result:
    """world_T_cam and flange_T_board from the calibration samples.

    One pass of outlier removal: points further off than ``outlier_px`` or three
    times the RMS (whichever is larger) leave, then the fit is repeated. A pose
    left with fewer than ``min_points_per_pose`` points drops out as a whole.

    If the flange only turned about the vertical, one direction is not
    observable: camera and board may move up together without any pixel
    changing. ``camera_z_m`` (camera height in world, from the belt in the
    depth image) then fixes it; the solution is shifted along exactly that
    direction, which leaves every residual as it is.
    """
    if len(samples) < 6:
        raise ValueError(f"zu wenige Posen: {len(samples)} (mindestens 6)")
    K = np.asarray(K, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    vertical_only = _all_turns_vertical(samples)
    if vertical_only and camera_z_m is None:
        raise ValueError("nur Drehungen um die Hochachse: die Kamerahöhe fehlt (Tiefenbild)")
    X, Y, init_name = initial_guess(samples, K, D, prior)
    masks = [np.ones(len(s.obj), dtype=bool) for s in samples]
    X, Y, _, r = _refine(X, Y, samples, masks, K, D)

    limit = max(outlier_px, 3.0 * rms(np.linalg.norm(r.reshape(-1, 2), axis=1)))
    rejected, dropped, new_masks = 0, 0, []
    for s in samples:
        err = reprojection_errors(s.obj, s.img, invert(X) @ s.world_T_flange @ Y, K, D)
        m = err <= limit
        if m.sum() < min_points_per_pose:  # too little left: the pose itself is off
            m[:] = False
            dropped += 1
        rejected += int((~m).sum())
        new_masks.append(m)
    if len(samples) - dropped < 6:
        raise ValueError(f"nach der Ausreißerprüfung zu wenige Posen: {len(samples) - dropped}")
    X, Y, J, r = _refine(X, Y, samples, new_masks, K, D)
    if vertical_only:
        # move camera and board up together to the measured camera height
        h = float(camera_z_m) - X[2, 3]
        X, Y = X.copy(), Y.copy()
        X[2, 3] += h
        Y[:3, 3] += h * (samples[0].world_T_flange[:3, :3].T @ np.array([0.0, 0.0, 1.0]))
        r = _residuals(X, Y, samples, new_masks, K, D)

    per_point = np.linalg.norm(r.reshape(-1, 2), axis=1)
    dof = max(1, len(r) - 12)
    try:
        cov = float(r @ r) / dof * np.linalg.pinv(J.T @ J)
        pos_std = 1000.0 * math.sqrt(max(0.0, np.trace(cov[3:6, 3:6])))
        rot_std = math.degrees(math.sqrt(max(0.0, np.trace(cov[0:3, 0:3]))))
    except np.linalg.LinAlgError:
        pos_std = rot_std = float("inf")
    per_sample = [rms(reprojection_errors(s.obj[m], s.img[m],
                                          invert(X) @ s.world_T_flange @ Y, K, D))
                  if m.any() else float("nan") for s, m in zip(samples, new_masks)]
    return Stage1Result(world_T_cam=X, flange_T_board=Y, rms_px=rms(per_point),
                        points_used=int(sum(m.sum() for m in new_masks)),
                        points_rejected=rejected, poses_dropped=dropped,
                        init_method=init_name,
                        position_std_mm=pos_std, rotation_std_deg=rot_std,
                        per_sample_rms_px=per_sample)


@dataclass
class Validation:
    """Held-out poses: how well the solution predicts what it has not seen."""

    rms_px: float
    #: Board corners placed by the robot chain vs. seen by the camera, in world.
    mean_mm: float
    max_mm: float


def validate(result: Stage1Result, samples: Sequence[Sample], K, D) -> Validation:
    px, dist = [], []
    X, Y = result.world_T_cam, result.flange_T_board
    for s in samples:
        px.extend(reprojection_errors(s.obj, s.img, invert(X) @ s.world_T_flange @ Y, K, D))
        C, _ = solve_pnp(s.obj, s.img, K, D)
        pts = np.c_[s.obj, np.ones(len(s.obj))].T
        by_robot = (s.world_T_flange @ Y @ pts)[:3]
        by_camera = (X @ C @ pts)[:3]
        dist.extend(1000.0 * np.linalg.norm(by_robot - by_camera, axis=0))
    if not dist:
        return Validation(float("nan"), float("nan"), float("nan"))
    return Validation(rms(np.asarray(px)), float(np.mean(dist)), float(np.max(dist)))


def board_slip(world_T_cam: np.ndarray, first: Sample, again: Sample, K, D) -> Tuple[float, float]:
    """Shift (mm) and turn (deg) of the board in the jaws between two visits of
    the same pose. Camera errors cancel since both views are nearly identical."""
    Y = [invert(s.world_T_flange) @ world_T_cam @ solve_pnp(s.obj, s.img, K, D)[0]
         for s in (first, again)]
    delta = invert(Y[0]) @ Y[1]
    return 1000.0 * float(np.linalg.norm(delta[:3, 3])), rotation_angle_deg(delta[:3, :3])


# ---------------------------------------------------------------------------
# Belt plane from the aligned depth image
# ---------------------------------------------------------------------------

@dataclass
class Plane:
    """n . p + d = 0 in the camera frame; n points towards the camera."""

    normal: np.ndarray
    offset: float
    rms_mm: float
    count: int


def depth_points(depth_m: np.ndarray, K: np.ndarray,
                 roi: Tuple[int, int, int, int] = (0, 0, 0, 0), stride: int = 4) -> np.ndarray:
    """Nx3 camera-frame points of a color-aligned depth image (meters, 0 = none).

    Lens distortion is ignored: the L515 color stream is close to distortion
    free, and the plane only feeds a check of tilt and height.
    """
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        x, y, h, w = 0, 0, depth_m.shape[0], depth_m.shape[1]
    vs, us = np.mgrid[y:y + h:stride, x:x + w:stride]
    z = depth_m[vs, us]
    ok = np.isfinite(z) & (z > 0)
    z, us, vs = z[ok], us[ok], vs[ok]
    return np.c_[(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z]


def fit_plane(points: np.ndarray, inlier_mm: float = 4.0, iterations: int = 300,
              seed: int = 0) -> Plane:
    """The dominant plane (RANSAC), then least squares over its points.

    At the setup the arm and the board stand in the depth image above the belt;
    a plain fit tilted towards them by 72 degrees (25.09.2026). The belt is
    the largest plane in the ROI, so the consensus finds it.
    """
    if len(points) < 3:
        raise ValueError("zu wenige Tiefenpunkte für die Ebene")
    rng = np.random.default_rng(seed)
    tol = inlier_mm / 1000.0
    best, best_count = None, -1
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n = n / norm
        count = int(np.count_nonzero(np.abs((points - a) @ n) <= tol))
        if count > best_count:
            best, best_count = (n, a), count
    if best is None:
        raise ValueError("keine Ebene im Tiefenbild")
    n, a = best
    keep = np.abs((points - a) @ n) <= tol
    for _ in range(2):  # refine on the consensus set
        centre = points[keep].mean(axis=0)
        _, _, Vt = np.linalg.svd(points[keep] - centre, full_matrices=False)
        n = Vt[2]
        keep = np.abs((points - centre) @ n) <= tol
    if n[2] > 0:
        n = -n  # towards the camera
    d = -float(n @ points[keep].mean(axis=0))
    return Plane(n, d, 1000.0 * rms(points[keep] @ n + d), int(keep.sum()))


def camera_height_from_depth(depth_m: np.ndarray, K: np.ndarray, expected_height_m: float,
                             belt_z_m: float, window_m: float = 0.08,
                             stride: int = 4) -> Tuple[float, Plane]:
    """Camera z in world from the belt in the depth image: belt z + distance.

    Only depths within ``window_m`` of the expected camera height above the
    belt count -- board and arm, much nearer the camera, drop out before the
    plane is sought. The belt lies level in world (B17), so the distance to
    its plane is the height above it.
    """
    pts = depth_points(depth_m, K, stride=stride)
    pts = pts[np.abs(pts[:, 2] - expected_height_m) <= window_m]
    if len(pts) < 2000:
        raise ValueError(f"Band im Tiefenbild nicht gefunden ({len(pts)} Punkte im Fenster)")
    plane = fit_plane(pts)
    if plane.count < 0.5 * len(pts) or plane.rms_mm > 5.0:
        raise ValueError(f"Bandebene unsicher: {plane.count} von {len(pts)} Punkten, "
                         f"Rest {plane.rms_mm:.1f} mm")
    return belt_z_m + abs(plane.offset), plane


def belt_plane_check(world_T_cam: np.ndarray, plane: Plane, belt_z_m: float) -> Tuple[float, float]:
    """Tilt of the measured plane against world-horizontal (deg) and its height
    where the optical axis meets it, minus the belt surface (mm)."""
    n_world = world_T_cam[:3, :3] @ plane.normal
    tilt = math.degrees(math.acos(min(1.0, abs(float(n_world[2])))))
    t = -plane.offset / float(plane.normal[2])
    hit = world_T_cam @ np.array([0.0, 0.0, t, 1.0])
    return tilt, 1000.0 * (float(hit[2]) - belt_z_m)


# ---------------------------------------------------------------------------
# Depth image against colour image (L515)
# ---------------------------------------------------------------------------
#
# base_cam places a block from the aligned depth image (outline and depth) in
# the colour pixel grid, without undistortion. The L515 depth is tilted against
# its colour image by about a degree and reads 4-13 mm too deep (board seen by
# both at once, setup 25.09.2026), so the colour camera's true pose (stage 1
# above) would put base_cam's blocks off by up to 13 mm in height. Stage 1
# records the board in the depth image at every pose; the error was modelled
# from that (27.09.2026, L27, docs/architektur/bilder/2026-09-25-basiskamera-
# kalibrierung/modell_tiefenfehler.py): see DepthErrorModel below.
#
# base_cam stays as it is. Instead the calibration hands it a pose made for its
# own view, the way the hand calibration L6 was made (tilt and height from the
# belt in the depth image, yaw and x/y from blocks at belt level) -- only
# automatically: see basecam_pose().


def pixel_rays(px: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Nx3 rays with z = 1, as base_cam deprojects (no undistortion)."""
    px = np.asarray(px, dtype=np.float64).reshape(-1, 2)
    return np.c_[(px[:, 0] - K[0, 2]) / K[0, 0], (px[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(px))]


def board_depth_plane(depth_m: np.ndarray, K: np.ndarray, corners_px: np.ndarray,
                      shrink_px: int = 8, stride: int = 4) -> Optional[Plane]:
    """Plane of the board in the depth image, over the area inside its corners
    (every ``stride``-th pixel: some 8000 points, a few ms per pose)."""
    pts = np.round(np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)).astype(np.int32)
    if len(pts) < 8:
        return None
    mask = np.zeros(depth_m.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(pts), 1)
    if shrink_px > 0:
        mask = cv2.erode(mask, np.ones((2 * shrink_px + 1, 2 * shrink_px + 1), np.uint8))
    vs, us = np.nonzero(mask[::stride, ::stride])
    vs, us = vs * stride, us * stride
    z = depth_m[vs, us]
    ok = np.isfinite(z) & (z > 0)
    if ok.sum() < 300:
        return None
    rays = pixel_rays(np.c_[us[ok], vs[ok]], K)
    try:
        plane = fit_plane(rays * z[ok, None], inlier_mm=3.0)
    except ValueError:
        return None
    return plane if plane.count >= 0.5 * ok.sum() else None


def depth_grid(depth_m: np.ndarray, corners_px: np.ndarray, stride: int = 8,
               shrink_px: int = 8) -> np.ndarray:
    """Mx3 (u, v, depth m) on a pixel grid inside the corners -- raw material for
    modelling the depth error against the colour image, stored with every pose."""
    pts = np.round(np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)).astype(np.int32)
    if len(pts) < 3:
        return np.zeros((0, 3))
    mask = np.zeros(depth_m.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(pts), 1)
    if shrink_px > 0:
        mask = cv2.erode(mask, np.ones((2 * shrink_px + 1, 2 * shrink_px + 1), np.uint8))
    vs, us = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    keep = mask[vs, us] > 0
    z = depth_m[vs, us]
    keep &= np.isfinite(z) & (z > 0)
    return np.c_[us[keep], vs[keep], z[keep]]


@dataclass(frozen=True)
class DepthErrorModel:
    """How much the L515 depth reads too deep, against the colour image (model M1s).

    In what base_cam has at hand -- pixel (u, v), measured depth z in m, rays
    without undistortion, X = (u - cx) / fx * z, Y = (v - cy) / fy * z::

        error [mm] = c0 + cx * X + cy * Y + cz * z

    The defaults were fitted on 13 board images at belt level and 28 gripper
    poses (27.09.2026, L27): 3.7 mm offset, 1.25 deg about the image x axis,
    0.19 deg about y, 4.4 mm per m. Left out in turn, no image missed by more
    than 1.5 mm (without the model: 5.5-13.4 mm). A property of the camera,
    not of where it stands.
    """

    c0_mm: float = 3.715
    cx_mm_per_m: float = 3.359
    cy_mm_per_m: float = 21.812
    cz_mm_per_m: float = 4.427

    def error_mm(self, px: np.ndarray, depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
        rays = pixel_rays(px, K)
        z = np.asarray(depth_m, dtype=np.float64).reshape(-1)
        return (self.c0_mm + self.cx_mm_per_m * rays[:, 0] * z
                + self.cy_mm_per_m * rays[:, 1] * z + self.cz_mm_per_m * z)

    def to_dict(self) -> dict:
        return {"c0_mm": self.c0_mm, "cx_mm_per_m": self.cx_mm_per_m,
                "cy_mm_per_m": self.cy_mm_per_m, "cz_mm_per_m": self.cz_mm_per_m}


#: Heights above the belt the base_cam pose is fitted at: belt, flat block,
#: block halves, 100 mm block -- where base_cam measures.
BASECAM_LEVELS_M = (0.0, 0.025, 0.05, 0.10)


@dataclass
class BaseCamPose:
    world_T_cam: np.ndarray
    #: What a rigid pose cannot absorb of the depth error, over the ROI and the
    #: levels: horizontal distance of base_cam's points from the truth.
    rest_mm_rms: float
    rest_mm_max: float
    points: int


def _tilt_to_level(normal: np.ndarray) -> np.ndarray:
    """Shortest rotation taking ``normal`` (camera frame, towards the camera) to world +z."""
    n = np.asarray(normal, dtype=np.float64) / np.linalg.norm(normal)
    up = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, up)
    s, c = float(np.linalg.norm(v)), float(n @ up)
    if s < 1e-12:
        return np.eye(3) if c > 0 else axis_rotation((1.0, 0.0, 0.0), 180.0)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / s ** 2)


def basecam_pose(world_T_colour: np.ndarray, K: np.ndarray, D: np.ndarray, belt_plane: Plane,
                 belt_z_m: float, roi: Tuple[int, int, int, int],
                 model: DepthErrorModel = DepthErrorModel(),
                 levels: Sequence[float] = BASECAM_LEVELS_M, grid: int = 24) -> BaseCamPose:
    """The camera pose for base_cam's view, from the colour pose of stage 1.

    Built like the hand calibration L6, which base_cam has been gripping with:

    * **tilt** from the belt plane in the depth image (as base_cam deprojects
      it): the belt lies level, block heights come out against the belt. This
      also takes up the belt's own cross slope (about 0.5 deg, L27) -- base_cam
      measures height against a level belt.
    * **height** so that this belt lies at ``belt_z_m``.
    * **yaw and x/y** from points at belt and block level over the ROI: where
      the colour pose says a point is, against where base_cam would put it from
      its pixel and the depth the model says the L515 reads there.

    ``world_T_colour`` may carry a camera height from the uncorrected depth: it
    shifts the levels by millimetres along z, which moves nothing here.
    Checked against 12 board images at belt level (left out of the fit): 3.3 mm
    mean, 7.1 mm largest position error, against 4.3 / 8.4 mm of L6; board
    heights as even as with L6 (27.09.2026).
    """
    R_tilt = _tilt_to_level(belt_plane.normal)

    # pairs: base_cam's point (camera frame) -> true point (world)
    x, y, w, h = roi
    us, vs = np.meshgrid(np.linspace(x, x + w, grid), np.linspace(y, y + h, grid))
    px = np.c_[us.ravel(), vs.ravel()]
    und = cv2.undistortPoints(px.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays_world = np.c_[und, np.ones(len(und))] @ world_T_colour[:3, :3].T
    origin = world_T_colour[:3, 3]
    colour_T_world = invert(world_T_colour)
    base, true = [], []
    for level in levels:
        s = (belt_z_m + level - origin[2]) / rays_world[:, 2]
        ok = s > 0
        W = origin + s[ok, None] * rays_world[ok]
        z_true = (W @ colour_T_world[:3, :3].T + colour_T_world[:3, 3])[:, 2]
        z = z_true.copy()
        for _ in range(3):  # the error depends on the depth as read
            z = z_true + model.error_mm(px[ok], z, K) / 1000.0
        base.append(pixel_rays(px[ok], K) * z[:, None])
        true.append(W)
    P = np.vstack(base) @ R_tilt.T
    Q = np.vstack(true)
    if len(P) < 10:
        raise ValueError("Bildausschnitt trifft das Band nicht")

    # yaw and x/y: 2D Procrustes after the tilt
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    a, b = P[:, :2] - pc[:2], Q[:, :2] - qc[:2]
    theta = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])),
                       float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])))
    Rz = axis_rotation((0.0, 0.0, 1.0), math.degrees(theta))
    T = np.eye(4)
    T[:3, :3] = orthonormalize(Rz @ R_tilt)
    T[:2, 3] = qc[:2] - (Rz @ pc)[:2]
    # height: points of the belt plane satisfy n.p = -offset, and R_tilt n = +z
    T[2, 3] = belt_z_m + belt_plane.offset
    rest = np.linalg.norm((P @ Rz.T)[:, :2] + T[:2, 3] - Q[:, :2], axis=1) * 1000.0
    return BaseCamPose(T, rms(rest), float(np.max(rest)), len(P))


# ---------------------------------------------------------------------------
# Stage 2: reference tags on the conveyor frame, and the check
# ---------------------------------------------------------------------------

def marker_corners_local(size_m: float) -> np.ndarray:
    """Corners of a single tag in its own frame (centre, x right, y up)."""
    s = size_m / 2.0
    return np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]])


def reference_corners_world(world_T_cam: np.ndarray, corners_px: np.ndarray,
                            size_m: float, K, D) -> np.ndarray:
    """4x3 world corners of one reference tag seen in the calibrated camera."""
    local = marker_corners_local(size_m)
    ok, rvec, tvec = cv2.solvePnP(local, np.asarray(corners_px, dtype=np.float64), K, D,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("Referenzmarke nicht lösbar")
    T = world_T_cam @ make_transform(rotvec_to_matrix(rvec), tvec)
    return (T @ np.c_[local, np.ones(4)].T)[:3].T


def solve_from_references(references: Dict[int, np.ndarray], detections: Detections,
                          K, D, guess: Optional[np.ndarray] = None,
                          min_tags: int = 2) -> Tuple[np.ndarray, float, List[int]]:
    """world_T_cam from reference tags with known world corners."""
    used = sorted(i for i in detections if i in references)
    if len(used) < min_tags:
        raise ValueError(f"zu wenige Referenzmarken im Bild: {len(used)} (mindestens {min_tags})")
    obj = np.vstack([references[i] for i in used])
    img = np.vstack([detections[i] for i in used])
    cam_T_world, err = solve_pnp(obj, img, np.asarray(K, float), np.asarray(D, float),
                                 guess=None if guess is None else invert(guess))
    return invert(cam_T_world), err, used


def belt_sample_points(world_T_cam: np.ndarray, K: np.ndarray, image_size: Tuple[int, int],
                       belt_z_m: float, roi: Tuple[int, int, int, int] = (0, 0, 0, 0),
                       grid: int = 5) -> np.ndarray:
    """World points on the belt plane under a grid of pixels (Nx3)."""
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        x, y, w, h = 0, 0, image_size[0], image_size[1]
    pts = []
    for v in np.linspace(y, y + h - 1, grid):
        for u in np.linspace(x, x + w - 1, grid):
            ray = world_T_cam[:3, :3] @ np.array([(u - K[0, 2]) / K[0, 0],
                                                  (v - K[1, 2]) / K[1, 1], 1.0])
            if abs(ray[2]) < 1e-9:
                continue
            t = (belt_z_m - world_T_cam[2, 3]) / ray[2]
            if t > 0:
                pts.append(world_T_cam[:3, 3] + t * ray)
    return np.array(pts)


def belt_displacement_mm(world_T_cam_a: np.ndarray, world_T_cam_b: np.ndarray,
                         points_world: np.ndarray) -> float:
    """Largest shift of a seen point when calibration a is replaced by b."""
    if len(points_world) == 0:
        return float("nan")
    M = world_T_cam_b @ invert(world_T_cam_a)
    moved = (M @ np.c_[points_world, np.ones(len(points_world))].T)[:3].T
    return 1000.0 * float(np.max(np.linalg.norm(moved - points_world, axis=1)))


@dataclass
class CheckLimits:
    """When the camera counts as moved (start-up check)."""

    max_belt_shift_mm: float = 3.0
    max_angle_deg: float = 0.5


def camera_moved(stored: np.ndarray, current: np.ndarray, belt_points: np.ndarray,
                 limits: CheckLimits) -> Tuple[bool, float, float]:
    """(moved, largest belt shift in mm, rotation in deg) between two calibrations."""
    shift = belt_displacement_mm(stored, current, belt_points)
    angle = rotation_angle_deg(current[:3, :3] @ stored[:3, :3].T)
    return (shift > limits.max_belt_shift_mm or angle > limits.max_angle_deg), shift, angle


# ---------------------------------------------------------------------------
# Calibration file
# ---------------------------------------------------------------------------

@dataclass
class ExtrinsicsRecord:
    world_T_cam: np.ndarray
    method: str  # "stufe1" or "stufe2"
    created: str = ""
    quality: dict = field(default_factory=dict)
    #: Reference tags for stage 2: size and world corners (id -> 4x3).
    reference_size_m: float = 0.0
    references: Dict[int, np.ndarray] = field(default_factory=dict)
    grid: dict = field(default_factory=dict)

    def cal(self) -> Dict[str, float]:
        return cal_from_matrix(self.world_T_cam)


def record_to_dict(record: ExtrinsicsRecord) -> dict:
    return {
        "schema": SCHEMA,
        "version": SCHEMA_VERSION,
        "created": record.created or datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "method": record.method,
        "frame": {"parent": "world", "child": "base camera, color optical frame"},
        "convention": "R = Rz(yaw) @ Ry(pitch) @ Rx(roll); m, deg",
        "cal": {k: round(v, 6) for k, v in record.cal().items()},
        "matrix": [[round(float(v), 9) for v in row] for row in record.world_T_cam],
        "quality": record.quality,
        "references": {
            "size_m": record.reference_size_m,
            "corners_world": {str(i): np.round(c, 6).tolist()
                              for i, c in sorted(record.references.items())},
        },
        "grid": record.grid,
    }


def save_record(path: str, record: ExtrinsicsRecord) -> None:
    """Write atomically: a crash mid-write never leaves a broken file behind."""
    data = record_to_dict(record)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")
    os.replace(tmp, path)


def replace_calibration(path: str, record: ExtrinsicsRecord) -> Optional[str]:
    """Write ``record`` over the calibration at ``path`` (relative = to the package),
    keeping the file it replaces as ``<name>_vorher.json``. Returns the backup path,
    None if there was nothing to keep.

    The calibration run writes straight into the file base_cam reads by default
    (28.09.2026): base_cam takes it at its next activation, no build. One step
    back stays possible without a build via the backup.
    """
    path = resolve_calibration_path(path)
    backup = None
    if os.path.exists(path):
        stem, ext = os.path.splitext(path)
        backup = stem + "_vorher" + ext
        with open(path, encoding="utf-8") as src, open(backup, "w", encoding="utf-8") as dst:
            dst.write(src.read())
    save_record(path, record)
    return backup


def _matrix_problems(T: np.ndarray) -> List[str]:
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return ["Matrix nicht 4x4 oder nicht endlich"]
    found = []
    R = T[:3, :3]
    if np.max(np.abs(R.T @ R - np.eye(3))) > 1e-6 or abs(np.linalg.det(R) - 1.0) > 1e-6:
        found.append("Drehung nicht orthonormal")
    if np.max(np.abs(T[3] - [0, 0, 0, 1])) > 1e-9:
        found.append("letzte Zeile nicht 0 0 0 1")
    if np.linalg.norm(T[:3, 3]) > 5.0:
        found.append("Kamera mehr als 5 m vom Ursprung")
    return found


def record_from_dict(data: dict) -> ExtrinsicsRecord:
    if data.get("schema") != SCHEMA:
        raise ValueError(f"kein Kalibrier-Datensatz der Basiskamera (schema {data.get('schema')!r})")
    if data.get("version") != SCHEMA_VERSION:
        raise ValueError(f"unbekannte Version {data.get('version')!r}")
    T = np.asarray(data.get("matrix"), dtype=np.float64)
    problems = _matrix_problems(T)
    if problems:
        raise ValueError("; ".join(problems))
    refs = data.get("references") or {}
    corners = {int(k): np.asarray(v, dtype=np.float64).reshape(4, 3)
               for k, v in (refs.get("corners_world") or {}).items()}
    return ExtrinsicsRecord(world_T_cam=T, method=str(data.get("method", "")),
                            created=str(data.get("created", "")),
                            quality=dict(data.get("quality") or {}),
                            reference_size_m=float(refs.get("size_m") or 0.0),
                            references=corners, grid=dict(data.get("grid") or {}))


def resolve_calibration_path(value: str) -> str:
    """Absolute path of the ``calibration_file`` parameter; relative = to the package."""
    value = value.strip()
    return value if os.path.isabs(value) else os.path.join(PACKAGE_DIR, value)


def load_camera_calibration(value: str) -> Tuple[Optional[ExtrinsicsRecord], str]:
    """The record named by ``calibration_file`` and a log line saying what holds.

    Empty value: no file, the cal_* parameters hold. A file that is missing or
    broken also leaves the cal_* parameters in force -- the line says why, so
    the log shows which calibration the detection runs on.
    """
    if not value.strip():
        return None, "Kalibrierdatei leer - es gelten die cal_*-Parameter"
    path = resolve_calibration_path(value)
    try:
        record = load_record(path)
    except ValueError as exc:
        return None, f"Kalibrierdatei {path} unbrauchbar ({exc}) - es gelten die cal_*-Parameter"
    cal = record.cal()
    return record, (f"Kalibrierung aus {path}: {record.method}, {record.created}; "
                    + ", ".join(f"{k} {v:.4f}" for k, v in cal.items()))


def load_record(path: str) -> ExtrinsicsRecord:
    """Read and check a calibration file; raises ValueError with the reason."""
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except OSError as exc:
        raise ValueError(f"Kalibrierdatei nicht lesbar: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Kalibrierdatei kein gültiges JSON: {exc}") from exc
    return record_from_dict(data)
