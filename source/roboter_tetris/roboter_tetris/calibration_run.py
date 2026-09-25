"""Run sequence of the base camera calibration behind `base_cam_calibration`.

No ROS: the component only turns images into tag detections, feeds them in
together with the flange pose, and sends the target pose that comes out to the
Signal Point Attractor. The mathematics lives in :mod:`basecam_extrinsics`.

Three modes, one per AICA block of the calibration application:

``stufe1`` -- the robot moves. The run first holds the flange where it is, looks
at the board there, checks the layout of the grid, measures how far the board
sits in front of the flange, plans the poses around that start and refuses to
move if the plan breaks a clearance. Then it drives pose by pose, waits until the
flange stands still, averages a few frames, and at the end returns to the start
and solves. The result is written only if its quality checks hold.

``stufe2`` -- no motion. Solves the camera from the reference tags on the
conveyor frame, whose world corners the calibration file holds since stage 1.

``pruefen`` -- no motion. Same as stage 2 but writes nothing; it only answers
"has the camera moved?" -- the start-up check.

States (``state``)::

    WARTEN -> START -> FAHREN -> BERUHIGEN -> AUFNAHME -> ... -> RUECKFAHRT
           -> RECHNEN -> FERTIG                          (stufe1)
    WARTEN -> AUFNAHME -> RECHNEN -> FERTIG              (stufe2, pruefen)
    any -> FEHLER; after motion via RUECKFAHRT to the start pose
"""

import datetime
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

from .basecam_extrinsics import (
    DEFAULT_CALIBRATION_FILE, CheckLimits, Detections, ExtrinsicsRecord, GridSpec, PlanParams, Sample,
    average_detections, average_transforms, belt_displacement_mm, belt_plane_check,
    belt_sample_points, board_slip, camera_moved, depth_points, fit_plane, grid_points,
    identify_layout, layout_candidates, plan_poses, plan_problems, reference_corners_world,
    rotation_angle_deg, solve_from_references, solve_pnp, solve_stage1, validate,
)

MODES = ("stufe1", "stufe2", "pruefen")

WARTEN = "WARTEN"
START = "START"
FAHREN = "FAHREN"
BERUHIGEN = "BERUHIGEN"
AUFNAHME = "AUFNAHME"
RUECKFAHRT = "RUECKFAHRT"
RECHNEN = "RECHNEN"
FERTIG = "FERTIG"
FEHLER = "FEHLER"


@dataclass
class RunParams:
    mode: str = "stufe1"
    #: Flange must stand still this long (within still_mm) before frames count.
    settle_s: float = 1.0
    still_mm: float = 0.3
    #: "Arrived" at a planned pose. The attractor need not hit it exactly -- the
    #: solution uses the measured flange pose, never the commanded one.
    reach_mm: float = 5.0
    reach_deg: float = 2.0
    frames_per_pose: int = 8
    pose_timeout_s: float = 40.0
    #: Fewer grid tags than this at a pose: the pose counts as missing.
    min_tags: int = 8
    max_missing_poses: int = 6
    reference_first_id: int = 100
    reference_size_m: float = 0.07
    min_reference_tags: int = 2
    #: Belt area the comparisons are measured in: base_cam's ROI (L21).
    roi: Tuple[int, int, int, int] = (342, 60, 618, 580)
    grid: GridSpec = field(default_factory=GridSpec)
    plan: PlanParams = field(default_factory=PlanParams)
    limits: CheckLimits = field(default_factory=CheckLimits)
    #: Quality gates of stage 1; a result outside them is reported, not written.
    max_rms_px: float = 1.0
    max_holdout_mm: float = 2.0
    max_slip_mm: float = 0.5
    max_slip_deg: float = 0.2

    def problems(self) -> List[str]:
        found = []
        if self.mode not in MODES:
            found.append(f"Betriebsart {self.mode!r} unbekannt ({', '.join(MODES)})")
        if self.frames_per_pose < 1 or self.settle_s < 0 or self.still_mm <= 0:
            found.append("Aufnahme: Bilder je Pose >= 1, Wartezeit >= 0, Ruhe > 0")
        if self.grid.rows < 2 or self.grid.cols < 2 or self.grid.tag_size_m <= 0:
            found.append("Board: mindestens 2 x 2 Tags, Taggröße > 0")
        if self.reference_first_id < self.grid.first_id + self.grid.rows * self.grid.cols:
            found.append("Referenzmarken-IDs überschneiden sich mit dem Board")
        if self.plan.passes < 1:
            found.append("Durchgänge >= 1")
        return found


#: Parameters of the component: name, default, description. The component
#: registers them from here and builds RunParams with :func:`run_params`, so the
#: component description can be checked against this table without ROS.
PARAMETERS = (
    ("mode", "stufe1",
     "Betriebsart: stufe1 = Roboter fährt das Board ab (Grundkalibrierung), stufe2 = neu aus "
     "den Referenzmarken ohne Bewegung, pruefen = nur prüfen, ob sich die Kamera bewegt hat "
     "(schreibt nichts)."),
    ("output_file", "/tmp/base_cam_extrinsics.json",
     "Hierhin schreibt stufe1/stufe2 das Ergebnis (im Container). Übernahme: ins Repo nach "
     "roboter_tetris/Extrinsics/base_cam_extrinsics.json kopieren, Paket bauen."),
    ("calibration_file", DEFAULT_CALIBRATION_FILE,
     "Bisher gültige Kalibrierdatei, relativ zum Paket oder absolut: Vergleich, Lage der "
     "Referenzmarken für stufe2/pruefen. Leer = Übergangswerte L6."),
    ("grid_rows", 7, "Board: Tag-Zeilen des AprilGrid."),
    ("grid_cols", 11, "Board: Tag-Spalten des AprilGrid."),
    ("tag_size_mm", 20.0, "Board: Kantenlänge eines Tags (schwarzer Rand außen) in mm."),
    ("tag_spacing_mm", 6.0, "Board: Abstand zwischen zwei Tags in mm."),
    ("tag_dictionary", "DICT_APRILTAG_36h11", "Tag-Familie (OpenCV-Name), Board und Referenzmarken."),
    ("reference_first_id", 100,
     "Referenzmarken am Bandgestell: kleinste ID. Tags ab dieser ID gelten als Referenz."),
    ("reference_size_mm", 70.0, "Referenzmarken: Kantenlänge (schwarzer Rand außen) in mm."),
    ("max_shift_m", 0.15, "Posenplan: Verschiebung der Board-Mitte um die Startpose, je Richtung (m)."),
    ("height_step_m", 0.08, "Posenplan: Höhenänderung nach oben und unten (m)."),
    ("max_tilt_deg", 25.0, "Posenplan: seitliches Kippen um die Werkzeugachse (Grad)."),
    ("max_pitch_deg", 12.0,
     "Posenplan: Nicken quer zur Werkzeugachse (Grad). Der Flansch schwingt dabei um "
     "Abstand x sin(Winkel) nach unten - bestimmt die Mindesthöhe der Startpose."),
    ("max_yaw_deg", 35.0, "Posenplan: Drehung um die Hochachse (Grad)."),
    ("passes", 2, "Posenplan: Durchgänge zu je 20 Posen. Mehr Posen mitteln den Posenfehler des Roboters."),
    ("min_clearance_m", 0.10, "Mindestabstand von Board, Flansch und Greifer zum Band (m)."),
    ("belt_z_m", 0.0536, "Bandoberfläche in world (m), B17."),
    ("settle_s", 1.0, "Wartezeit nach dem Ankommen, in der der Flansch ruhen muss (s)."),
    ("frames_per_pose", 8, "Bilder, die je Pose gemittelt werden."),
    ("depth_scale_to_mm", 1.0, "mm pro Tiefen-Rohwert (16UC1), wie in base_cam."),
    ("roi_x", 342, "Bandbereich im Bild für die Vergleiche: x-Offset in px (wie base_cam)."),
    ("roi_y", 60, "Bandbereich: y-Offset in px."),
    ("roi_width", 618, "Bandbereich: Breite in px."),
    ("roi_height", 580, "Bandbereich: Höhe in px."),
    ("max_belt_shift_mm", 3.0, "Prüfen: ab dieser Verschiebung auf dem Band gilt die Kamera als bewegt (mm)."),
    ("max_angle_deg", 0.5, "Prüfen: ab dieser Verdrehung gilt die Kamera als bewegt (Grad)."),
    ("debug_enable", True, "Debug-Bild mit erkannten Tags und Zustand senden."),
)


def run_params(values: Dict[str, object]) -> RunParams:
    """RunParams from the component's parameter values (see PARAMETERS)."""
    v = values
    grid = GridSpec(rows=int(v["grid_rows"]), cols=int(v["grid_cols"]),
                    tag_size_m=float(v["tag_size_mm"]) / 1000.0,
                    tag_spacing_m=float(v["tag_spacing_mm"]) / 1000.0)
    plan = PlanParams(max_shift_m=float(v["max_shift_m"]), height_step_m=float(v["height_step_m"]),
                      max_tilt_deg=float(v["max_tilt_deg"]), max_pitch_deg=float(v["max_pitch_deg"]),
                      max_yaw_deg=float(v["max_yaw_deg"]), passes=int(v["passes"]),
                      belt_z_m=float(v["belt_z_m"]), min_clearance_m=float(v["min_clearance_m"]))
    return RunParams(mode=str(v["mode"]).strip(), settle_s=float(v["settle_s"]),
                     frames_per_pose=int(v["frames_per_pose"]),
                     reference_first_id=int(v["reference_first_id"]),
                     reference_size_m=float(v["reference_size_mm"]) / 1000.0,
                     roi=(int(v["roi_x"]), int(v["roi_y"]), int(v["roi_width"]), int(v["roi_height"])),
                     grid=grid, plan=plan,
                     limits=CheckLimits(max_belt_shift_mm=float(v["max_belt_shift_mm"]),
                                        max_angle_deg=float(v["max_angle_deg"])))


@dataclass
class Camera:
    K: np.ndarray
    D: np.ndarray
    size: Tuple[int, int]  # width, height


@dataclass
class Frame:
    """One camera frame, as the component hands it in."""

    stamp: float
    detections: Detections
    depth_m: Optional[np.ndarray] = None


@dataclass
class _Capture:
    frames: List[Frame] = field(default_factory=list)
    flanges: List[np.ndarray] = field(default_factory=list)
    skipped: bool = False


def _split(detections: Detections, spec: GridSpec, reference_first_id: int):
    board = {i: c for i, c in detections.items()
             if spec.first_id <= i < spec.first_id + spec.rows * spec.cols}
    refs = {i: c for i, c in detections.items() if i >= reference_first_id}
    return board, refs


def _pose_gap(a: np.ndarray, b: np.ndarray) -> Tuple[float, float]:
    return (1000.0 * float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
            rotation_angle_deg(a[:3, :3] @ b[:3, :3].T))


class CalibrationRun:
    """One run of one mode. Feed :meth:`step` every cycle."""

    def __init__(self, params: RunParams, stored: Optional[ExtrinsicsRecord],
                 fallback_world_T_cam: np.ndarray):
        self.params = params
        #: Calibration in force before the run: the file, else base_cam's cal_*.
        self.stored = stored
        self.reference_T = stored.world_T_cam if stored is not None else fallback_world_T_cam
        self.state = WARTEN
        self.result: Optional[ExtrinsicsRecord] = None
        self.report: Dict[str, float] = {}
        self.moved: Optional[bool] = None
        self._events: List[str] = []
        self._camera: Optional[Camera] = None
        self._start: Optional[np.ndarray] = None
        self._target: Optional[np.ndarray] = None
        self._poses = []
        self._index = 0
        self._capture: Optional[_Capture] = None
        self._state_since = 0.0
        self._still: List[Tuple[float, np.ndarray]] = []
        self._samples: Dict[str, List[Sample]] = {"kalibrieren": [], "pruefen": [], "wiederholen": []}
        self._missing = 0
        self._spec = params.grid
        self._ref_frames: List[Detections] = []
        self._depth: Optional[np.ndarray] = None
        self._moved_robot = False
        self._return_then_fail = False
        self._static_detections: Detections = {}
        self._waiting_logged = False
        problems = params.problems()
        if problems:
            self._fail("; ".join(problems))

    # -- interface ------------------------------------------------------------

    @property
    def moves_robot(self) -> bool:
        return self.params.mode == "stufe1"

    @property
    def running(self) -> bool:
        return self.state not in (FERTIG, FEHLER)

    @property
    def failed(self) -> bool:
        return self.state == FEHLER

    @property
    def progress(self) -> Tuple[int, int]:
        return self._index, len(self._poses)

    def wants_frames(self) -> bool:
        return self.state in (WARTEN, START, AUFNAHME)

    @property
    def needs_depth(self) -> bool:
        """One depth image is enough (belt plane); the component converts only then."""
        return self._depth is None and self.wants_frames()

    def pop_events(self) -> List[str]:
        events, self._events = self._events, []
        return events

    def step(self, now: float, flange: Optional[np.ndarray], camera: Optional[Camera],
             frame: Optional[Frame]) -> Optional[np.ndarray]:
        """Advance; returns the flange target to command (None: send nothing)."""
        if camera is not None:
            self._camera = camera
        if self.state == WARTEN:
            self._wait(now, flange, frame)
        elif self.state in (START, AUFNAHME):
            self._collect(now, flange, frame)
        elif self.state == FAHREN:
            self._drive(now, flange)
        elif self.state == BERUHIGEN:
            self._settle(now, flange)
        elif self.state == RUECKFAHRT:
            self._return(now, flange)
        if self.state == RECHNEN:
            self._solve()
        return self._target if self.moves_robot else None

    # -- states ---------------------------------------------------------------

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self._state_since = now

    def _fail(self, message: str) -> None:
        self._events.append(f"FEHLER: {message}")
        if self._moved_robot and self._start is not None and self.state != RUECKFAHRT:
            # drive back first, then stop there
            self._target = self._start
            self._return_then_fail = True
            self.state = RUECKFAHRT
            return
        self.state = FEHLER

    def _wait(self, now: float, flange, frame) -> None:
        missing = []
        if self._camera is None:
            missing.append("Kamera-Intrinsik")
        if frame is None:
            missing.append("Kamerabild")
        if self.moves_robot and flange is None:
            missing.append("Roboterzustand")
        if self.moves_robot and flange is not None and self._start is None:
            self._start = flange.copy()
            self._target = self._start  # hold where the robot stands
        if missing:
            if not self._waiting_logged:
                self._events.append("warte auf " + ", ".join(missing))
                self._waiting_logged = True
            return
        self._capture = _Capture()
        self._enter(START if self.moves_robot else AUFNAHME, now)
        self._collect(now, flange, frame)

    def _collect(self, now: float, flange, frame) -> None:
        cap = self._capture
        if flange is not None:
            cap.flanges.append(flange)
        if frame is None:
            return
        if not cap.skipped:  # exposed while the capture was being set up
            cap.skipped = True
            return
        cap.frames.append(frame)
        if self._depth is None and frame.depth_m is not None:
            self._depth = frame.depth_m
        _, refs = _split(frame.detections, self._spec, self.params.reference_first_id)
        if refs:
            self._ref_frames.append(refs)
        if len(cap.frames) < self.params.frames_per_pose:
            return
        detections = average_detections([f.detections for f in cap.frames])
        mean_flange = average_transforms(cap.flanges) if cap.flanges else None
        self._capture = None
        if self.state == START:
            self._plan(now, detections)
        elif self.moves_robot:
            self._store_sample(now, detections, mean_flange)
        else:
            self._enter(RECHNEN, now)
            self._static_detections = detections

    def _plan(self, now: float, detections: Detections) -> None:
        p, cam = self.params, self._camera
        board, _ = _split(detections, p.grid, p.reference_first_id)
        if len(board) < p.min_tags:
            self._fail(f"Board nicht im Bild der Basiskamera ({len(board)} Tags, "
                       f"mindestens {p.min_tags})")
            return
        ranked = identify_layout(board, layout_candidates(p.grid), cam.K, cam.D)
        if not ranked or ranked[0][0] > 1.0:
            self._fail("Board-Layout passt nicht (Zeilen, Spalten, Taggröße, Wörterbuch prüfen)")
            return
        if len(ranked) > 1 and ranked[1][0] < 3.0 * ranked[0][0]:
            self._fail("Board-Layout nicht eindeutig")
            return
        self._spec = ranked[0][1]
        if self._spec != p.grid:
            self._events.append(f"Board-Layout erkannt: {self._spec.rows} Zeilen x "
                                f"{self._spec.cols} Spalten, Ursprung {self._spec.origin}")
        obj, img = grid_points(self._spec, board)
        cam_T_board, _ = solve_pnp(obj, img, cam.K, cam.D)
        w, h = self._spec.size_m
        centre = self.reference_T @ cam_T_board @ np.array([w / 2, h / 2, 0.0, 1.0])
        tool_z = self._start[:3, 2]
        pivot = float(np.dot(centre[:3] - self._start[:3, 3], tool_z))
        if not 0.15 <= pivot <= 0.7:
            self._fail(f"Board-Mitte liegt {pivot:.2f} m vor dem Flansch entlang der "
                       "Werkzeugachse - erwartet 0,15 ... 0,7 m. Greift der Greifer das Board?")
            return
        plan = replace(p.plan, pivot_along_tool_m=pivot)
        poses = plan_poses(self._start, plan)
        problems = plan_problems(self._start, poses, plan)
        if problems:
            self._fail("Plan nicht fahrbar, der Roboter bleibt stehen: " + "; ".join(problems[:4])
                       + (f" (+{len(problems) - 4} weitere)" if len(problems) > 4 else ""))
            return
        self._poses = poses
        self._index = 0
        self._events.append(f"Board-Mitte {pivot:.3f} m vor dem Flansch; {len(poses)} Posen geplant")
        self._next_pose(now)

    def _next_pose(self, now: float) -> None:
        if self._index >= len(self._poses):
            self._target = self._start
            self._enter(RUECKFAHRT, now)
            return
        self._target = self._poses[self._index].world_T_flange
        self._moved_robot = True
        self._enter(FAHREN, now)

    def _timed_out(self, now: float) -> bool:
        return now - self._state_since > self.params.pose_timeout_s

    def _drive(self, now: float, flange) -> None:
        if flange is not None:
            dist, angle = _pose_gap(flange, self._target)
            if dist <= self.params.reach_mm and angle <= self.params.reach_deg:
                self._still = []
                self._enter(BERUHIGEN, now)
                return
        if self._timed_out(now):
            self._fail(f"Pose {self._index + 1} nicht erreicht")

    def _settle(self, now: float, flange) -> None:
        if flange is not None:
            self._still.append((now, flange[:3, 3].copy()))
        self._still = [(t, p) for t, p in self._still if now - t <= self.params.settle_s]
        window = [p for _, p in self._still]
        if now - self._state_since >= self.params.settle_s and window:
            spread = 1000.0 * float(np.max(np.linalg.norm(np.array(window) - window[-1], axis=1)))
            if spread <= self.params.still_mm:
                self._capture = _Capture()
                self._enter(AUFNAHME, now)
                return
        if self._timed_out(now):
            self._fail(f"Roboter kommt an Pose {self._index + 1} nicht zur Ruhe")

    def _store_sample(self, now: float, detections: Detections, flange) -> None:
        pose = self._poses[self._index]
        board, _ = _split(detections, self._spec, self.params.reference_first_id)
        obj, img = grid_points(self._spec, board)
        if len(board) < self.params.min_tags or flange is None:
            self._missing += 1
            self._events.append(f"Pose {self._index + 1}: nur {len(board)} Tags - ausgelassen")
            if self._missing > self.params.max_missing_poses:
                self._fail(f"{self._missing} Posen ohne ausreichende Sicht aufs Board")
                return
        else:
            self._samples[pose.role].append(Sample(flange, obj, img))
        self._index += 1
        self._next_pose(now)

    def _return(self, now: float, flange) -> None:
        if flange is not None:
            dist, angle = _pose_gap(flange, self._start)
            if dist <= self.params.reach_mm and angle <= self.params.reach_deg:
                if self._return_then_fail:
                    self.state = FEHLER
                else:
                    self._enter(RECHNEN, now)
                return
        if self._timed_out(now):
            self._events.append("FEHLER: Startpose auf dem Rückweg nicht erreicht")
            self.state = FEHLER

    # -- solving --------------------------------------------------------------

    def _solve(self) -> None:
        try:
            if self.moves_robot:
                self._solve_stage1()
            else:
                self._solve_static()
        except (ValueError, np.linalg.LinAlgError) as exc:
            self._fail(f"Rechnung: {exc}")

    def _belt(self) -> np.ndarray:
        cam = self._camera
        return belt_sample_points(self.reference_T, cam.K, cam.size, self.params.plan.belt_z_m,
                                  roi=self.params.roi)

    def _plane_report(self, world_T_cam: np.ndarray) -> Dict[str, float]:
        if self._depth is None:
            return {}
        try:
            plane = fit_plane(depth_points(self._depth, self._camera.K, self.params.roi))
        except ValueError:
            return {}
        tilt, height = belt_plane_check(world_T_cam, plane, self.params.plan.belt_z_m)
        return {"ebene_neigung_grad": round(tilt, 3), "ebene_hoehe_mm": round(height, 2),
                "ebene_rest_mm": round(plane.rms_mm, 2)}

    def _intrinsics(self) -> dict:
        cam = self._camera
        return {"K": np.round(cam.K, 4).tolist(), "D": np.round(np.ravel(cam.D), 6).tolist(),
                "bild": list(cam.size)}

    def _solve_stage1(self) -> None:
        p, cam = self.params, self._camera
        cal = self._samples["kalibrieren"]
        result = solve_stage1(cal, cam.K, cam.D)
        X = result.world_T_cam
        check = validate(result, self._samples["pruefen"], cam.K, cam.D)
        slip_mm = slip_deg = float("nan")
        if cal and self._samples["wiederholen"]:
            slip_mm, slip_deg = board_slip(X, cal[0], self._samples["wiederholen"][0], cam.K, cam.D)
        shift = belt_displacement_mm(self.reference_T, X, self._belt())
        angle = rotation_angle_deg(X[:3, :3] @ self.reference_T[:3, :3].T)
        references = self._measure_references(X)
        self.report = {
            "posen": len(cal) - result.poses_dropped,
            "posen_ausgelassen": self._missing + result.poses_dropped,
            "punkte": result.points_used,
            "punkte_verworfen": result.points_rejected,
            "bildfehler_px": round(result.rms_px, 3),
            "pruefposen_px": round(check.rms_px, 3),
            "pruefposen_mm_mittel": round(check.mean_mm, 2),
            "pruefposen_mm_max": round(check.max_mm, 2),
            "rutschen_mm": round(slip_mm, 2),
            "rutschen_grad": round(slip_deg, 3),
            "streuung_position_mm": round(result.position_std_mm, 3),
            "streuung_drehung_grad": round(result.rotation_std_deg, 4),
            "gegen_vorher_band_mm": round(shift, 2),
            "gegen_vorher_grad": round(angle, 3),
            "referenzmarken": len(references),
            **self._plane_report(X),
        }
        gates = []
        if result.rms_px > p.max_rms_px:
            gates.append(f"Bildfehler {result.rms_px:.2f} px > {p.max_rms_px}")
        if not check.mean_mm <= p.max_holdout_mm:
            gates.append(f"Prüfposen {check.mean_mm:.1f} mm > {p.max_holdout_mm}")
        if not (slip_mm <= p.max_slip_mm and slip_deg <= p.max_slip_deg):
            gates.append(f"Board im Greifer verrutscht ({slip_mm:.2f} mm / {slip_deg:.2f} Grad)")
        self._events.append("Ergebnis: " + ", ".join(f"{k} {v}" for k, v in self.report.items()))
        if gates:
            self._fail("Gütegrenzen verletzt, nichts geschrieben: " + "; ".join(gates))
            return
        self.result = ExtrinsicsRecord(
            world_T_cam=X, method="stufe1", created=_now_iso(),
            quality={**self.report, "startverfahren": result.init_method,
                     "intrinsik": self._intrinsics()},
            reference_size_m=p.reference_size_m if references else 0.0,
            references=references, grid=self._spec.to_dict())
        self.state = FERTIG

    def _measure_references(self, X: np.ndarray) -> Dict[int, np.ndarray]:
        if not self._ref_frames:
            return {}
        seen = average_detections(self._ref_frames)
        out = {}
        for tag_id, corners in seen.items():
            try:
                out[tag_id] = reference_corners_world(X, corners, self.params.reference_size_m,
                                                      self._camera.K, self._camera.D)
            except ValueError:
                continue
        if out:
            self._events.append(f"Referenzmarken vermessen: {sorted(out)}")
        return out

    def _solve_static(self) -> None:
        p, cam = self.params, self._camera
        if self.stored is None or len(self.stored.references) < p.min_reference_tags:
            raise ValueError("die Kalibrierdatei enthält keine Referenzmarken - erst Stufe 1 "
                             "mit Referenzmarken im Bild")
        _, refs = _split(self._static_detections, self._spec, p.reference_first_id)
        X, err, used = solve_from_references(self.stored.references, refs, cam.K, cam.D,
                                             guess=self.stored.world_T_cam,
                                             min_tags=p.min_reference_tags)
        self.moved, shift, angle = camera_moved(self.stored.world_T_cam, X, self._belt(), p.limits)
        self.report = {
            "referenzmarken": len(used),
            "bildfehler_px": round(err, 3),
            "gegen_datei_band_mm": round(shift, 2),
            "gegen_datei_grad": round(angle, 3),
            "kamera_bewegt": bool(self.moved),
            **self._plane_report(X),
        }
        verdict = ("Kamera hat sich bewegt - nachkalibrieren" if self.moved
                   else "Kamera unverändert")
        self._events.append(f"{verdict}: " + ", ".join(f"{k} {v}" for k, v in self.report.items()))
        if p.mode == "stufe2":
            self.result = ExtrinsicsRecord(
                world_T_cam=X, method="stufe2", created=_now_iso(),
                quality={**self.report, "intrinsik": self._intrinsics(),
                         "grundlage": self.stored.created},
                reference_size_m=self.stored.reference_size_m,
                references=self.stored.references, grid=self.stored.grid)
        self.state = FERTIG


def _now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()
