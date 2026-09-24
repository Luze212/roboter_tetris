"""AICA lifecycle component `object_follower` -- stages 4a to 4d.

Thin shell around :class:`roboter_tetris.follower_logic.FollowerCore`. It feeds
the core with the flange pose (``robot_state``, also into the core's ring
buffer), the target (S4), the robot camera (S2) and the gripper (S9), and turns
the core's flange target into a ``cartesian_pose`` for the attractor input of
the ``SignalPointAttractor`` (S6).
"""

import math
from dataclasses import fields
from numbers import Real

from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
import state_representation as sr
from std_msgs.msg import Bool, Float64MultiArray

from .contracts import (
    S6_REFERENCE_FRAME, STATE_ABORT, STATE_DESCEND, STATE_FOLLOW, ContractError,
    pack_follower_status, pack_picked_id, unpack_object_position, unpack_target,
)
from .follower_logic import (
    REQUIRED, FollowerCore, FollowerParams, GripperFeedback, Pose,
)

WARN_LOG_PERIOD_S = 2.0
ROBOT_STATE_MAX_AGE_S = 0.2

#: Operator-facing description of every parameter; the JSON says the same.
DESCRIPTIONS = {
    "ws": "Arbeitsraum des Flansches in world (m), B10 - nicht die Greifzone. KEIN "
          "DEFAULT: laut Safety/README erst mit dokumentierter Festlegung eintragen.",
    "observe": "Beobachtungspose des Flansches in world (m), B8. Orientierung fest "
               "senkrecht. KEIN DEFAULT. Auch die Höhe, auf der gefolgt wird.",
    "observe_yaw_deg": "Gierwinkel der Beobachtungspose (Grad): Richtung der "
                       "Werkzeug-x-Achse in world. KEIN DEFAULT (B8).",
    "transfer_height_m": "Freihöhe des Flansches (m) für Transfer und Abbruchpfad: Band "
                         "53,6 + stehender Klotz 100 + untere Hälfte eines gehaltenen "
                         "Klotzes 50 + Griffpunkt 235 + Luft 50 mm (D12, Nachtrag 10 / J1).",
    "pose_tolerance_m": "Ab diesem Abstand (m) gilt eine stehende Zielpose als erreicht.",
    "max_target_jump_m": "Sicherheitsgate: springt die Zielpose innerhalb eines Zustands "
                         "weiter (m), sind die Daten falsch - Abbruch. (D15)",
    "lead_time_s": "Vorhalt als Zeit (s), Theorie 1/K. In 4b über err_laengs einmessen (B4).",
    "latency_compensation_s": "Zusätzlicher Vorhersagehorizont (s) (D7).",
    "max_extrapolation_s": "Deckel der Vorhersage (s), Gate-Prüfung 3 (D14). Muss über "
                           "dem Alter von S4 vor der nächsten Messung liegen - gemessen "
                           "bis ~0,5 s (Nachtrag 13 / L2).",
    "target_timeout_s": "S4-Zeitstempel steht so lange (s) still -> Abbruch, Gate-Prüfung 2. "
                        "Über dem Abstand zweier Messungen der Basiskamera (bis ~0,5 s).",
    "timeout_approach_s": "ANFAHREN darf so lange (s) über die erwartete Ankunft des "
                          "Klotzes an der Zone hinaus dauern (D5).",
    "timeout_track_s": "Höchstdauer (s) in FOLGEN bis zum Absenken (D5). Zum Einmessen "
                       "des Vorhalts auf 10 s oder mehr.",
    "use_block_orientation": "Modus 2: entlang des Klotzwinkels greifen statt entlang "
                             "der geschätzten Bandrichtung (Modus 1).",
    "orientation_quality_min": "Mindestgüte (S4 Feld 16, 0...1) für den Klotzwinkel (D11).",
    "gripper_yaw_offset_deg": "Montagewinkel der Backen gegen die Werkzeug-x-Achse "
                              "(Grad). Nicht gemessen (D23).",
    "weight_along": "Gewicht der Roboterkamera entlang des Bandes, 0...1. 0 = nur "
                    "Basiskamera (Nachtrag 6 / Z10).",
    "weight_across": "Gewicht der Roboterkamera quer zum Band, 0...1.",
    "w_ramp_s": "Ein- und Ausblenden der Gewichte (s) (D8).",
    "correction_filter_window": "Gleitender Mittelwert über so viele angenommene "
                                "Korrekturen.",
    "max_correction_m": "Größere Korrektur (m) gilt als anderer Klotz im Bild - "
                        "verwerfen, w -> 0 (R4, D16).",
    "robot_cam_max_age_s": "Ältere Messung (s) -> Rückfall auf w = 0 (D6).",
    "require_robot_cam_for_grasp": "Absenken nur mit voll eingeblendeter Roboterkamera "
                                   "(D9/D10). Vorerst aus (Nachtrag 6 / Z10).",
    "handeye_x": "Hand-Auge Flansch -> Kamera, x (m). Eingemessen 23.09.2026 (Nachtrag 13 / L11).",
    "handeye_y": "Hand-Auge Flansch -> Kamera, y (m). Eingemessen 23.09.2026 (Nachtrag 13 / L11).",
    "handeye_z": "Hand-Auge Flansch -> Kamera, z (m). Eingemessen 23.09.2026 (Nachtrag 13 / L11).",
    "handeye_roll_deg": "Hand-Auge, Rollwinkel (Grad), R = Rz * Ry * Rx (Nachtrag 13 / L11).",
    "handeye_pitch_deg": "Hand-Auge, Nickwinkel (Grad) (Nachtrag 13 / L11).",
    "handeye_yaw_deg": "Hand-Auge, Gierwinkel (Grad) (Nachtrag 13 / L11).",
    "belt_surface_z_m": "Höhe der Bandoberfläche in world (m), gemessen (B17).",
    "flange_to_grip_point_m": "Flansch -> Griffpunkt (m), gemessen 0,235 - nicht der TCP "
                              "der UR-Steuerung (215 mm) (Nachtrag 6 / Z7).",
    "min_grip_height_m": "Untere Grenze der Greifhöhe über dem Band (m), Mitte der Auflage. "
                         "0,021: liegt über der Arbeitsraum-Untergrenze (geschlossene "
                         "Backenspitze 10 mm über dem Band, Nachtrag 13 / L14).",
    "descend_speed_mps": "Sinkgeschwindigkeit (m/s). Gekoppelt an t_descend_s des "
                         "priority_handler (Nachtrag 10 / J2).",
    "lift_clearance_m": "So hoch (m) über die Greifhöhe fährt der Roboter beim Heben "
                        "noch mit dem Band mit.",
    "tol_along_m": "Greif-Freigabe: Abweichung entlang des Bandes (m) (D3, B18).",
    "tol_across_m": "Greif-Freigabe: Abweichung quer zum Band (m) (D3, B18).",
    "tol_z_m": "Greif-Freigabe und Greifhöhe: Höhenabweichung (m).",
    "tol_yaw_rad": "Greif-Freigabe: Gierabweichung (rad) (Nachtrag 2 / F1).",
    "stable_cycles": "So viele Takte muss die Freigabe halten, bevor abgesenkt wird.",
    "timeout_grasp_s": "Greifer meldet so lange (s) keinen Klotz -> Fehlgriff.",
    "timeout_place_s": "Höchstdauer (s) für Heben und für die Fahrt zur Ablagepose.",
    "timeout_release_s": "Höchstdauer (s) für das Öffnen des Greifers.",
    "place_x": "Ablagepose über der Kiste, x (m), geteacht (B9) - Gegenprobe offen.",
    "place_y": "Ablagepose, y (m) (B9).",
    "place_z": "Ablagepose, z (m) (B9).",
    "place_yaw_deg": "Ablagepose, Gierwinkel (Grad) (B9).",
    "robot_state_max_age_s": "Ist der letzte robot_state älter (s), gilt die Flanschpose "
                             "als unbekannt; keine neue Zielpose.",
}
_BOUNDED_01 = ("orientation_quality_min", "weight_along", "weight_across")
_NON_NEGATIVE = ("lead_time_s", "latency_compensation_s", "min_grip_height_m",
                 "lift_clearance_m")
_FREE = ("gripper_yaw_offset_deg", "handeye_x", "handeye_y", "handeye_z",
         "handeye_roll_deg", "handeye_pitch_deg", "handeye_yaw_deg", "place_x",
         "place_y", "place_z", "place_yaw_deg", "belt_surface_z_m")


def _description(name: str) -> str:
    if name in DESCRIPTIONS:
        return DESCRIPTIONS[name]
    return DESCRIPTIONS[name.split("_")[0]]


def _parameter_type(default):
    if isinstance(default, bool):
        return sr.ParameterType.BOOL
    if isinstance(default, int):
        return sr.ParameterType.INT
    return sr.ParameterType.DOUBLE


class ObjectFollower(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # -- Parameters: one per FollowerParams field, plus the shell's own ------
        for f in fields(FollowerParams):
            if f.name in REQUIRED:
                parameter = sr.Parameter(f.name, sr.ParameterType.DOUBLE)
            else:
                parameter = sr.Parameter(f.name, f.default, _parameter_type(f.default))
            self.add_parameter(parameter, _description(f.name))
        self.add_parameter(
            sr.Parameter("robot_state_max_age_s", ROBOT_STATE_MAX_AGE_S,
                         sr.ParameterType.DOUBLE),
            DESCRIPTIONS["robot_state_max_age_s"])

        # -- Inputs -----------------------------------------------------------------
        self._robot_state = sr.CartesianState()
        self.add_input("robot_state", "_robot_state", EncodedState,
                       user_callback=self._on_robot_state)
        self._target_in = []
        self.add_input("target", "_target_in", Float64MultiArray)
        self._object_position_in = []
        self.add_input("object_position", "_object_position_in", Float64MultiArray)
        self._gripper_motion_done = False
        self.add_input("gripper_motion_done", "_gripper_motion_done", Bool)
        self._gripper_has_object = False
        self.add_input("gripper_has_object", "_gripper_has_object", Bool)

        # -- Outputs ----------------------------------------------------------------
        # Empty until the first gated target: an empty state is not published,
        # so the attractor keeps whatever it had.
        self._target_pose = sr.CartesianPose("target_pose", S6_REFERENCE_FRAME)
        self.add_output("target_pose", "_target_pose", EncodedState,
                        MessageType.CARTESIAN_POSE_MESSAGE)
        self._gripper_close = False
        self.add_output("gripper_close", "_gripper_close", Bool)
        # S7 from activation on: seq 0 = no attempt yet (Nachtrag 7 / H5).
        self._picked_id = pack_picked_id(0, 0, 0)
        self.add_output("picked_id", "_picked_id", Float64MultiArray)
        self._follower_status = pack_follower_status(0.0, STATE_ABORT, 0, 0, 0, 0, 0)
        self.add_output("follower_status", "_follower_status", Float64MultiArray)

        # -- Predicates -------------------------------------------------------------
        self.add_predicate("is_tracking", False)
        self.add_predicate("is_holding_object", False)
        self.add_predicate("has_aborted", False)

        # -- State ------------------------------------------------------------------
        self._core = FollowerCore(FollowerParams())
        self._last_robot_state = None        # (clock time, Pose)
        self._frame_reported = False
        self._last_warn = {}

    # -- Validation -----------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            if name in REQUIRED:
                return True                   # checked as a set in on_configure
            self.get_logger().warn(f"{name} must not be empty")
            return False
        value = parameter.get_value()
        if isinstance(value, bool):
            return True
        # ``add_input`` and ``add_output`` create AICA topic parameters as
        # strings (for example ``robot_state = "/.../cartesian_state"``).
        # They are transport configuration, not FollowerParams; only the
        # component's numeric parameters belong to the checks below.
        if not isinstance(value, Real):
            return True
        if not math.isfinite(value):
            self.get_logger().warn(f"{name} must be finite")
            return False
        if name in REQUIRED or name in _FREE:
            return True
        if name in _BOUNDED_01:
            if not 0.0 <= value <= 1.0:
                self.get_logger().warn(f"{name} must lie in [0, 1]")
                return False
            return True
        if name in _NON_NEGATIVE:
            if value < 0.0:
                self.get_logger().warn(f"{name} must not be negative")
                return False
            return True
        if value <= 0:
            self.get_logger().warn(f"{name} must be positive")
            return False
        return True

    def _params(self) -> FollowerParams:
        values = {}
        for f in fields(FollowerParams):
            parameter = self.get_parameter(f.name)
            values[f.name] = None if parameter.is_empty() else parameter.get_value()
        return FollowerParams(**values)

    # -- Lifecycle ------------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        problems = self._params().problems()
        if problems:
            self.get_logger().error("object_follower: " + "; ".join(problems))
            return False
        return True

    def on_activate_callback(self) -> bool:
        self._core = FollowerCore(self._params())
        self._target_pose = sr.CartesianPose("target_pose", S6_REFERENCE_FRAME)
        self._gripper_close = False           # open on activation
        self._picked_id = pack_picked_id(0, 0, 0)
        for name in ("is_tracking", "is_holding_object", "has_aborted"):
            self.set_predicate(name, False)
        return True

    def on_deactivate_callback(self) -> bool:
        # The last target stays with the attractor: the robot finishes that
        # move and holds -- bounded, because the gate kept it in the workspace.
        # A held block stays in the gripper; the gripper component opens it at
        # its next bring-up (Thema 7).
        return True

    # -- Helpers --------------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _warn_throttled(self, key: str, message: str) -> None:
        now = self.get_clock().now()
        last = self._last_warn.get(key)
        if last is None or (now - last).nanoseconds / 1e9 >= WARN_LOG_PERIOD_S:
            self.get_logger().warn(message)
            self._last_warn[key] = now

    def _on_robot_state(self) -> None:
        """Every robot_state: into the core's ring buffer, stamped on arrival."""
        if self._robot_state.is_empty():
            return
        position = self._robot_state.get_position()
        orientation = self._robot_state.get_orientation_coefficients()   # w, x, y, z
        pose = Pose(*(float(v) for v in position), *(float(v) for v in orientation))
        now = self._now_s()
        self._last_robot_state = (now, pose)
        self._core.history.add(now, pose)
        if not self._frame_reported:
            self._frame_reported = True
            frame = self._robot_state.get_reference_frame()
            log = (self.get_logger().info if frame == S6_REFERENCE_FRAME
                   else self.get_logger().warn)
            log(f"object_follower: robot_state kommt in '{frame}', "
                f"target_pose geht in '{S6_REFERENCE_FRAME}'")

    def _current_flange(self):
        if self._last_robot_state is None:
            return None
        received, pose = self._last_robot_state
        max_age = self.get_parameter("robot_state_max_age_s").get_value()
        return pose if self._now_s() - received <= max_age else None

    def _read(self, key, unpack, value):
        try:
            return unpack(value)
        except ContractError as exc:
            # Malformed counts as missing: a running attempt aborts on S4.
            self._warn_throttled(key, f"object_follower: {key} verworfen - {exc}")
            return None

    # -- Periodic processing ----------------------------------------------------------

    def on_step_callback(self):
        params = self._params()
        problems = params.problems()
        if problems:
            self._warn_throttled("params", "object_follower hält an: " + "; ".join(problems))
            return
        self._core.params = params

        flange = self._current_flange()
        if flange is None:
            self._warn_throttled("robot_state", "object_follower: kein frischer "
                                 "robot_state - keine neue Zielpose")
        target = self._read("target", unpack_target, self._target_in)
        measurement = self._read("object_position", unpack_object_position,
                                 self._object_position_in)
        gripper = GripperFeedback(bool(self._gripper_motion_done),
                                  bool(self._gripper_has_object))
        now = self._now_s()
        output = self._core.step(flange, target, now, measurement, gripper)
        for line in self._core.pop_events():
            self.get_logger().info(f"object_follower: {line}")

        if output.target is not None:
            self._target_pose.set_position(output.target.x, output.target.y,
                                           output.target.z)
            self._target_pose.set_orientation(list(output.target.orientation))
        self._gripper_close = output.gripper_close
        self._picked_id = pack_picked_id(*self._core.picked)
        status = output.status
        self._follower_status = pack_follower_status(
            now, output.state, status.target_id, status.err_along,
            status.err_across, status.err_z, status.w_effective)
        self.set_predicate("is_tracking", output.state in (STATE_FOLLOW, STATE_DESCEND))
        self.set_predicate("is_holding_object", self._core.holding)
        self.set_predicate("has_aborted", self._core.has_aborted)
