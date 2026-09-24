"""AICA lifecycle component `priority_handler` -- target selection (project goal 4).

Thin shell around :class:`roboter_tetris.target_selection.TargetSelector`, which
holds the whole rule set and is tested on its own. This file only wires it to
S3 (``tracks``), S7 (``picked_id``) and the flange position (``robot_state``),
and publishes S4 (``target``) and S5 (``not_pickable``).
"""

from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
import state_representation as sr
from std_msgs.msg import Float64MultiArray

from .contracts import (
    ContractError, pack_not_pickable, pack_target, unpack_picked_id,
    unpack_tracks,
)
from .target_selection import SelectorParams, TargetSelector

# No new S3 frame for this long -> withdraw the target (wall-clock seconds).
# A target on dead data is more dangerous than no target.
STALE_TIMEOUT_S = 1.0   # s; base_cam delivers ~7/s, gaps up to ~0.5 s (Nachtrag 13 / L2)
WARN_LOG_PERIOD_S = 2.0

_ZONE = ("zone_x_min", "zone_x_max", "zone_y_min", "zone_y_max")
_POSITIVE = ("attractor_v_max_mps", "attractor_gain", "grasp_time_margin",
             "reach_safety_factor", "max_gripper_opening_m")
_NON_NEGATIVE = ("t_settle_s", "t_descend_s", "t_grasp_s", "t_lift_s",
                 "min_graspable_height_m", "gripper_margin_m")


class PriorityHandler(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        d = SelectorParams()

        # -- Parameters (operator-facing descriptions) ----------------------------
        zone_note = (" Festgelegt 23.09.2026 (B19, Nachtrag 13 / L14): im Arbeitsraum, "
                     "außerhalb des Bildes der Basiskamera; y +0,40 ... -0,22 bei "
                     "Bandende -0,32.")
        self.add_parameter(sr.Parameter("zone_x_min", d.zone_x_min, sr.ParameterType.DOUBLE),
                           "Greifzone in world, untere x-Grenze (m)." + zone_note)
        self.add_parameter(sr.Parameter("zone_x_max", d.zone_x_max, sr.ParameterType.DOUBLE),
                           "Greifzone in world, obere x-Grenze (m)." + zone_note)
        self.add_parameter(sr.Parameter("zone_y_min", d.zone_y_min, sr.ParameterType.DOUBLE),
                           "Greifzone in world, untere y-Grenze (m)." + zone_note)
        self.add_parameter(sr.Parameter("zone_y_max", d.zone_y_max, sr.ParameterType.DOUBLE),
                           "Greifzone in world, obere y-Grenze (m)." + zone_note)
        self.add_parameter(
            sr.Parameter("attractor_v_max_mps", d.attractor_v_max_mps, sr.ParameterType.DOUBLE),
            "Höchste Fahrgeschwindigkeit beim Anfahren (m/s) — der kleinere Wert aus "
            "Attractor und IK-Controller. Bindend ist meist der IK-Controller (A1).")
        self.add_parameter(
            sr.Parameter("attractor_gain", d.attractor_gain, sr.ParameterType.DOUBLE),
            "Verstärkung K des Signal Point Attractors, wie dort eingestellt. Das "
            "Einschwingen dauert etwa 3/K.")
        self.add_parameter(
            sr.Parameter("t_settle_s", d.t_settle_s, sr.ParameterType.DOUBLE),
            "Einschwingen des Followers bis zur Greif-Freigabe (s), zusätzlich zu 3/K. "
            "Gemessen 0,23-0,33 s; mit 0,4 trifft die Rechnung den Rückweg von der "
            "Ablagepose (Nachtrag 13 / L23, L24).")
        self.add_parameter(
            sr.Parameter("t_descend_s", d.t_descend_s, sr.ParameterType.DOUBLE),
            "Dauer des Absenkens (s): (observe_z - Greifhöhe) / descend_speed_mps des "
            "Followers plus Einschwingen. Legt mit Greifen und Heben die Greifebene fest. (D22)")
        self.add_parameter(
            sr.Parameter("t_grasp_s", d.t_grasp_s, sr.ParameterType.DOUBLE),
            "Dauer des Greifens bis motion_done (s). (D22)")
        self.add_parameter(
            sr.Parameter("t_lift_s", d.t_lift_s, sr.ParameterType.DOUBLE),
            "Dauer des Hebens bis lift_clearance_m (s) — so lange fährt der Roboter "
            "noch mit dem Band. (D22)")
        self.add_parameter(
            sr.Parameter("grasp_time_margin", d.grasp_time_margin, sr.ParameterType.DOUBLE),
            "Aufschlag auf die Dauer des Greifprozesses bei der Lage der Greifebene. (D22)")
        self.add_parameter(
            sr.Parameter("reach_safety_factor", d.reach_safety_factor, sr.ParameterType.DOUBLE),
            "Ein Klotz ist erreichbar, wenn die Zeit bis zur Greifebene das "
            "so-Vielfache der Anfahrzeit beträgt. Unter 1,0 werden Klötze gewählt, "
            "die an der Greifebene verloren gehen (Nachtrag 13 / L24).")
        self.add_parameter(
            sr.Parameter("min_graspable_height_m", d.min_graspable_height_m, sr.ParameterType.DOUBLE),
            "Flachere Klötze werden nicht gewählt (m). 0,02: flache 25-mm-Klötze greift "
            "der Follower an seiner Untergrenze min_grip_height_m (B15, Nachtrag 13 / L24).")
        self.add_parameter(
            sr.Parameter("max_gripper_opening_m", d.max_gripper_opening_m, sr.ParameterType.DOUBLE),
            "Backenabstand bei offenem Greifer (m), gemessen 0,127. (B16)")
        self.add_parameter(
            sr.Parameter("gripper_margin_m", d.gripper_margin_m, sr.ParameterType.DOUBLE),
            "Reserve zur Öffnungsweite (m). Die Diagonale des Klotzes muss in "
            "Öffnung minus Reserve passen.")

        # -- Inputs / outputs (std_msgs signals are plain Python values) ---------
        self._tracks_in = []
        self.add_input("tracks", "_tracks_in", Float64MultiArray)
        self._picked_in = []
        self.add_input("picked_id", "_picked_in", Float64MultiArray)
        self._robot_state = sr.CartesianState()
        self.add_input("robot_state", "_robot_state", EncodedState)

        # Never empty: before the first frame t = 0 and no target.
        self._target = pack_target(0.0, False, 0.0, 0.0, (0.0, 0.0))
        self.add_output("target", "_target", Float64MultiArray)
        self._not_pickable = pack_not_pickable(0.0, [])
        self.add_output("not_pickable", "_not_pickable", Float64MultiArray)

        # -- Predicates -----------------------------------------------------------
        self.add_predicate("has_target", False)
        self.add_predicate("zone_empty", True)
        self.add_predicate("is_zone_feasible", False)

        # -- State ----------------------------------------------------------------
        self._selector = TargetSelector()
        self._last_msg = None
        self._last_frame_walltime = None
        self._stale = False
        self._last_warn = {}

    # -- Validation ---------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        value = parameter.get_value()
        if name in _POSITIVE and value <= 0.0:
            self.get_logger().warn(f"{name} must be positive")
            return False
        if name in _NON_NEGATIVE and value < 0.0:
            self.get_logger().warn(f"{name} must not be negative")
            return False
        return True

    def _params(self) -> SelectorParams:
        names = _ZONE + _POSITIVE + _NON_NEGATIVE
        return SelectorParams(**{n: self.get_parameter(n).get_value() for n in names})

    def _zone_is_valid(self, params: SelectorParams) -> bool:
        return (params.zone_x_min < params.zone_x_max
                and params.zone_y_min < params.zone_y_max)

    # -- Lifecycle ----------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        if not self._zone_is_valid(self._params()):
            self.get_logger().error(
                "Greifzone ist leer: zone_x_min < zone_x_max und "
                "zone_y_min < zone_y_max erforderlich")
            return False
        return True

    def on_activate_callback(self) -> bool:
        self._selector = TargetSelector(self._params())
        self._last_msg = None
        self._last_frame_walltime = None
        self._stale = False
        self._target = pack_target(0.0, False, 0.0, 0.0, (0.0, 0.0))
        self._not_pickable = pack_not_pickable(0.0, [])
        self.set_predicate("has_target", False)
        self.set_predicate("zone_empty", True)
        self.set_predicate("is_zone_feasible", False)
        return True

    def on_deactivate_callback(self) -> bool:
        # Outputs are not published outside ACTIVE; the follower sees the S4
        # timestamp stop and aborts by its own rule.
        self._selector.withdraw("deaktiviert")
        self._log_events()
        self.set_predicate("has_target", False)
        return True

    # -- Helpers ------------------------------------------------------------------

    def _warn_throttled(self, key: str, message: str) -> None:
        now = self.get_clock().now()
        last = self._last_warn.get(key)
        if last is None or (now - last).nanoseconds / 1e9 >= WARN_LOG_PERIOD_S:
            self.get_logger().warn(message)
            self._last_warn[key] = now

    def _log_events(self) -> None:
        for line in self._selector.pop_events():
            self.get_logger().info(f"priority_handler: {line}")

    def _flange_xy(self):
        if self._robot_state.is_empty():
            return None
        position = self._robot_state.get_position()
        return (float(position[0]), float(position[1]))

    def _input_is_stale(self) -> bool:
        if self._last_frame_walltime is None:
            return False
        age_s = (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9
        return age_s > STALE_TIMEOUT_S

    def _publish(self, selection) -> None:
        self._target = selection.target
        self._not_pickable = selection.not_pickable
        self.set_predicate("has_target", self._selector.locked_id is not None)
        self.set_predicate("zone_empty", selection.zone_empty)
        frame = selection.frame
        self.set_predicate("is_zone_feasible", frame is not None and frame.feasible)
        if frame is not None and not frame.feasible:
            self._warn_throttled(
                "feasible",
                f"Greifzone zu kurz für {frame.speed:.3f} m/s: Greifebene liegt "
                f"{frame.zone_upstream - frame.grasp_plane:.3f} m vor zone_upstream "
                f"— es wird nichts gewählt (B19)")

    # -- Periodic processing --------------------------------------------------------

    def on_step_callback(self):
        self._selector.params = self._params()
        if not self._zone_is_valid(self._selector.params):
            self._warn_throttled("zone", "Greifzone ist leer — es wird nichts gewählt")

        changed = False
        try:
            picked = unpack_picked_id(self._picked_in)
        except ContractError as exc:
            self._warn_throttled("picked", f"S7 verworfen — {exc}")
            picked = None
        if picked is not None:
            before = self._selector.locked_id
            self._selector.on_picked(picked)
            changed = self._selector.locked_id != before

        try:
            msg = unpack_tracks(self._tracks_in)
        except ContractError as exc:
            self._warn_throttled("tracks", f"S3 verworfen — {exc}")
            msg = None

        if msg is not None and (self._last_msg is None or msg.t != self._last_msg.t):
            self._last_msg = msg
            self._last_frame_walltime = self.get_clock().now()
            self._stale = False
            flange_xy = self._flange_xy()
            if flange_xy is None and self._selector.locked_id is None:
                self._warn_throttled("robot_state",
                                     "robot_state fehlt — ohne Flanschposition "
                                     "wird kein Ziel gewählt")
            self._publish(self._selector.step(msg, flange_xy))
        elif self._last_msg is not None and self._input_is_stale():
            if not self._stale:
                self._stale = True
                self._selector.withdraw("Eingang steht (S3-Zeitstempel)")
                self._publish(self._selector.output(self._last_msg))
        elif changed and self._last_msg is not None:
            # picked_id between two frames: release now, choose on the next frame.
            self._publish(self._selector.output(self._last_msg))

        self._log_events()
