"""AICA test component: moves the robot arm to a manually configured (x, y) coordinate.

Standalone smoke test for the "move to a given location" building block of the
future perception -> target_selection -> pick_sequencer pipeline. Deliberately
minimal for now:

* ``z_height`` and the target orientation are fixed defaults (parameters), not
  computed from a detected object.
* ``target_x``/``target_y`` are live, ``dynamic`` AICA UI parameters rather than
  a signal input. The later production component (``target_selection``) will
  replace this parameter read with a ``CartesianPose`` signal input; the
  motion-triggering logic below (broadcast a target TF frame, pulse a trigger
  predicate) is written so that swap only touches *where the target pose comes
  from*, not how it is used.

Motion path: this component only broadcasts a TF frame named
``target_frame_name`` at the configured pose via ``tf2_ros`` and pulses the
``move_requested`` predicate on a rising edge of ``move_trigger``. The actual
``set_trajectory`` call is *not* made from Python — the JTC exposes "Set
trajectory" as a graph Event (Cartesian-frame variant), wired directly in AICA
Studio from ``move_requested`` (event edge, rising-edge triggered) to that
JTC transition. Frame selection and duration are configured on that event edge
in Studio, not sent as a service payload from here. Watch the JTC's own
``has_trajectory_succeeded``/``has_trajectory_failed`` predicates in Studio to
observe the outcome — this component has no visibility into it.
"""

import math

import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Roll/pitch/yaw in degrees (extrinsic XYZ, ZYX-intrinsic equivalent) -> (x, y, z, w).

    Pure, hardware/ROS-free conversion so it stays unit-testable on its own.
    """
    r, p, y = math.radians(roll_deg) / 2.0, math.radians(pitch_deg) / 2.0, math.radians(yaw_deg) / 2.0
    cr, sr_ = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr_ * cp * cy - cr * sp * sy,
        cr * sp * cy + sr_ * cp * sy,
        cr * cp * sy - sr_ * sp * cy,
        cr * cp * cy + sr_ * sp * sy,
    )


class MoveTriggerLogic:
    """Rising-edge detector for the ``move_trigger`` input.

    No I/O — fully unit-testable without the modulo runtime, same pattern as
    ``GripperTargetLogic`` in ``robotiq_gripper.py``. ``update()`` returns True
    for exactly one step per False->True transition, which is also exactly the
    one-step pulse shape AICA Events need (predicates trigger events only on
    their own rising edge).
    """

    def __init__(self) -> None:
        self._last = False

    def reset(self) -> None:
        self._last = False

    def update(self, trigger: bool) -> bool:
        """Return True exactly once per False->True transition of ``trigger``."""
        rising_edge = bool(trigger) and not self._last
        self._last = bool(trigger)
        return rising_edge


class MoveToPoseTest(LifecycleComponent):
    """Broadcasts a configurable target TF frame and pulses a trigger predicate.

    Wire a bool into ``move_trigger`` (e.g. the existing ``true_signal``/
    ``toggle_signal`` components) to arm a move on its rising edge. In AICA
    Studio, wire the ``move_requested`` predicate (event edge) to the JTC's
    "Set trajectory" transition, configured there to use the
    ``target_frame_name`` frame this component broadcasts.
    ``target_x``/``target_y`` are live UI parameters; ``z_height`` and the
    orientation stay at their configured defaults.
    """

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        # -- Parameters -----------------------------------------------------------
        self.add_parameter(
            sr.Parameter("target_x", 0.4, sr.ParameterType.DOUBLE),
            "Ziel-X in Metern im Referenzframe 'reference_frame' (live änderbar).",
        )
        self.add_parameter(
            sr.Parameter("target_y", 0.0, sr.ParameterType.DOUBLE),
            "Ziel-Y in Metern (live änderbar).",
        )
        self.add_parameter(
            sr.Parameter("z_height", 0.3, sr.ParameterType.DOUBLE),
            "Feste Ziel-Z-Höhe in Metern für diesen Test (im vollen Programm "
            "später aus der erkannten Objekthöhe berechnet).",
        )
        self.add_parameter(
            sr.Parameter("roll_deg", 180.0, sr.ParameterType.DOUBLE),
            "Ziel-Orientierung Roll in Grad (Default: Greifer zeigt nach unten).",
        )
        self.add_parameter(
            sr.Parameter("pitch_deg", 0.0, sr.ParameterType.DOUBLE),
            "Ziel-Orientierung Pitch in Grad.",
        )
        self.add_parameter(
            sr.Parameter("yaw_deg", 0.0, sr.ParameterType.DOUBLE),
            "Ziel-Orientierung Yaw in Grad.",
        )
        self.add_parameter(
            sr.Parameter("reference_frame", "world", sr.ParameterType.STRING),
            "TF-Referenzframe, in dem 'target_x'/'target_y'/'z_height' ausgedrückt sind.",
        )
        self.add_parameter(
            sr.Parameter("target_frame_name", "test_target", sr.ParameterType.STRING),
            "Name des TF-Frames, der für die Zielpose gebroadcastet wird. Muss beim "
            "Verdrahten des 'Set trajectory'-Events in AICA Studio als Cartesian-Frame "
            "ausgewählt werden.",
        )

        # -- Inputs -----------------------------------------------------------------
        self._move_trigger = False
        self.add_input("move_trigger", "_move_trigger", Bool)

        # -- Predicates ---------------------------------------------------------------
        # Rising edge -> wire as an event edge to the JTC's "Set trajectory"
        # transition in AICA Studio. This component cannot observe whether the
        # resulting motion succeeds; watch the JTC's own predicates for that.
        self.add_predicate("move_requested", False)

        # -- State ----------------------------------------------------------------------
        self._tf_broadcaster = None
        self._trigger_logic = MoveTriggerLogic()

    # -- Validation -------------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        return True

    # -- Lifecycle ----------------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        self._tf_broadcaster = TransformBroadcaster(self)
        return True

    def on_activate_callback(self) -> bool:
        self._move_trigger = False
        self._trigger_logic.reset()
        self.set_predicate("move_requested", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Periodic processing -------------------------------------------------------------

    def on_step_callback(self) -> None:
        # Broadcast every step so the target is always visible/previewable in TF
        # (e.g. RViz) and immediately reflects live parameter edits, independent
        # of whether a move is actually triggered.
        self._broadcast_target_frame()

        # Pulse exactly one step on a move_trigger rising edge, then drop back to
        # False so the predicate has a clean rising edge for the Event wiring.
        self.set_predicate("move_requested", self._trigger_logic.update(self._move_trigger))

    def _broadcast_target_frame(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler_deg(
            self.get_parameter("roll_deg").get_value(),
            self.get_parameter("pitch_deg").get_value(),
            self.get_parameter("yaw_deg").get_value(),
        )
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("reference_frame").get_value()
        t.child_frame_id = self.get_parameter("target_frame_name").get_value()
        t.transform.translation.x = float(self.get_parameter("target_x").get_value())
        t.transform.translation.y = float(self.get_parameter("target_y").get_value())
        t.transform.translation.z = float(self.get_parameter("z_height").get_value())
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)
