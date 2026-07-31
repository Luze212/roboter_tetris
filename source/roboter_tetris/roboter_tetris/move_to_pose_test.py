"""AICA test component: moves the robot arm to a manually configured (x, y) coordinate.

Standalone smoke test for the "move to a given location" building block of the
future perception -> target_selection -> pick_sequencer pipeline. Deliberately
minimal for now:

* ``z_height`` and the target orientation are fixed defaults (parameters), not
  computed from a detected object.
* ``target_x``/``target_y`` are live, ``dynamic`` AICA UI parameters rather than
  a signal input. The later production component (``target_selection``) will
  replace this parameter read with a ``CartesianPose`` signal input; the
  motion-triggering logic below (broadcast a target TF frame, then call
  ``set_trajectory``) is written so that swap only touches *where the target
  pose comes from*, not how it is used.

Motion path ("Weg 2" from ARCHITECTURE.md): broadcast a TF frame named
``target_frame_name`` at the configured pose via ``tf2_ros``, then trigger the
JTC's ``set_trajectory`` service (``modulo_interfaces.srv.StringTrigger``)
referencing that frame name. The service call follows the non-blocking
call_async + add_done_callback rule for service calls in ``on_step_callback``.
"""

import json
import math

import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from modulo_interfaces.srv import StringTrigger


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
    ``GripperTargetLogic`` in ``robotiq_gripper.py``.
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
    """Broadcasts a configurable target TF frame and triggers ``set_trajectory`` to it.

    Wire a bool into ``move_trigger`` (e.g. the existing ``true_signal``/
    ``toggle_signal`` components) to fire the move on its rising edge.
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
            "Name des TF-Frames, der für die Zielpose gebroadcastet wird.",
        )
        self.add_parameter(
            sr.Parameter("move_duration_s", 3.0, sr.ParameterType.DOUBLE),
            "Trajektoriendauer (s), die dem 'set_trajectory'-Service übergeben wird.",
        )
        self.add_parameter(
            sr.Parameter("set_trajectory_service", "/set_trajectory", sr.ParameterType.STRING),
            "Service-Name des JTC 'set_trajectory' (StringTrigger).",
        )

        # -- Inputs -----------------------------------------------------------------
        self._move_trigger = False
        self.add_input("move_trigger", "_move_trigger", Bool)

        # -- Predicates ---------------------------------------------------------------
        self.add_predicate("is_moving", False)
        self.add_predicate("has_move_succeeded", False)
        self.add_predicate("has_move_failed", False)

        # -- State ----------------------------------------------------------------------
        self._tf_broadcaster = None
        self._trajectory_client = None
        self._trigger_logic = MoveTriggerLogic()

    # -- Validation -------------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "move_duration_s" and parameter.get_value() <= 0.0:
            self.get_logger().warn("move_duration_s must be positive")
            return False
        return True

    # -- Lifecycle ----------------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        self._tf_broadcaster = TransformBroadcaster(self)
        self._trajectory_client = self.create_client(
            StringTrigger, self.get_parameter("set_trajectory_service").get_value())
        return True

    def on_activate_callback(self) -> bool:
        self._move_trigger = False
        self._trigger_logic.reset()
        self.set_predicate("is_moving", False)
        self.set_predicate("has_move_succeeded", False)
        self.set_predicate("has_move_failed", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Periodic processing -------------------------------------------------------------

    def on_step_callback(self) -> None:
        # Broadcast every step so the target is always visible/previewable in TF
        # (e.g. RViz) and immediately reflects live parameter edits, independent
        # of whether a move is actually triggered.
        self._broadcast_target_frame()

        if self._trigger_logic.update(self._move_trigger):
            self._send_move_request()

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

    def _send_move_request(self) -> None:
        if not self._trajectory_client.service_is_ready():
            self.get_logger().warn("set_trajectory Service nicht erreichbar.")
            self.set_predicate("has_move_failed", True)
            return

        # JSON is a valid subset of YAML, so this satisfies either a strict JSON
        # or a YAML-flavoured payload parser on the receiving end.
        payload = json.dumps({
            "frames": [self.get_parameter("target_frame_name").get_value()],
            "durations": [self.get_parameter("move_duration_s").get_value()],
        })
        request = StringTrigger.Request()
        request.payload = payload

        self.set_predicate("is_moving", True)
        self.set_predicate("has_move_succeeded", False)
        self.set_predicate("has_move_failed", False)

        future = self._trajectory_client.call_async(request)
        future.add_done_callback(self._on_move_response)

    def _on_move_response(self, future) -> None:
        self.set_predicate("is_moving", False)
        try:
            response = future.result()
            if response.success:
                self.set_predicate("has_move_succeeded", True)
            else:
                self.get_logger().warn(f"set_trajectory abgelehnt: {response.message}")
                self.set_predicate("has_move_failed", True)
        except Exception as exc:
            self.get_logger().error(f"set_trajectory Service-Call fehlgeschlagen: {exc}")
            self.set_predicate("has_move_failed", True)
