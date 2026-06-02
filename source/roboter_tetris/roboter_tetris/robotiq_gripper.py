"""AICA lifecycle component controlling a Robotiq 2F-140 gripper.

The component drives a Robotiq 2-finger gripper 2F-140 that is connected to the
AICA/Ubuntu host via USB, using the vendored :mod:`roboter_tetris.robotiq_driver`
(Modbus RTU on top of pymodbus/pyserial).

All blocking serial I/O happens on a dedicated worker thread, because

* AICA callbacks must never block, and
* the underlying pymodbus client is *not* thread-safe, so the serial port may
  only ever be touched from a single thread.

The input callbacks therefore only resolve a motion target and hand it to the
worker; the worker owns the gripper connection and reacts to newer targets
cooperatively (preemption by overwriting the ``move`` target).
"""

import threading
from dataclasses import dataclass
from typing import Optional

import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool, Int32

try:  # pragma: no cover - needs pymodbus/pyserial, only present in built image
    from .robotiq_driver import RobotiqGripper
except ImportError:  # allow importing this module (and unit-testing the pure
    RobotiqGripper = None  # logic) without the serial/Modbus stack present


# --- Hardware constants (Robotiq 2F-140, calibrated range used by our setup) ---
GRIPPER_CLOSE_MM = 0.0       # opening width when fully closed
GRIPPER_OPEN_MM = 130.0      # opening width when fully open (calibrated 0..130)
RAW_MIN = 0                  # raw bit value: fully open
RAW_MAX = 255                # raw bit value: fully closed

# gOBJ status-register codes (Robotiq specification)
GOBJ_IN_MOTION = 0
GOBJ_OBJECT_WHILE_OPENING = 1
GOBJ_OBJECT_WHILE_CLOSING = 2
GOBJ_AT_POSITION = 3


def percent_to_raw(percent: float) -> int:
    """Map a 0-100 % value to the gripper's raw 0-255 range (clamped)."""
    raw = round(float(percent) / 100.0 * RAW_MAX)
    return max(RAW_MIN, min(RAW_MAX, raw))


def clamp_mm(value: float) -> float:
    """Clamp an opening width to the calibrated [0, 130] mm range."""
    return max(GRIPPER_CLOSE_MM, min(GRIPPER_OPEN_MM, float(value)))


@dataclass(frozen=True)
class GripperCommand:
    """A resolved motion target for the worker thread."""

    kind: str               # "open" | "close" | "move_mm"
    value_mm: float = 0.0    # target opening width, only used for "move_mm"


class GripperTargetLogic:
    """Hardware-free resolution of the two inputs into a motion target.

    Encapsulates the precedence rule ("gripper_close" wins) and edge detection,
    so repeated identical inputs do not re-trigger motion. No I/O — fully
    unit-testable without the modulo runtime or real hardware.
    """

    def __init__(self) -> None:
        self._close = False
        self._last_change: Optional[int] = None

    def update_close(self, close: bool) -> Optional[GripperCommand]:
        """Handle a new ``gripper_close`` value; return a target if it changed."""
        if close == self._close:
            return None
        self._close = close
        # Any close transition resets change tracking, so a later (even identical)
        # change value re-applies once the gripper has reopened.
        self._last_change = None
        return GripperCommand("close") if close else GripperCommand("open")

    def update_change(self, value: int) -> Optional[GripperCommand]:
        """Handle a new ``gripper_change`` value; return a target if applicable.

        Ignored entirely while ``gripper_close`` is active (Close has priority).
        """
        if self._close:
            return None
        if value == self._last_change:
            return None
        self._last_change = value
        return GripperCommand("move_mm", clamp_mm(value))


class RobotiqGripperComponent(LifecycleComponent):
    """Lifecycle component that controls a Robotiq 2F-140 gripper over USB."""

    #: How often the worker polls the gripper status while a motion is running.
    POLL_PERIOD_S = 0.05
    #: How long to wait for the worker thread to join on teardown.
    JOIN_TIMEOUT_S = 5.0

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        # Inputs (event-driven; callbacks must not block and must not do I/O).
        # modulo exposes std_msgs signals as plain Python values, not messages.
        self._gripper_close = False
        self._gripper_change = 0
        self.add_input(
            "gripper_close", "_gripper_close", Bool,
            user_callback=self._on_gripper_close,
        )
        self.add_input(
            "gripper_change", "_gripper_change", Int32,
            user_callback=self._on_gripper_change,
        )

        # Parameters (dynamic, expressed in percent for the operator).
        self.add_parameter(
            sr.Parameter("force", 50.0, sr.ParameterType.DOUBLE),
            "Greifkraft in Prozent (0-100 %). Intern auf 0-255 gemappt.",
        )
        self.add_parameter(
            sr.Parameter("grasping_speed", 100.0, sr.ParameterType.DOUBLE),
            "Schließgeschwindigkeit in Prozent (0-100 %). Intern auf 0-255 gemappt.",
        )

        # Predicates (boolean status surfaced to the AICA UI / event system).
        self.add_predicate("is_connected", False)
        self.add_predicate("is_object_grasped", False)

        self._logic = GripperTargetLogic()
        self._gripper = None

        # Worker-thread coordination.
        self._lock = threading.Lock()
        self._pending: Optional[GripperCommand] = None
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None

    # -- Parameter validation -------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() in ("force", "grasping_speed"):
            if parameter.is_empty():
                self.get_logger().warn(f"{parameter.get_name()} must not be empty")
                return False
            value = parameter.get_value()
            if value < 0.0 or value > 100.0:
                self.get_logger().warn(
                    f"{parameter.get_name()} must be within 0-100 %, got {value}"
                )
                return False
        return True

    # -- Input callbacks (executor thread: no blocking, no serial I/O) --------

    def _on_gripper_close(self) -> None:
        command = self._logic.update_close(bool(self._gripper_close))
        if command is not None:
            self._enqueue(command)

    def _on_gripper_change(self) -> None:
        command = self._logic.update_change(int(self._gripper_change))
        if command is not None:
            self._enqueue(command)

    def _enqueue(self, command: GripperCommand) -> None:
        with self._lock:
            self._pending = command
        self._wake.set()

    # -- Lifecycle ------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        if RobotiqGripper is None:
            self.get_logger().error(
                "robotiq_driver unavailable (pymodbus/pyserial not installed)"
            )
            return False
        try:
            self._gripper = RobotiqGripper(com_port="auto")
            self._gripper.connect()
            self._gripper.activate()
            # Auto bit-calibration performs a full open/close cycle; fingers must
            # be free to move during start-up.
            self._gripper.calibrate_bit()
            self._gripper.calibrate_mm(GRIPPER_CLOSE_MM, GRIPPER_OPEN_MM)
        except Exception as exc:
            self.get_logger().error(f"Failed to set up Robotiq gripper: {exc}")
            self._gripper = None
            return False
        self.set_predicate("is_connected", True)
        return True

    def on_activate_callback(self) -> bool:
        # Drop any command that arrived while inactive.
        with self._lock:
            self._pending = None
        self._wake.clear()
        self._stop.clear()
        self._worker = threading.Thread(
            target=self._worker_loop, name="robotiq_gripper_worker", daemon=True,
        )
        self._worker.start()
        return True

    def on_deactivate_callback(self) -> bool:
        self._join_worker()
        try:
            if self._gripper is not None:
                self._gripper.stop()
        except Exception as exc:
            self.get_logger().warn(f"Failed to stop gripper on deactivate: {exc}")
        return True

    def on_cleanup_callback(self) -> bool:
        self._teardown()
        return True

    def on_shutdown_callback(self) -> bool:
        self._teardown()
        return True

    def on_error_callback(self) -> bool:
        self._teardown()
        return True

    def _join_worker(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._worker is not None:
            self._worker.join(timeout=self.JOIN_TIMEOUT_S)
            self._worker = None

    def _teardown(self) -> None:
        self._join_worker()
        if self._gripper is not None:
            try:
                self._gripper.stop()
                self._gripper.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"Failed to disconnect gripper: {exc}")
            self._gripper = None
        self.set_predicate("is_connected", False)
        self.set_predicate("is_object_grasped", False)

    # -- Worker thread (sole owner of the serial connection) ------------------

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            command = self._take_pending()
            if command is None:
                self._wake.wait(timeout=self.POLL_PERIOD_S)
                self._wake.clear()
                continue
            try:
                self._execute(command)
            except Exception as exc:
                self.get_logger().error(f"Gripper command failed: {exc}")

    def _take_pending(self) -> Optional[GripperCommand]:
        with self._lock:
            command = self._pending
            self._pending = None
        return command

    def _has_pending(self) -> bool:
        with self._lock:
            return self._pending is not None

    def _execute(self, command: GripperCommand) -> None:
        speed = percent_to_raw(self.get_parameter("grasping_speed").get_value())
        force = percent_to_raw(self.get_parameter("force").get_value())

        # Issue the motion non-blocking; a newer target later simply overrides it.
        if command.kind == "open":
            self._gripper.move(RAW_MIN, speed, force, wait=False)
        elif command.kind == "close":
            self._gripper.move(RAW_MAX, speed, force, wait=False)
        else:  # "move_mm"
            self._gripper.move_mm(command.value_mm, speed, force, wait=False)

        # Cooperative poll loop: we own the serial port here and stay responsive
        # to newer commands. Waiting before the first read also gives the gripper
        # a moment to start moving before we sample gOBJ.
        while not self._stop.is_set():
            self._wake.wait(timeout=self.POLL_PERIOD_S)
            self._wake.clear()
            if self._has_pending():
                return  # newer command overrides; the outer loop picks it up
            status = self._gripper.status()
            gobj = status.get("gOBJ", GOBJ_IN_MOTION)
            self.set_predicate(
                "is_object_grasped",
                gobj in (GOBJ_OBJECT_WHILE_OPENING, GOBJ_OBJECT_WHILE_CLOSING),
            )
            if gobj != GOBJ_IN_MOTION:
                return  # reached target or stopped on an object / blockage
