"""AICA lifecycle component controlling a Robotiq 2F-140 gripper.

The component drives a Robotiq 2-finger gripper 2F-140 that is connected to the
AICA/Ubuntu host via USB, using the ``pyrobotiqgripper`` library (2.x line —
only depends on pymodbus + pyserial, no numpy, so it installs cleanly into the
ROS image whose system numpy is 1.26.4).

All blocking serial I/O happens on a dedicated worker thread, because

* AICA callbacks must never block, and
* the underlying pymodbus client is *not* thread-safe, so the serial port may
  only ever be touched from a single thread.

The input callbacks therefore only resolve a motion target and hand it to the
worker; the worker owns the gripper connection. Motions are issued non-blocking
(``wait=False``) and a newer target simply overrides the current one.
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional

import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool, Int32

try:  # pragma: no cover - library only present in the built AICA image
    from pyrobotiqgripper import RobotiqGripper
except ImportError:  # allow importing this module (and unit-testing the pure
    RobotiqGripper = None  # logic) without the library/serial stack present


# --- Hardware constants (Robotiq 2F-140, calibrated range used by our setup) ---
GRIPPER_CLOSE_MM = 0.0       # opening width when fully closed
GRIPPER_OPEN_MM = 130.0      # opening width when fully open (calibrated 0..130)

# gOBJ status-register codes (Robotiq specification)
GOBJ_IN_MOTION = 0
GOBJ_OBJECT_WHILE_OPENING = 1
GOBJ_OBJECT_WHILE_CLOSING = 2
GOBJ_AT_POSITION = 3

# Raw 0-255 limits for the percent->raw mapping of speed/force.
RAW_MIN = 0
RAW_MAX = 255


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

        # Connection parameters (read once at configure time).
        self.add_parameter(
            sr.Parameter("port", "/dev/ttyUSB0", sr.ParameterType.STRING),
            "Serieller Port des Greifers (Default '/dev/ttyUSB0'). 'auto' = automatische Erkennung.",
        )
        self.add_parameter(
            sr.Parameter("device_id", 9, sr.ParameterType.INT),
            "Modbus-Slave-ID des Greifers (Robotiq-Standard: 9).",
        )
        self.add_parameter(
            sr.Parameter("activation_timeout", 30.0, sr.ParameterType.DOUBLE),
            "Max. Wartezeit (s) auf Abschluss der Aktivierung beim Start. Der 2F-140 "
            "braucht für seinen Aktivierungshub teils länger als die Library-Default-10 s.",
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
            self.get_logger().error("pyrobotiqgripper is not installed")
            return False
        # Defensive: release any connection left over from a previous configure,
        # so re-configuring never leaves the serial port held open (which would make
        # auto-detect report "no gripper detected on any available ports").
        if self._gripper is not None:
            try:
                self._gripper.disconnect()
            except Exception:
                pass
            self._gripper = None
        port = self.get_parameter("port").get_value()
        device_id = int(self.get_parameter("device_id").get_value())
        activation_timeout = float(self.get_parameter("activation_timeout").get_value())
        try:
            self._gripper = RobotiqGripper(com_port=port, device_id=device_id)
            # The 2F-140 activation stroke can exceed the library's 10 s default
            # timeout (tuned for the smaller 2F-85); allow more time.
            self._gripper.timeOut = activation_timeout
            self._connect_with_retries()
            # Clear any stale/fault state (rACT=0) before re-activating (rACT=1) —
            # the canonical Robotiq sequence; avoids a stuck activation if a previous
            # load attempt left the gripper half-activated.
            self._gripper.reset()
            # activate() and calibrate() each perform a full open/close cycle;
            # the fingers must be free to move during start-up.
            self._gripper.activate()
            self._gripper.calibrate(GRIPPER_CLOSE_MM, GRIPPER_OPEN_MM)
            # Calibration leaves the gripper closed; open it so the physical state
            # matches the logic's initial "open" assumption (gripper_close == False).
            # Otherwise the first "open" command is a no-op and looks unresponsive.
            self._gripper.open(wait=False)
            # Release the port after bring-up. The worker (re)opens it only for the
            # duration of each command, so the port is never held while idle — this
            # keeps reloads/restarts from failing with "Failed to connect" when a
            # previous instance lingers.
            self._gripper.disconnect()
        except Exception as exc:
            self.get_logger().error(f"Failed to set up Robotiq gripper: {exc}")
            # Release the serial port if connect() already succeeded — otherwise the
            # port stays open and the next load reports "no gripper detected".
            if self._gripper is not None:
                try:
                    self._gripper.disconnect()
                except Exception:
                    pass
            self._gripper = None
            return False
        self.set_predicate("is_connected", True)
        return True

    def on_activate_callback(self) -> bool:
        if self._gripper is None:
            self.get_logger().error("Gripper is not configured")
            return False
        # The serial port is opened per command by the worker, not held here.
        self.set_predicate("is_connected", True)
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
        # Stopping the worker also releases the port (the in-flight command's
        # finally-block disconnects). Defensive disconnect in case of an odd state.
        self._join_worker()
        if self._gripper is not None:
            try:
                self._gripper.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"Failed to disconnect gripper on deactivate: {exc}")
        self.set_predicate("is_connected", False)
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

    def _connect_with_retries(self, attempts: int = 12, delay_s: float = 0.5) -> None:
        """Open the serial port, retrying briefly.

        Handles the case where a just-stopped instance is still releasing the port
        (`Failed to connect`). Runs at configure time and inside the worker thread
        (a background thread) — never in the node's cyclic executor, so the brief
        sleeps are fine.
        """
        last_exc: Optional[Exception] = None
        for attempt in range(attempts):
            try:
                self._gripper.connect()
                return
            except Exception as exc:
                last_exc = exc
                if attempt < attempts - 1:
                    time.sleep(delay_s)
        raise last_exc if last_exc is not None else RuntimeError("connect failed")

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

        # Open the port only for this command, then release it in the finally block.
        try:
            self._connect_with_retries()
        except Exception as exc:
            self.get_logger().error(f"Gripper connect failed: {exc}")
            self.set_predicate("is_connected", False)
            return
        self.set_predicate("is_connected", True)
        try:
            # Issue the motion non-blocking; a newer target later simply overrides it.
            if command.kind == "open":
                self._gripper.open(speed=speed, force=force, wait=False)
            elif command.kind == "close":
                self._gripper.close(speed=speed, force=force, wait=False)
            else:  # "move_mm"
                self._gripper.move_mm(command.value_mm, speed=speed, force=force, wait=False)

            # Cooperative poll loop: stay responsive to newer commands. Waiting before
            # the first read also gives the gripper a moment to start moving.
            while not self._stop.is_set():
                self._wake.wait(timeout=self.POLL_PERIOD_S)
                self._wake.clear()
                if self._has_pending():
                    return  # newer command overrides; the outer loop picks it up
                self._gripper.readStatus()
                gobj = self._gripper.status.get("gOBJ", GOBJ_IN_MOTION)
                self.set_predicate(
                    "is_object_grasped",
                    gobj in (GOBJ_OBJECT_WHILE_OPENING, GOBJ_OBJECT_WHILE_CLOSING),
                )
                if gobj != GOBJ_IN_MOTION:
                    return  # reached target or stopped on an object / blockage
        finally:
            # Always release the port so it is free between commands.
            try:
                self._gripper.disconnect()
            except Exception:
                pass
