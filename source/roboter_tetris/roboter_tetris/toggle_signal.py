"""AICA test component: a bool output that toggles True/False on a fixed period.

For exercising downstream components (e.g. the gripper) hands-free: wire ``value``
into a bool input and it alternates every ``period`` seconds. Useful to watch the
gripper open/close repeatedly without pressing anything.
"""

import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool


class ToggleSignal(LifecycleComponent):
    """Outputs a bool on ``value`` that flips every ``period`` seconds."""

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        # std_msgs bool output, published every step while ACTIVE.
        self._value = False
        self.add_output("value", "_value", Bool)

        self.add_parameter(
            sr.Parameter("period", 5.0, sr.ParameterType.DOUBLE),
            "Zeit (s) zwischen den Wechseln des Signals zwischen True und False.",
        )

        self.add_predicate("is_true", False)

        self._last_toggle = None  # set on activate; uses the ROS clock, no sleep

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._value = False
        self.set_predicate("is_true", self._value)
        self._last_toggle = self.get_clock().now()
        return True

    def on_step_callback(self) -> None:
        if self._last_toggle is None:
            self._last_toggle = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self._last_toggle).nanoseconds / 1e9
        if elapsed >= self.get_parameter("period").get_value():
            self._value = not self._value
            self.set_predicate("is_true", self._value)
            self._last_toggle = self.get_clock().now()
