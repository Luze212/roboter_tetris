"""AICA test component that continuously outputs a boolean ``True`` signal.

Minimal helper to verify signal wiring/flow in AICA: it exposes a single bool
output that is always ``True``. Wire it into any bool input to feed a constant
True. The engine publishes the output every step while the component is ACTIVE.
"""

from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool


class TrueSignal(LifecycleComponent):
    """Outputs a constant boolean ``True`` on the ``value`` signal."""

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)
        # std_msgs bool output is a plain Python bool; modulo publishes it
        # periodically (publish_on_step defaults to True) while ACTIVE.
        self._value = True
        self.add_output("value", "_value", Bool)

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        return True
