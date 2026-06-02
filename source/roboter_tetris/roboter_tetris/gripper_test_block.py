"""AICA test component to manually toggle the Robotiq gripper open/close.

A minimal helper to verify that gripper control through AICA works end to end.
It exposes a single bool output ``gripper_close`` (wire it to the Robotiq Gripper
component's ``gripper_close`` input) and a trigger service ``toggle_gripper``.

Each call of the service — the "Trigger" button in AICA Studio — flips the output
between close (``True``) and open (``False``). Press once to close, again to open.
"""

from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool


class GripperTestBlock(LifecycleComponent):
    """Toggles a bool output to drive the gripper open/close for testing."""

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        # Output published on demand (only when the service is triggered), not
        # every step. modulo exposes a std_msgs bool output as a plain Python bool.
        self._gripper_close = False
        self.add_output(
            "gripper_close", "_gripper_close", Bool, publish_on_step=False,
        )

        # Predicate so the last commanded direction is visible in the UI.
        self.add_predicate("is_close_commanded", False)

        # Trigger service: appears as a "Trigger" button in AICA Studio.
        self.add_service("toggle_gripper", self._toggle_gripper)

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        return True

    def _toggle_gripper(self) -> dict:
        """Flip the commanded state and publish it once."""
        self._gripper_close = not self._gripper_close
        self.set_predicate("is_close_commanded", self._gripper_close)
        self.publish_output("gripper_close")
        action = "schließen" if self._gripper_close else "öffnen"
        return {
            "success": True,
            "message": f"Greifer {action} (gripper_close={self._gripper_close}).",
        }
