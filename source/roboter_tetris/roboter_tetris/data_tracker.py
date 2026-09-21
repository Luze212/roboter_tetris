"""AICA lifecycle component `data_tracker` -- the list of every block, for display.

Thin shell around :class:`roboter_tetris.world_bookkeeping.WorldBook`. A leaf
of the graph: no line leads from here back into control, so it may run at
10 Hz and stay silent when in doubt.
"""

from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray

from .contracts import (
    ContractError, pack_world_state, unpack_not_pickable, unpack_picked_id,
    unpack_tracks,
)
from .world_bookkeeping import WorldBook

WARN_LOG_PERIOD_S = 5.0


class DataTracker(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # -- Parameters (operator-facing descriptions) ----------------------------
        self.add_parameter(
            sr.Parameter("expiry_after_done_s", 10.0, sr.ParameterType.DOUBLE),
            "So lange (s, in Bildzeit) bleibt ein erledigter Eintrag noch sichtbar — "
            "gepickt, hinter der Greifebene oder aus dem Bild verschwunden, gezählt "
            "ab der letzten dieser Änderungen.")

        # -- Inputs / output (std_msgs signals are plain Python values) ----------
        self._tracks_in = []
        self.add_input("tracks", "_tracks_in", Float64MultiArray)
        self._not_pickable_in = []
        self.add_input("not_pickable", "_not_pickable_in", Float64MultiArray)
        self._picked_in = []
        self.add_input("picked_id", "_picked_in", Float64MultiArray)
        # Never empty: before the first frame t = 0 and no entries.
        self._world_state = pack_world_state(0.0, (0.0, 0.0), 0, [])
        self.add_output("world_state", "_world_state", Float64MultiArray)

        self.add_predicate("has_objects", False)

        # -- State ----------------------------------------------------------------
        self._book = WorldBook()
        self._last_t = None
        self._last_warn = {}

    # -- Validation ---------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "expiry_after_done_s" and parameter.get_value() <= 0.0:
            self.get_logger().warn("expiry_after_done_s must be positive")
            return False
        return True

    # -- Lifecycle ----------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._book = WorldBook(self.get_parameter("expiry_after_done_s").get_value())
        self._last_t = None
        self._world_state = pack_world_state(0.0, (0.0, 0.0), 0, [])
        self.set_predicate("has_objects", False)
        return True

    def on_deactivate_callback(self) -> bool:
        self._book.reset()
        self.set_predicate("has_objects", False)
        return True

    # -- Helpers ------------------------------------------------------------------

    def _warn_throttled(self, key: str, message: str) -> None:
        now = self.get_clock().now()
        last = self._last_warn.get(key)
        if last is None or (now - last).nanoseconds / 1e9 >= WARN_LOG_PERIOD_S:
            self.get_logger().warn(message)
            self._last_warn[key] = now

    def _read(self, key: str, unpack, value):
        try:
            return unpack(value)
        except ContractError as exc:
            self._warn_throttled(key, f"data_tracker: {key} verworfen — {exc}")
            return None

    # -- Periodic processing --------------------------------------------------------

    def on_step_callback(self):
        self._book.expiry_after_done_s = self.get_parameter(
            "expiry_after_done_s").get_value()

        # Tracks first: the flags below take their time from the latest frame.
        msg = self._read("tracks", unpack_tracks, self._tracks_in)
        if msg is not None and msg.t != self._last_t:
            self._last_t = msg.t
            self._book.update_tracks(msg)

        not_pickable = self._read("not_pickable", unpack_not_pickable,
                                  self._not_pickable_in)
        if not_pickable is not None:
            self._book.mark_out_of_bounds(not_pickable.ids)

        picked = self._read("picked_id", unpack_picked_id, self._picked_in)
        if picked is not None:
            self._book.on_picked(picked)

        self._world_state = self._book.pack()
        self.set_predicate("has_objects", len(self._book) > 0)
