"""AICA lifecycle component `interface_streamer` -- the overview image (Phase 5).

Thin shell around :mod:`roboter_tetris.interface_layout`. The only component
the system works without: it may run slowly (5 Hz), and whatever fails here is
logged and skipped -- it must never disturb anything else.
"""

from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from .contracts import ContractError, unpack_follower_status, unpack_world_state
from .interface_layout import compose, object_rows, status_lines

WARN_LOG_PERIOD_S = 5.0
#: No new debug image for this long -> its area turns grey.
IMAGE_TIMEOUT_S = 2.0
#: follower_status / world_state older than this -> marked as stale.
DATA_STALE_S = 1.0


class InterfaceStreamer(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self.add_parameter(sr.Parameter("panel_width", 1280, sr.ParameterType.INT),
                           "Breite des Gesamtbildes (px). Die Höhe folgt aus "
                           "image_height und show_object_list.")
        self.add_parameter(sr.Parameter("image_height", 360, sr.ParameterType.INT),
                           "Höhe des Kamerabildbereichs (px).")
        self.add_parameter(sr.Parameter("show_object_list", True, sr.ParameterType.BOOL),
                           "Objektliste unter dem Status einblenden.")

        self._base_msg = Image()
        self.add_input("base_debug_image", "_base_msg", Image,
                       user_callback=self._on_base_image)
        self._world_in = []
        self.add_input("world_state", "_world_in", Float64MultiArray)
        self._status_in = []
        self.add_input("follower_status", "_status_in", Float64MultiArray)

        self._interface_msg = Image()
        self.add_output("interface_image", "_interface_msg", Image,
                        publish_on_step=False)

        self._bridge = None
        self._received = {}               # source -> clock time of the last image
        self._last_warn = {}

    # -- Validation -----------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        value = parameter.get_value()
        if name == "panel_width" and value < 320:
            self.get_logger().warn("panel_width must be at least 320")
            return False
        if name == "image_height" and value < 90:
            self.get_logger().warn("image_height must be at least 90")
            return False
        return True

    # -- Lifecycle ------------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        self._bridge = CvBridge()
        return True

    def on_activate_callback(self) -> bool:
        self._received = {}
        return True

    def on_deactivate_callback(self) -> bool:
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

    def _on_base_image(self) -> None:
        self._received["base"] = self._now_s()

    def _image(self, source: str, msg: Image):
        """The latest image of a source, or None if silent or unreadable."""
        received = self._received.get(source)
        if received is None or self._now_s() - received > IMAGE_TIMEOUT_S:
            return None
        try:
            return self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:          # any conversion problem: skip this source
            self._warn_throttled(source, f"interface_streamer: {source}-Bild - {exc}")
            return None

    def _read(self, key, unpack, value):
        try:
            return unpack(value)
        except ContractError as exc:
            self._warn_throttled(key, f"interface_streamer: {key} verworfen - {exc}")
            return None

    # -- Periodic processing ----------------------------------------------------------

    def on_step_callback(self):
        if self._bridge is None:
            return
        try:
            now = self._now_s()
            status = self._read("follower_status", unpack_follower_status, self._status_in)
            world = self._read("world_state", unpack_world_state, self._world_in)
            lines = status_lines(
                status, world,
                follower_stale=status is not None and now - status.t > DATA_STALE_S,
                world_stale=world is not None and now - world.t > DATA_STALE_S)
            panel = compose(
                self._image("base", self._base_msg), lines, object_rows(world),
                int(self.get_parameter("panel_width").get_value()),
                int(self.get_parameter("image_height").get_value()),
                bool(self.get_parameter("show_object_list").get_value()))
            self._interface_msg = self._bridge.cv2_to_imgmsg(panel, "bgr8")
            self.publish_output("interface_image")
        except Exception as exc:          # the display must never take anything down
            self._warn_throttled("step", f"interface_streamer: Bild übersprungen - {exc}")
