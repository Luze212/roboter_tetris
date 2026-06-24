"""AICA lifecycle component: endeffector fine-localization via EDGE detection.

Drop-in alternative to :class:`roboter_tetris.robot_cam.RobotCam` for A/B testing
on the robot: identical inputs, output ``[t, x, y, z, orientation]`` and
predicates, but the block footprint is found from image **edges** (Canny) instead
of the green-belt color region. See
:mod:`roboter_tetris.vision.robot_detection_edge`.
"""

import math

import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from .vision.robot_detection import BeltDistanceFilter, near_mask
from .vision.robot_detection_edge import (
    EdgeDetectionParams, color_edge_map, depth_edge_map,
    detect_object_edges, edge_candidate_mask,
)

STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0


class RobotCam2(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()

        # -- Parameters (operator-facing descriptions) ----------------------------
        self.add_parameter(
            sr.Parameter("canny_low", 50, sr.ParameterType.INT),
            "Untere Canny-Schwelle (Hysterese). Schwächere Kanten als dieser Wert "
            "werden verworfen. Zu hoch → Blockkanten fehlen (Umriss bricht auf); "
            "zu niedrig → viele Störkanten. Typisch ~1/3 der oberen Schwelle.")
        self.add_parameter(
            sr.Parameter("canny_high", 150, sr.ParameterType.INT),
            "Obere Canny-Schwelle (Hysterese). Startpunkte starker Kanten. Zu hoch "
            "→ schwache Blockkanten fehlen; zu niedrig → verrauschte Kanten.")
        self.add_parameter(
            sr.Parameter("blur_ksize", 5, sr.ParameterType.INT),
            "Weichzeichnung vor Canny (ungerader Kernel in px) gegen Bildrauschen. "
            "Größer → glattere, aber unschärfere Kanten. 1 = aus.")
        self.add_parameter(
            sr.Parameter("use_depth_edges", True, sr.ParameterType.BOOL),
            "Tiefenstufe (Block↔Band-Rand) als zweite Kantenquelle dazunehmen "
            "(ODER mit den Farb-Kanten). Schließt den Umriss auch bei schwachem "
            "Helligkeitskontrast (z. B. weiß auf grün). Nicht das rohe Tiefenbild, "
            "sondern der Rand der Nah-Region (rauschrobust). Aus = reine Farb-Kanten.")
        self.add_parameter(
            sr.Parameter("min_contour_area", 500.0, sr.ParameterType.DOUBLE),
            "Mindest-Blobfläche in px; kleinere geschlossene Kantenflächen gelten "
            "als Rauschen.")
        self.add_parameter(
            sr.Parameter("morph_kernel_size", 15, sr.ParameterType.INT),
            "Schließt den (oft unterbrochenen) Kantenumriss zu einer Fläche "
            "(Morphologie-Close, elliptischer Kernel in px). Zu klein → Umriss "
            "bleibt offen, keine Fläche; zu groß → benachbarte Objekte verschmelzen.")
        self.add_parameter(
            sr.Parameter("use_depth_gate", True, sr.ParameterType.BOOL),
            "Nah-Gate: behalte nur Kanten-Blobs, die ein Tiefen-Loch / eine "
            "Erhebung über dem Band überlappen. Verwirft Hintergrund-/Bandkanten. "
            "Zum reinen Kanten-Test abschaltbar.")
        self.add_parameter(
            sr.Parameter("depth_search_radius_px", 2, sr.ParameterType.INT),
            "Suchradius (px) um den Band-Messpunkt für einen gültigen Tiefenwert.")
        self.add_parameter(
            sr.Parameter("depth_average_frames", 5, sr.ParameterType.INT),
            "Gleitender Mittelwert der Band-Distanz über die letzten N Frames "
            "(1 = aus). Mittelung gegen den ungenauen Tiefensensor.")
        self.add_parameter(
            sr.Parameter("depth_scale_to_mm", 1.0, sr.ParameterType.DOUBLE),
            "mm pro Tiefen-Rohwert (16UC1-Bild). 1.0 = bereits mm (D400-Serie); für "
            "Kameras mit anderer Tiefen-Einheit entsprechend setzen.")
        self.add_parameter(
            sr.Parameter("debug_enable", False, sr.ParameterType.BOOL),
            "Debug-Bild erzeugen und publizieren (kostet Rechenzeit).")

        # -- Inputs (endeffector RealSense only) ----------------------------------
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._aligned_depth_msg = Image()
        self.add_input("aligned_depth_image", "_aligned_depth_msg", Image)

        # -- Outputs ---------------------------------------------------------------
        # std_msgs signals are plain Python values: list -> Float64MultiArray.
        self._object_position = []
        self.add_output("object_position", "_object_position", Float64MultiArray)
        self._debug_msg = Image()
        self.add_output("debug_image", "_debug_msg", Image)

        # -- Predicates -------------------------------------------------------------
        self.add_predicate("is_object_visible", False)
        self.add_predicate("is_receiving_frames", False)

        # -- State ------------------------------------------------------------------
        self._belt_filter = BeltDistanceFilter()
        self._last_stamp = None
        self._last_frame_walltime = None
        self._last_error_walltime = None
        self._logged_shapes = False

    # -- Validation ----------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name in ("canny_low", "canny_high") and parameter.get_value() < 0:
            self.get_logger().warn(f"{name} must be >= 0")
            return False
        if name == "min_contour_area" and parameter.get_value() <= 0.0:
            self.get_logger().warn("min_contour_area must be positive")
            return False
        return True

    # -- Lifecycle -----------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._belt_filter.reset()
        self._last_stamp = None
        self._last_frame_walltime = None
        self._logged_shapes = False
        self._color_msg = Image()
        self._info_msg = CameraInfo()
        self._aligned_depth_msg = Image()
        self._object_position = []
        self.set_predicate("is_object_visible", False)
        self.set_predicate("is_receiving_frames", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Helpers -------------------------------------------------------------------

    def _params(self) -> EdgeDetectionParams:
        return EdgeDetectionParams(
            canny_low=int(self.get_parameter("canny_low").get_value()),
            canny_high=int(self.get_parameter("canny_high").get_value()),
            blur_ksize=int(self.get_parameter("blur_ksize").get_value()),
            use_depth_edges=bool(self.get_parameter("use_depth_edges").get_value()),
            min_contour_area=self.get_parameter("min_contour_area").get_value(),
            morph_kernel_size=int(self.get_parameter("morph_kernel_size").get_value()),
            use_depth_gate=bool(self.get_parameter("use_depth_gate").get_value()),
            depth_search_radius_px=int(self.get_parameter("depth_search_radius_px").get_value()),
            depth_average_frames=int(self.get_parameter("depth_average_frames").get_value()),
        )

    def _log_error_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if (self._last_error_walltime is None
                or (now - self._last_error_walltime).nanoseconds / 1e9 >= ERROR_LOG_PERIOD_S):
            self.get_logger().error(message)
            self._last_error_walltime = now

    def _clear_outputs(self) -> None:
        self._object_position = []
        self.set_predicate("is_object_visible", False)

    def _handle_stale(self) -> None:
        if self._last_frame_walltime is None:
            return
        if (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9 > STALE_TIMEOUT_S:
            self._clear_outputs()
            self.set_predicate("is_receiving_frames", False)

    @staticmethod
    def _valid_info(info) -> bool:
        return len(info.k) >= 9 and info.k[0] > 0.0 and info.k[4] > 0.0

    # -- Periodic processing -------------------------------------------------------

    def on_step_callback(self):
        # Color is the primary detection source; aligned depth (in the color frame)
        # gates background edges and gives the live belt distance. Both required.
        if self._color_msg.width == 0 or self._aligned_depth_msg.width == 0:
            self._handle_stale()
            return

        if not self._valid_info(self._info_msg):
            self._log_error_throttled(
                "Warte auf gültige color_camera_info (fx=0) — ist die CameraInfo verdrahtet?")
            self._handle_stale()
            return

        stamp = (self._color_msg.header.stamp.sec, self._color_msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            self._handle_stale()
            return
        self._last_stamp = stamp
        self._last_frame_walltime = self.get_clock().now()
        self.set_predicate("is_receiving_frames", True)

        try:
            color_bgr = self._bridge.imgmsg_to_cv2(self._color_msg, desired_encoding="bgr8")
            ch, cw = color_bgr.shape[:2]

            depth_img = self._bridge.imgmsg_to_cv2(self._aligned_depth_msg,
                                                   desired_encoding="passthrough")
            if depth_img.dtype == np.float32:
                depth_mm = depth_img * 1000.0  # 32FC1 already in meters
            else:
                depth_mm = depth_img.astype(np.float32) \
                    * self.get_parameter("depth_scale_to_mm").get_value()  # 16UC1 -> mm

            dh, dw = depth_mm.shape[:2]
            if (dw, dh) != (cw, ch):
                depth_mm = cv2.resize(depth_mm, (cw, ch), interpolation=cv2.INTER_NEAREST)

            iw = int(getattr(self._info_msg, "width", 0)) or cw
            ih = int(getattr(self._info_msg, "height", 0)) or ch
            sx, sy = cw / iw, ch / ih
            fx = self._info_msg.k[0] * sx
            cx = self._info_msg.k[2] * sx
            fy = self._info_msg.k[4] * sy
            cy = self._info_msg.k[5] * sy

            if not self._logged_shapes:
                self.get_logger().info(
                    f"robot_cam_2 (edge): Farbe {cw}x{ch}, aligned Depth {dw}x{dh}, "
                    f"CameraInfo {iw}x{ih}")
                self._logged_shapes = True

            result = detect_object_edges(color_bgr, depth_mm, fx, fy, cx, cy,
                                         self._params(), self._belt_filter)

            if result is None:
                self._clear_outputs()
            else:
                t = stamp[0] + stamp[1] / 1e9
                self._object_position = [t, result.x_mm, result.y_mm,
                                         result.z_band_mm, result.orientation_rad]
                self.set_predicate("is_object_visible", True)

            if self.get_parameter("debug_enable").get_value():
                self._publish_debug(color_bgr, depth_mm, result)

        except Exception as exc:
            self._log_error_throttled(f"robot_cam_2 pipeline error: {exc}")

    def _publish_debug(self, color_bgr, depth_mm, result) -> None:
        # Overlay on the COLOR image so the operator sees the result and *why*:
        #   - blue tint   = depth near-gate region
        #   - cyan        = color Canny edges (raw, before closing)
        #   - orange      = depth-step edges (fused 2nd source, if enabled)
        #   - gray lines  = closed edge blobs that did NOT win
        #   - green       = detected contour, yellow = oriented box,
        #     red         = center + orientation line, blue dot = belt sample point
        params = self._params()
        debug_img = color_bgr.copy()

        # 1. Depth near-gate region.
        if params.use_depth_gate:
            near = near_mask(depth_mm)
            if near.any():
                tint = debug_img.copy()
                tint[near] = (255, 90, 0)
                cv2.addWeighted(tint, 0.25, debug_img, 0.75, 0, debug_img)

        # 2. Color Canny edges (cyan) and the fused depth-step edges (orange), so
        #    the operator sees which source carries the outline.
        debug_img[color_edge_map(color_bgr, params) > 0] = (255, 255, 0)
        if params.use_depth_edges:
            debug_img[depth_edge_map(depth_mm) > 0] = (0, 140, 255)

        # 3. All closed candidate blobs over min area; loser(s) in thin gray.
        candidate = edge_candidate_mask(color_bgr, depth_mm, params)
        cnts, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) >= params.min_contour_area:
                cv2.drawContours(debug_img, [c], 0, (160, 160, 160), 1)

        # 4. The winning detection.
        if result is not None:
            cv2.drawContours(debug_img, [result.contour], 0, (0, 255, 0), 2)
            box = cv2.boxPoints(cv2.minAreaRect(result.contour)).astype(np.int32)
            cv2.polylines(debug_img, [box], True, (0, 255, 255), 2)
            rx, ry = result.ref_px
            cv2.drawMarker(debug_img, (rx, ry), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            ex = int(rx + 45 * math.cos(result.orientation_rad))
            ey = int(ry + 45 * math.sin(result.orientation_rad))
            cv2.line(debug_img, (rx, ry), (ex, ey), (0, 0, 255), 2)
            cv2.circle(debug_img, result.belt_px, 4, (255, 0, 0), -1)
            status = (f"z_band={result.z_band_mm:.0f}mm  x={result.x_mm:.0f}  "
                      f"y={result.y_mm:.0f}  ang={math.degrees(result.orientation_rad):.0f}deg")
        else:
            status = "NO OBJECT (kein Kanten-Blob >= min area / Gate)"

        # 5. Readout + legend on a dark strip.
        h, w = debug_img.shape[:2]
        cv2.rectangle(debug_img, (0, 0), (w, 50), (0, 0, 0), -1)
        cv2.putText(debug_img, status, (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        legend = "cyan=Farbkante gruen=Detektion gelb=Box rot=Mitte/Winkel blau=Bandpunkt grau=verworfen"
        if params.use_depth_edges:
            legend += " orange=Tiefenkante"
        if params.use_depth_gate:
            legend += " blaue-Flaeche=Nah-Gate"
        cv2.putText(debug_img, legend, (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
