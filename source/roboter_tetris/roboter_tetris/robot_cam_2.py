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
from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from .contracts import pack_object_position
from .vision.robot_detection import BeltDistanceFilter, near_mask
from .vision.robot_detection_edge import (
    EdgeDetectionParams, color_edge_map, depth_edge_map,
    detect_object_edges, edge_candidate_mask,
)

STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0

# The D435i driver publishes both infrared images and aligns the depth to them
# as well, although nothing here uses them. Switching them off at the setup took
# the event_engine (camera drivers and the 500 Hz control loop) from 123 to
# 106 % CPU and let the aligned depth reach the full 15/s (24.09.2026, Nachtrag 13
# / L16). The AICA RealSense block does not expose them (its enable_infra is the
# L515 stream), so the component switches them off on the camera node itself --
# the same way base_cam enforces global_time_enabled. Off by default: set by
# the component right after start-up, it hung the camera driver and with it the
# event_engine; set by hand while running it worked (L16).
INFRA_PARAMETERS = ("enable_infra1", "enable_infra2")
INFRA_RETRY_PERIOD_S = 2.0
INFRA_MAX_ATTEMPTS = 10


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
            "Ohne Wirkung seit 23.09.2026: Die Band-Distanz ist jetzt der Median der "
            "gültigen Tiefe über das Bild, nicht mehr ein Messpunkt unter dem Blob.")
        self.add_parameter(
            sr.Parameter("depth_average_frames", 5, sr.ParameterType.INT),
            "Gleitender Mittelwert der Band-Distanz über die letzten N Frames "
            "(1 = aus). Mittelung gegen den ungenauen Tiefensensor.")
        self.add_parameter(
            sr.Parameter("depth_scale_to_mm", 1.0, sr.ParameterType.DOUBLE),
            "mm pro Tiefen-Rohwert (16UC1-Bild). 1.0 = bereits mm (D400-Serie); für "
            "Kameras mit anderer Tiefen-Einheit entsprechend setzen.")
        self.add_parameter(
            sr.Parameter("max_contour_area", 50000.0, sr.ParameterType.DOUBLE),
            "Obergrenze der Blobflaeche in Pixeln (0 = aus). Wird sie ueberschritten, "
            "meldet die Komponente valid=0 fuer das ganze Bild: Eine bildfuellende "
            "Kontur heisst, dass die Maske versagt hat, und dann ist keiner Kontur zu "
            "trauen. Anhaltspunkt vom 15.09.: ein Klotz belegt rund 4800 px, der "
            "beobachtete Fehlalarm 325000 px.")
        self.add_parameter(
            sr.Parameter("roi_radius_px", 0.0, sr.ParameterType.DOUBLE),
            "Suchradius (px) um den Erwartungspunkt; Kandidaten weiter draussen werden "
            "verworfen (0 = aus, ganzes Bild). Schliesst Maschinenstruktur am Bildrand "
            "konstruktiv aus. Wert am Debug-Bild ablesen, sobald der Klotz sicher "
            "erkannt wird.")
        self.add_parameter(
            sr.Parameter("expect_offset_x_mm", 0.0, sr.ParameterType.DOUBLE),
            "Erwarteter seitlicher Versatz des Klotzes zur optischen Achse, in mm "
            "(Bild-x). 0 = Bildmitte. Bewusst in mm statt in Pixeln: So bleibt der "
            "Wert gueltig, wenn sich die Beobachtungshoehe aendert.")
        self.add_parameter(
            sr.Parameter("expect_offset_y_mm", 0.0, sr.ParameterType.DOUBLE),
            "Wie expect_offset_x_mm, in Bild-y-Richtung.")
        self.add_parameter(
            sr.Parameter("select_nearest_to_expect", False, sr.ParameterType.BOOL),
            "Aus mehreren Kandidaten den zum Erwartungspunkt naechsten waehlen statt "
            "den flaechengroessten. Erst sinnvoll, wenn der Erwartungspunkt gesetzt ist.")
        self.add_parameter(
            sr.Parameter("min_belt_distance_m", 0.0, sr.ParameterType.DOUBLE),
            "Untergrenze der gemessenen Kamera-Band-Distanz in m (0 = aus). Darunter "
            "valid=0 - zu nah, der Seitenflaechen-Verzug setzt ein.")
        self.add_parameter(
            sr.Parameter("max_belt_distance_m", 0.0, sr.ParameterType.DOUBLE),
            "Obergrenze der gemessenen Kamera-Band-Distanz in m (0 = aus). Darueber "
            "valid=0 - zu wenig Bildaufloesung auf dem Klotz.")
        self.add_parameter(
            sr.Parameter("debug_enable", False, sr.ParameterType.BOOL),
            "Debug-Bild erzeugen und publizieren (kostet Rechenzeit).")
        self.add_parameter(
            sr.Parameter("camera_node", "", sr.ParameterType.STRING),
            "Node-Name des RealSense-Blocks der Roboterkamera (über serial_no "
            "241122074842 prüfen). Ist er gesetzt, schaltet die Komponente dort "
            "enable_infra1/enable_infra2 ab - ungenutzte Infrarotbilder kosten "
            "Rechenzeit im event_engine. Leer = aus (Standard): nach dem Start "
            "gesetzt, hängte das AICA auf (Nachtrag 13 / L16).")

        # -- Inputs (endeffector RealSense only) ----------------------------------
        # Queue depth 1, as in base_cam: with modulo's default of 10 the step
        # worked on piled-up images ~0.4 s old (Nachtrag 13). The default is
        # restored for the outputs declared later.
        default_qos = self.get_qos()
        self.set_qos(QoSProfile(depth=1))
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._aligned_depth_msg = Image()
        self.add_input("aligned_depth_image", "_aligned_depth_msg", Image)
        self.set_qos(default_qos)

        # -- Outputs ---------------------------------------------------------------
        # std_msgs signals are plain Python values: list -> Float64MultiArray.
        # S2: fixed length 6. Never empty -- before the first frame t = 0, which
        # fails any age check the receiver applies.
        self._object_position = pack_object_position(0.0, False)
        self.add_output("object_position", "_object_position", Float64MultiArray)
        self._debug_msg = Image()
        # Not on every step: at 100 Hz that would treble the image traffic over
        # the camera rate and make dropped frames look like detection failures
        # during commissioning (N5).
        self.add_output("debug_image", "_debug_msg", Image, publish_on_step=False)

        # -- Predicates -------------------------------------------------------------
        self.add_predicate("is_object_visible", False)
        self.add_predicate("is_receiving_frames", False)

        # -- State ------------------------------------------------------------------
        self._belt_filter = BeltDistanceFilter()
        self._last_stamp = None
        self._last_t = 0.0
        self._last_frame_walltime = None
        self._last_error_walltime = None
        self._logged_shapes = False
        self._set_param_client = None     # created in on_configure (ROS 2 service client)
        self._infra_done = False
        self._infra_attempts = 0
        self._infra_last_try = None

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
        # Service clients are not abstracted by AICA; created here (not in
        # __init__) so the UI parameters are already applied (ARCHITECTURE.md §3).
        node_name = self.get_parameter("camera_node").get_value().strip()
        if node_name:
            if not node_name.startswith("/"):
                node_name = "/" + node_name
            self._set_param_client = self.create_client(
                SetParameters, f"{node_name}/set_parameters")
        return True

    def on_activate_callback(self) -> bool:
        self._belt_filter.reset()
        self._last_stamp = None
        self._last_t = 0.0
        self._last_frame_walltime = None
        self._logged_shapes = False
        self._color_msg = Image()
        self._info_msg = CameraInfo()
        self._aligned_depth_msg = Image()
        self._object_position = []
        self._infra_done = False
        self._infra_attempts = 0
        self._infra_last_try = None
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
            max_contour_area=self.get_parameter("max_contour_area").get_value(),
            roi_radius_px=self.get_parameter("roi_radius_px").get_value(),
            expect_offset_x_mm=self.get_parameter("expect_offset_x_mm").get_value(),
            expect_offset_y_mm=self.get_parameter("expect_offset_y_mm").get_value(),
            select_nearest_to_expect=bool(
                self.get_parameter("select_nearest_to_expect").get_value()),
        )

    def _log_error_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if (self._last_error_walltime is None
                or (now - self._last_error_walltime).nanoseconds / 1e9 >= ERROR_LOG_PERIOD_S):
            self.get_logger().error(message)
            self._last_error_walltime = now

    def _publish_invalid(self) -> None:
        """S2 with ``valid = 0``, keeping the last known ``t``.

        While frames keep arriving ``t`` advances, so the receiver can tell
        "camera works, sees nothing" (fall back to w = 0) from "camera no longer
        delivers" (abort) -- which is the whole point of the flag.
        """
        self._object_position = pack_object_position(self._last_t, False)
        self.set_predicate("is_object_visible", False)

    def _handle_stale(self) -> None:
        if self._last_frame_walltime is None:
            return
        if (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9 > STALE_TIMEOUT_S:
            self._publish_invalid()
            self.set_predicate("is_receiving_frames", False)

    @staticmethod
    def _valid_info(info) -> bool:
        return len(info.k) >= 9 and info.k[0] > 0.0 and info.k[4] > 0.0

    # -- Periodic processing -------------------------------------------------------

    # -- Camera streams ---------------------------------------------------------------

    def _ensure_infra_off(self) -> None:
        """Switch the unused infrared streams off; retried until it sticks.

        Non-blocking as required in a step callback: readiness is polled via
        ``service_is_ready()`` and the call goes out via ``call_async`` with a
        done callback (ARCHITECTURE.md §3). The camera node may come up after
        this component, hence the retries.
        """
        if self._infra_done or self._set_param_client is None:
            return
        if self._infra_attempts >= INFRA_MAX_ATTEMPTS:
            return
        now = self.get_clock().now()
        if (self._infra_last_try is not None
                and (now - self._infra_last_try).nanoseconds / 1e9 < INFRA_RETRY_PERIOD_S):
            return
        self._infra_last_try = now
        if not self._set_param_client.service_is_ready():
            return  # camera node not up yet; try again after the retry period
        self._infra_attempts += 1

        request = SetParameters.Request()
        for name in INFRA_PARAMETERS:
            parameter = RosParameter()
            parameter.name = name
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                             bool_value=False)
            request.parameters.append(parameter)
        future = self._set_param_client.call_async(request)
        future.add_done_callback(self._on_infra_response)

    def _on_infra_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Infrarotbilder konnten nicht abgeschaltet werden: {exc}")
            return
        rejected = [r.reason for r in response.results if not r.successful]
        if rejected:
            # Not retried: a rejection means the wrong node (an L515 has no
            # infra1/infra2) or a driver that refuses -- retrying will not help.
            self._infra_done = True
            self.get_logger().warn(
                "Kamera hat das Abschalten der Infrarotbilder abgelehnt: "
                + "; ".join(rejected) + " — ist camera_node die Roboterkamera?")
            return
        self._infra_done = True
        self.get_logger().info("Infrarotbilder der Roboterkamera abgeschaltet.")

    def on_step_callback(self):
        self._ensure_infra_off()
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

            # S2 in SI units. t is set first and kept even when nothing is
            # detected, so a running t means "camera alive".
            self._last_t = stamp[0] + stamp[1] / 1e9
            if result is None:
                self._publish_invalid()
            else:
                z_band_m = result.z_band_mm / 1000.0
                lo = self.get_parameter("min_belt_distance_m").get_value()
                hi = self.get_parameter("max_belt_distance_m").get_value()
                if (lo > 0.0 and z_band_m < lo) or (hi > 0.0 and z_band_m > hi):
                    # Outside the usable distance window (R3/D18): too close and
                    # the side-face distortion sets in, too far and the block has
                    # too few pixels. Either way the measurement is not usable.
                    self._publish_invalid()
                else:
                    self._object_position = pack_object_position(
                        self._last_t, True,
                        result.x_mm / 1000.0, result.y_mm / 1000.0,
                        z_band_m, result.orientation_rad)
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
        #     red         = center + orientation line, blue dot = principal point (belt = frame median)
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
        self.publish_output("debug_image")
