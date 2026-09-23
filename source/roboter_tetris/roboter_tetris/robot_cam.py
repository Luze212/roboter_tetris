"""AICA lifecycle component: fine-localization from the endeffector RealSense.

Thin shell around :mod:`roboter_tetris.vision.robot_detection`. It produces a
time-stamped stream of the grasp object's position and orientation under the
gripper and nothing else — intentionally decoupled from the base camera and from
any pick/motion logic (those belong to a separate processing component).

Detection is **color-based**: the green belt is masked out in the color image and
the remaining matte block top face is localized; the aligned depth is used only
as a near-gate (rejecting belt reflections) and to measure the live belt distance
``z`` (the arm camera changes height while it tracks the object).

Output ``object_position`` is a flat double array ``[t, x, y, z, orientation]``:
``t`` = frame stamp in seconds, ``x``/``y`` = block center in mm in the **camera
frame**, ``z`` = measured **belt distance** in mm (camera -> conveyor surface, NOT
the grasp height — correct downstream with the object height from the base
camera), ``orientation`` = top-face angle in rad [0, pi). Empty when nothing is
visible.
"""

import math

import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from .contracts import pack_object_position
from .vision.robot_detection import (
    BeltDistanceFilter, RobotDetectionParams, detect_object,
    belt_candidate_mask, near_mask,
)

STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0


class RobotCam(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()

        # -- Parameters (operator-facing descriptions) ----------------------------
        self.add_parameter(
            sr.Parameter("belt_h_min", 35, sr.ParameterType.INT),
            "Grünes Band im HSV-Farbton (OpenCV-H 0-179, Grün liegt um 60), untere "
            "Grenze. Alles außerhalb dieses Bereichs gilt als Block-Kandidat. Zu eng "
            "→ Bandreste werden als Block erkannt (weiten); zu weit → grünstichige "
            "Blöcke verschwinden (einengen). Im Debug-Bild prüfen.")
        self.add_parameter(
            sr.Parameter("belt_h_max", 85, sr.ParameterType.INT),
            "Grünes Band im HSV-Farbton (OpenCV-H 0-179, Grün liegt um 60), obere "
            "Grenze. Spannt mit 'belt_h_min' den Grünbereich auf — gleiche Hinweise.")
        self.add_parameter(
            sr.Parameter("belt_s_min", 60, sr.ParameterType.INT),
            "Grünes Band: Mindestsättigung (S, 0-255). Entsättigte Spiegelungen "
            "fallen aus der Band-Maske und werden zu Kandidaten — das Nah-Gate "
            "sortiert sie über die Tiefe aus.")
        self.add_parameter(
            sr.Parameter("belt_v_min", 40, sr.ParameterType.INT),
            "Grünes Band: Mindesthelligkeit (V, 0-255). Dunklere Bandbereiche/Schatten "
            "darunter werden zu Block-Kandidaten — das Nah-Gate fängt sie ab. Höher "
            "setzen, wenn dunkle Bandstellen fälschlich als Block durchkommen.")
        self.add_parameter(
            sr.Parameter("min_contour_area", 500.0, sr.ParameterType.DOUBLE),
            "Mindest-Blobfläche in px; kleinere Farbflächen gelten als Rauschen.")
        self.add_parameter(
            sr.Parameter("morph_kernel_size", 15, sr.ParameterType.INT),
            "Glättet die Block-Maske (Morphologie-Open, elliptischer Kernel in px): "
            "entfernt kleine Störflecken. Zu klein → Rauschen bleibt; zu groß → "
            "schmale Blockkanten/Details werden weggefressen. 0/1 = aus.")
        self.add_parameter(
            sr.Parameter("use_depth_gate", True, sr.ParameterType.BOOL),
            "Nah-Gate: behalte nur Farb-Blobs, die ein Tiefen-Loch / eine "
            "Erhebung über dem Band überlappen. Killt Band-Spiegelungen "
            "(weißer Block bleibt). Zum reinen Farb-Test abschaltbar.")
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

        # -- Inputs (endeffector RealSense only) ----------------------------------
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._aligned_depth_msg = Image()
        self.add_input("aligned_depth_image", "_aligned_depth_msg", Image)

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

    # -- Validation ----------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name in ("belt_h_min", "belt_h_max") and not (0 <= parameter.get_value() <= 179):
            self.get_logger().warn(f"{name} must be in [0, 179]")
            return False
        if name in ("belt_s_min", "belt_v_min") and not (0 <= parameter.get_value() <= 255):
            self.get_logger().warn(f"{name} must be in [0, 255]")
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
        self._last_t = 0.0
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

    def _params(self) -> RobotDetectionParams:
        return RobotDetectionParams(
            belt_h_min=int(self.get_parameter("belt_h_min").get_value()),
            belt_h_max=int(self.get_parameter("belt_h_max").get_value()),
            belt_s_min=int(self.get_parameter("belt_s_min").get_value()),
            belt_v_min=int(self.get_parameter("belt_v_min").get_value()),
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

    def on_step_callback(self):
        # Color is the primary detection source; aligned depth (in the color frame)
        # gates reflections and gives the live belt distance. Both are required.
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

            # The aligned depth lives in the color frame but may arrive at a
            # different resolution; resize it (nearest, to preserve the zero
            # holes) onto the color grid so the gate overlays pixel-exact.
            dh, dw = depth_mm.shape[:2]
            if (dw, dh) != (cw, ch):
                depth_mm = cv2.resize(depth_mm, (cw, ch), interpolation=cv2.INTER_NEAREST)

            # Color intrinsics, scaled to the (color == working) image resolution.
            iw = int(getattr(self._info_msg, "width", 0)) or cw
            ih = int(getattr(self._info_msg, "height", 0)) or ch
            sx, sy = cw / iw, ch / ih
            fx = self._info_msg.k[0] * sx
            cx = self._info_msg.k[2] * sx
            fy = self._info_msg.k[4] * sy
            cy = self._info_msg.k[5] * sy

            if not self._logged_shapes:
                self.get_logger().info(
                    f"robot_cam: Farbe {cw}x{ch}, aligned Depth {dw}x{dh}, "
                    f"CameraInfo {iw}x{ih}")
                self._logged_shapes = True

            result = detect_object(color_bgr, depth_mm, fx, fy, cx, cy,
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
            self._log_error_throttled(f"robot_cam pipeline error: {exc}")

    def _publish_debug(self, color_bgr, depth_mm, result) -> None:
        # Overlay built on the COLOR image so the operator sees both the result and
        # *why* the segmentation decided what it did:
        #   - blue tint  = depth near-gate region (object / elevated above belt)
        #   - gray lines = block candidates that did NOT win (exposes belt-mask leaks)
        #   - green      = detected contour, yellow = oriented box,
        #     red        = center + orientation line, blue dot = principal point (belt = frame median)
        params = self._params()
        debug_img = color_bgr.copy()

        # 1. Depth near-gate region (only relevant when the gate is on).
        if params.use_depth_gate:
            near = near_mask(depth_mm)
            if near.any():
                tint = debug_img.copy()
                tint[near] = (255, 90, 0)  # BGR blue
                cv2.addWeighted(tint, 0.25, debug_img, 0.75, 0, debug_img)

        # 2. All block candidates over min area; the loser(s) in thin gray.
        candidate = belt_candidate_mask(color_bgr, params)
        cnts, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) >= params.min_contour_area:
                cv2.drawContours(debug_img, [c], 0, (160, 160, 160), 1)

        # 3. The winning detection.
        if result is not None:
            cv2.drawContours(debug_img, [result.contour], 0, (0, 255, 0), 2)
            box = cv2.boxPoints(cv2.minAreaRect(result.contour)).astype(np.int32)
            cv2.polylines(debug_img, [box], True, (0, 255, 255), 2)
            rx, ry = result.ref_px
            cv2.drawMarker(debug_img, (rx, ry), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            ex = int(rx + 45 * math.cos(result.orientation_rad))
            ey = int(ry + 45 * math.sin(result.orientation_rad))
            cv2.line(debug_img, (rx, ry), (ex, ey), (0, 0, 255), 2)   # orientation
            cv2.circle(debug_img, result.belt_px, 4, (255, 0, 0), -1)  # principal point
            status = (f"z_band={result.z_band_mm:.0f}mm  x={result.x_mm:.0f}  "
                      f"y={result.y_mm:.0f}  ang={math.degrees(result.orientation_rad):.0f}deg")
        else:
            status = "NO OBJECT (kein Block-Blob >= min area / Gate)"

        # 4. Readout + color legend on a dark strip, readable over any background.
        h, w = debug_img.shape[:2]
        cv2.rectangle(debug_img, (0, 0), (w, 50), (0, 0, 0), -1)
        cv2.putText(debug_img, status, (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        legend = "gruen=Detektion gelb=Box rot=Mitte/Winkel blau=Bandpunkt grau=verworfen"
        if params.use_depth_gate:
            legend += " blaue-Flaeche=Nah-Gate"
        cv2.putText(debug_img, legend, (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.publish_output("debug_image")
