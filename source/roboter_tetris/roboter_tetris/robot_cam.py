"""AICA lifecycle component: fine-localization from the endeffector RealSense.

Thin shell around :mod:`roboter_tetris.vision.robot_detection`. It produces a
time-stamped stream of the grasp object's position under the gripper and nothing
else — the previous group's C++ ``RobotCamera`` reduced to its localization core,
intentionally decoupled from the base camera and from any pick/motion logic
(those belong to a separate processing component).

Output ``object_position`` is a flat double array ``[t, x, y, z]``:
``t`` = frame stamp in seconds, ``x``/``y`` = reference point in mm in the
**camera frame**, ``z`` = measured **belt distance** in mm (camera -> conveyor
surface, NOT the object's grasp height — correct downstream with the object
height). Empty when nothing is visible.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from .vision.robot_detection import BeltDistanceFilter, RobotDetectionParams, detect_object

STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0


class RobotCam(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()

        # -- Parameters (operator-facing descriptions) ----------------------------
        self.add_parameter(
            sr.Parameter("border_filter_mode", "none", sr.ParameterType.STRING),
            "Konturauswahl: 'none' = größte Loch-Kontur überall (Mitfahren, Default); "
            "'top' = nur Konturen am oberen Bildrand (Original-Anfahrt von oben).")
        self.add_parameter(
            sr.Parameter("reference_point_mode", "centroid", sr.ParameterType.STRING),
            "Referenzpunkt: 'centroid' = Schwerpunkt der Loch-Maske (stabil fürs Tracking, "
            "Default); 'bottom_edge' = Unterkante wie im C++-Original.")
        self.add_parameter(
            sr.Parameter("min_contour_area", 500.0, sr.ParameterType.DOUBLE),
            "Mindest-Konturfläche in px; kleinere Tiefen-Löcher gelten als Rauschen.")
        self.add_parameter(
            sr.Parameter("morph_kernel_size", 15, sr.ParameterType.INT),
            "Kernelgröße des Morphologie-Open in px (Ellipse) gegen Tiefenrauschen an Kanten.")
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
        self._depth_msg = Image()
        self.add_input("depth_image", "_depth_msg", Image)
        self._aligned_depth_msg = Image()
        self.add_input("aligned_depth_image", "_aligned_depth_msg", Image)
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._depth_camera_info_msg = CameraInfo()
        self.add_input("depth_camera_info", "_depth_camera_info_msg", CameraInfo)
        self._aligned_info_msg = CameraInfo()
        self.add_input("aligned_depth_camera_info", "_aligned_info_msg", CameraInfo)

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

    # -- Validation ----------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "border_filter_mode" and parameter.get_value() not in ("none", "top"):
            self.get_logger().warn("border_filter_mode must be 'none' or 'top'")
            return False
        if name == "reference_point_mode" and parameter.get_value() not in ("centroid", "bottom_edge"):
            self.get_logger().warn("reference_point_mode must be 'centroid' or 'bottom_edge'")
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
        self._depth_msg = Image()
        self._aligned_depth_msg = Image()
        self._color_msg = Image()
        self._info_msg = CameraInfo()
        self._depth_camera_info_msg = CameraInfo()
        self._aligned_info_msg = CameraInfo()
        self._object_position = []
        self.set_predicate("is_object_visible", False)
        self.set_predicate("is_receiving_frames", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Helpers -------------------------------------------------------------------

    def _params(self) -> RobotDetectionParams:
        return RobotDetectionParams(
            border_filter_mode=self.get_parameter("border_filter_mode").get_value(),
            reference_point_mode=self.get_parameter("reference_point_mode").get_value(),
            min_contour_area=self.get_parameter("min_contour_area").get_value(),
            morph_kernel_size=int(self.get_parameter("morph_kernel_size").get_value()),
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

    def _pick_info(self, *candidates):
        """First CameraInfo with valid intrinsics, or None."""
        for info in candidates:
            if self._valid_info(info):
                return info
        return None

    # -- Periodic processing -------------------------------------------------------

    def on_step_callback(self):
        # Depth source and its MATCHING intrinsics as a pair, so the back-projection
        # is geometrically correct: aligned depth lives in the COLOR frame -> color
        # (aligned) intrinsics; raw depth -> depth intrinsics. Aligned is preferred
        # when wired (it overlays the color image), else raw depth is used.
        if self._aligned_depth_msg.width > 0:
            depth_msg = self._aligned_depth_msg
            info_msg = self._pick_info(self._aligned_info_msg, self._info_msg)
        elif self._depth_msg.width > 0:
            depth_msg = self._depth_msg
            info_msg = self._pick_info(self._depth_camera_info_msg)
        else:
            self._handle_stale()
            return

        if info_msg is None:
            self._log_error_throttled(
                "Warte auf gültige CameraInfo zur gewählten Tiefenquelle (fx=0) — "
                "ist die passende CameraInfo verdrahtet (aligned→color/aligned, roh→depth)?")
            self._handle_stale()
            return

        stamp = (depth_msg.header.stamp.sec, depth_msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            self._handle_stale()
            return
        self._last_stamp = stamp
        self._last_frame_walltime = self.get_clock().now()
        self.set_predicate("is_receiving_frames", True)

        try:
            fx = info_msg.k[0]
            cx = info_msg.k[2]
            fy = info_msg.k[4]
            cy = info_msg.k[5]

            depth_img = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_img.dtype == np.float32:
                depth_mm = depth_img * 1000.0  # 32FC1 already in meters
            else:
                depth_mm = depth_img.astype(np.float32) \
                    * self.get_parameter("depth_scale_to_mm").get_value()  # 16UC1 -> mm

            result = detect_object(depth_mm, fx, fy, cx, cy, self._params(), self._belt_filter)

            if result is None:
                self._clear_outputs()
            else:
                t = stamp[0] + stamp[1] / 1e9
                self._object_position = [t, result.x_mm, result.y_mm, result.z_band_mm]
                self.set_predicate("is_object_visible", True)

            if self.get_parameter("debug_enable").get_value():
                self._publish_debug(depth_mm, result)

        except Exception as exc:
            self._log_error_throttled(f"robot_cam pipeline error: {exc}")

    def _publish_debug(self, depth_mm, result) -> None:
        if self._color_msg.width == 0:
            return
        debug_img = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8").copy()

        # Diagnostics: how much of the frame is a depth "hole" (== 0, i.e. too
        # close for the sensor — what the trick needs) and the valid depth range.
        n_holes = int(np.count_nonzero(depth_mm == 0))
        valid = depth_mm[depth_mm > 0]
        dmin = float(valid.min()) if valid.size else 0.0
        dmax = float(valid.max()) if valid.size else 0.0
        dmed = float(np.median(valid)) if valid.size else 0.0
        cv2.putText(
            debug_img,
            f"holes(px)={n_holes}  depth[min/med/max]={dmin:.0f}/{dmed:.0f}/{dmax:.0f}mm",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if result is not None:
            cv2.drawContours(debug_img, [result.contour], 0, (0, 255, 0), 2)
            cv2.circle(debug_img, result.ref_px, 5, (0, 0, 255), -1)        # reference point
            cv2.circle(debug_img, result.belt_px, 4, (255, 0, 0), -1)       # belt sample point
            cv2.putText(debug_img,
                        f"z_band={result.z_band_mm:.0f}mm  x={result.x_mm:.0f} y={result.y_mm:.0f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "NO OBJECT (kein Tiefen-Loch >= min area)", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
