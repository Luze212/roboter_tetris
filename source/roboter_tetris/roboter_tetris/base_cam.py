"""AICA lifecycle component: conveyor-object detection from RealSense signals.

Thin shell around the pure vision modules in :mod:`roboter_tetris.vision` — a
faithful Python port of the previous group's C++ detection
(``FuE_Greifen-main/cameras/camera_static.cpp``, read-only reference). The
component consumes the AICA RealSense block's signals (color, aligned depth,
camera info) and never touches the camera hardware itself.

Output contract (unchanged): ``objects`` is a flat double array with stride 10
``[id, color, x, y, z, orientation, vy, length, width, height]`` per object —
x/y/z in mm in the robot frame, vy in mm/s, orientation in rad [0, pi).

Per-step cost is bounded: a frame is only processed when its header stamp
changed (frame gating), and the debug image is only rendered when enabled.
"""

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

from .basecam_extrinsics import DEFAULT_CALIBRATION_FILE, load_camera_calibration
from .contracts import ObjectEntry, pack_objects
from .vision.color_estimation import COLOR_NAMES
from .vision.detection import (
    DetectionParams, build_cam_to_robot, conveyor_mask, detect_objects, roi_bounds,
)
from .vision.tracker import VisionTracker

def pack_tracked_objects(timestamp: float, velocity_y_mm_s: float, tracks) -> list:
    """Pack tracker output into the S1 signal.

    The unit conversion mm -> m happens here, at the signal boundary: the vision
    modules keep working in mm, the contract is SI throughout.

    ``velocity_y_mm_s`` goes into the header rather than onto each object. The
    tracker assigns one global velocity to all tracks (``set_global_velocity``),
    so per object the value would be misleading -- a jammed block would still be
    tagged with the full belt speed.

    Module-level and free of component state, so the packing is testable without
    the modulo runtime.
    """
    return pack_objects(
        timestamp,
        velocity_y_mm_s / 1000.0,
        [ObjectEntry(id=float(obj.id), color=float(obj.color),
                     x=obj.x / 1000.0, y=obj.y / 1000.0, z=obj.z / 1000.0,
                     orientation=obj.orientation,
                     length=obj.length / 1000.0,
                     width=obj.width / 1000.0,
                     height=obj.height / 1000.0)
         for obj in tracks],
    )


# EMA low-pass for the global conveyor velocity (C++ VEL_FILTER_ALPHA).
DEFAULT_VEL_FILTER_ALPHA = 0.3
# No fresh frame for this long -> clear outputs instead of freezing stale ones.
STALE_TIMEOUT_S = 1.0
# Throttle pipeline error logging.
ERROR_LOG_PERIOD_S = 1.0

# The RealSense driver stamps in the sensor's own hardware clock unless global time
# is enabled. On the L515 that default is OFF: the stamps then sit in a foreign
# epoch, drift ~4 ms/s against ROS time and eventually stop advancing altogether --
# at which point the frame gating below discards every frame after the first and the
# object list goes silently empty. Measured at the setup on 2026-09-14; enabling the
# two parameters removed the drift (4.05 -> 0.02 ms/s). The AICA RealSense block does
# not expose them, so the component enforces them on the camera node itself.
GLOBAL_TIME_PARAMETERS = ("rgb_camera.global_time_enabled",
                          "depth_module.global_time_enabled")
GLOBAL_TIME_RETRY_PERIOD_S = 2.0
GLOBAL_TIME_MAX_ATTEMPTS = 10


class BaseCam(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()

        # -- Geometry / detection parameters (defaults from config_cam_static.yml;
        #    neutral where the C++ binary did not apply the config value) --------
        # ROI x 342...960 since 24.09.2026 (Nachtrag 13 / L21): from the belt edge
        # near the robot (u ~335) to the far limit of the workspace (x -1.0; u 960
        # keeps a block centred there whole). With 360 the belt edge near the
        # robot was cut and blocks there touched the ROI border -> discarded.
        self.add_parameter(sr.Parameter("roi_x", 342, sr.ParameterType.INT),
                           "ROI x-Offset in px (0/0/0/0 = Vollbild)")
        self.add_parameter(sr.Parameter("roi_y", 60, sr.ParameterType.INT),
                           "ROI y-Offset in px")
        self.add_parameter(sr.Parameter("roi_width", 618, sr.ParameterType.INT),
                           "ROI Breite in px")
        self.add_parameter(sr.Parameter("roi_height", 580, sr.ParameterType.INT),
                           "ROI Höhe in px")
        self.add_parameter(sr.Parameter("conveyor_z_dist", 865.0, sr.ParameterType.DOUBLE),
                           "Abstand Kamera→Fließband in mm; nur für die Trennung Band/Objekt "
                           "(Tiefenfenster), nicht mehr für die Höhe")
        self.add_parameter(sr.Parameter("belt_surface_z_mm", 53.6, sr.ParameterType.DOUBLE),
                           "Höhe der Bandoberfläche in world in mm (angetastet, B17). Die "
                           "Klotzhöhe ist z der Oberseite in world minus dieser Wert.")
        self.add_parameter(sr.Parameter("top_depth_bias_mm", 11.5, sr.ParameterType.DOUBLE),
                           "Die Kamera liest die Klotzoberseiten um diesen Wert zu tief (mm); er "
                           "wird von der Oberseitentiefe abgezogen. Gemessen 23.09.2026 an 25- "
                           "und 100-mm-Klötzen: 11,5 ± 1,6 mm (Nachtrag 13).")
        self.add_parameter(sr.Parameter("min_obj_height", 10.0, sr.ParameterType.DOUBLE),
                           "Mindesthöhe eines Objekts in mm, auf der rohen Tiefe vor "
                           "top_depth_bias_mm. 10: flache 25-mm-Klötze liegen roh nur "
                           "11-15 mm über dem Band und fielen mit 15 auf der tieferen "
                           "Bandseite heraus (Nachtrag 13 / L28; 9 gab Fehlerkennungen, L24).")
        self.add_parameter(sr.Parameter("max_obj_height_mm", 150.0, sr.ParameterType.DOUBLE),
                           "Maximale Objekthöhe in mm")
        self.add_parameter(sr.Parameter("z_offset", 0.0, sr.ParameterType.DOUBLE),
                           "Reflexions-Offset Fließband in mm (alte Config: 15; das alte "
                           "C++ wandte ihn nicht an, daher Default 0)")
        self.add_parameter(sr.Parameter("min_contour_area", 1000.0, sr.ParameterType.DOUBLE),
                           "Mindest-Konturfläche in px")
        self.add_parameter(sr.Parameter("depth_scale_to_mm", 1.0, sr.ParameterType.DOUBLE),
                           "mm pro Tiefen-Rohwert (16UC1-Bild). 1.0 = Werte sind bereits in mm "
                           "(D400-Serie). Manche Kameras (z. B. L515) liefern andere Einheiten "
                           "(z. B. 0.25). Faktor = bekannte Banddistanz / median im Debug-Bild.")
        self.add_parameter(sr.Parameter("erosion_px", 5, sr.ParameterType.INT),
                           "Erosion der Footprint-Maske (px) nur für Länge/Breite gegen den "
                           "verrauschten Tiefen-Rand. 0 = aus. Höhe/Position bleiben unberührt.")

        # -- Extrinsic calibration camera→robot ----------------------------------
        # In world (= ur_base_link, the frame AICA controls in), not in the UR
        # frame "base" (180° about z, Nachtrag 12 / K5). Interim values of
        # 23.09.2026 (Nachtrag 13): tilt from a belt-plane fit in the depth image,
        # yaw and x/y from five touch points with the robot, 25 and 100 mm blocks,
        # residual <= 4 mm. Since the base camera calibration (stage 1/2) the
        # calibration file below holds the extrinsics; these six only apply when
        # it is left empty or cannot be read. (The predecessor project's calibration
        # was expressed in "base" and is obsolete.)
        self.add_parameter(sr.Parameter("calibration_file", DEFAULT_CALIBRATION_FILE,
                                        sr.ParameterType.STRING),
                           "Kalibrierdatei der Basiskamera, relativ zum Paket oder absolut; "
                           "beim Aktivieren gelesen. Leer oder unlesbar = cal_*-Werte.")
        self.add_parameter(sr.Parameter("cal_x", -0.7787, sr.ParameterType.DOUBLE),
                           "Extrinsik: Translation x in m (Kamera→world)")
        self.add_parameter(sr.Parameter("cal_y", 0.7934, sr.ParameterType.DOUBLE),
                           "Extrinsik: Translation y in m")
        self.add_parameter(sr.Parameter("cal_z", 0.9163, sr.ParameterType.DOUBLE),
                           "Extrinsik: Translation z in m")
        self.add_parameter(sr.Parameter("cal_roll", 179.46, sr.ParameterType.DOUBLE),
                           "Extrinsik: Roll in Grad")
        self.add_parameter(sr.Parameter("cal_pitch", 0.45, sr.ParameterType.DOUBLE),
                           "Extrinsik: Pitch in Grad")
        self.add_parameter(sr.Parameter("cal_yaw", 179.76, sr.ParameterType.DOUBLE),
                           "Extrinsik: Yaw in Grad")

        # -- Optional affine correction + search area (neutral defaults; the old "
        #    config had x_offset_mm=-35 and search_area -350..190) ----------------
        self.add_parameter(sr.Parameter("x_scale", 1.0, sr.ParameterType.DOUBLE),
                           "Affine Korrektur: Skalierung x im Roboter-Frame")
        self.add_parameter(sr.Parameter("y_scale", 1.0, sr.ParameterType.DOUBLE),
                           "Affine Korrektur: Skalierung y")
        self.add_parameter(sr.Parameter("x_offset_mm", 0.0, sr.ParameterType.DOUBLE),
                           "Affine Korrektur: x-Offset in mm (alte Config: -35)")
        self.add_parameter(sr.Parameter("y_offset_mm", 0.0, sr.ParameterType.DOUBLE),
                           "Affine Korrektur: y-Offset in mm")
        self.add_parameter(sr.Parameter("search_area_y_min", -1.0e9, sr.ParameterType.DOUBLE),
                           "Suchbereich y-Min in mm, Roboter-Frame (alte Config: -350; "
                           "sehr groß = Filter aus)")
        self.add_parameter(sr.Parameter("search_area_y_max", 1.0e9, sr.ParameterType.DOUBLE),
                           "Suchbereich y-Max in mm (alte Config: 190)")

        # -- Tracker parameters (C++ tracker.hpp, turned into world: the belt runs
        #    from y = +1080 to -375, K5). The measuring region spans the whole belt
        #    since 23.09.2026 (Nachtrag 13 / L10): inside it an unseen track is
        #    deleted after track_max_missed_in_region frames, outside it the tracker
        #    carried it on with its own noisy EMA velocity -- behind the image that is
        #    now vectoring's job, with the pooled velocity. ------------------------
        self.add_parameter(sr.Parameter("track_max_match_distance_mm", 300.0, sr.ParameterType.DOUBLE),
                           "Tracker: max. Matching-Distanz in mm")
        self.add_parameter(sr.Parameter("track_min_y_mm", -375.0, sr.ParameterType.DOUBLE),
                           "Tracker: Löschen wenn y darunter (Bandende)")
        self.add_parameter(sr.Parameter("track_max_y_mm", 1080.0, sr.ParameterType.DOUBLE),
                           "Tracker: Löschen wenn y darüber (Bandanfang)")
        self.add_parameter(sr.Parameter("track_max_missed_in_region", 3, sr.ParameterType.INT),
                           "Tracker: max. verpasste Frames in der Mess-Region")
        self.add_parameter(sr.Parameter("track_velocity_region_y_min", -375.0, sr.ParameterType.DOUBLE),
                           "Tracker: Mess-Region y-Min in mm")
        self.add_parameter(sr.Parameter("track_velocity_region_y_max", 1080.0, sr.ParameterType.DOUBLE),
                           "Tracker: Mess-Region y-Max in mm")
        self.add_parameter(sr.Parameter("vel_filter_alpha", DEFAULT_VEL_FILTER_ALPHA, sr.ParameterType.DOUBLE),
                           "EMA-Tiefpass der Bandgeschwindigkeit (0-1; 0.3 = 30 % neu)")

        self.add_parameter(sr.Parameter("debug_enable", True, sr.ParameterType.BOOL),
                           "Debug-Bild erzeugen und publizieren - der interface_streamer zeigt es "
                           "(an seit dem finalen Build, Nachtrag 13 / L26)")
        self.add_parameter(sr.Parameter("camera_node", "/realsense_camera", sr.ParameterType.STRING),
                           "Node-Name des RealSense-Blocks dieser Kamera (z. B. /realsense_camera_2). "
                           "Ist er gesetzt, erzwingt die Komponente dort global_time_enabled=true, "
                           "damit die Bildstempel in ROS-Zeit laufen. Leer = aus.")

        # -- Inputs (signals from the AICA RealSense block) -----------------------
        # Queue depth 1: only the newest image counts. With modulo's default of 10
        # the frames piled up while the step was busy, and the step worked on
        # images about 0.4 s old (measured 23.09.2026, Nachtrag 13). add_input
        # subscribes immediately, so the QoS set here applies; the default is
        # restored for everything declared later (the outputs).
        default_qos = self.get_qos()
        self.set_qos(QoSProfile(depth=1))
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._depth_msg = Image()
        self.add_input("depth_image", "_depth_msg", Image)
        self.set_qos(default_qos)

        # -- Outputs ---------------------------------------------------------------
        # std_msgs signals are plain Python values: list -> Float64MultiArray.
        # S1: [t, n, v_band] + n*9. The header is sent even with no objects,
        # so "sees nothing" stays distinguishable from "no longer sending".
        self._objects = pack_tracked_objects(0.0, 0.0, [])
        self.add_output("objects", "_objects", Float64MultiArray)
        self._debug_msg = Image()
        # Not on every step: at 100 Hz that would treble the image traffic over
        # the camera rate and make dropped frames look like detection failures
        # during commissioning (N5).
        self.add_output("debug_image", "_debug_msg", Image, publish_on_step=False)

        # -- Predicates --------------------------------------------------------------
        self.add_predicate("is_receiving_frames", False)
        self.add_predicate("has_objects", False)

        # -- State -------------------------------------------------------------------
        self._tracker = VisionTracker()
        self._filtered_velocity_y = 0.0   # mm/s, as the tracker reports it
        self._last_t = 0.0                # header stamp (s) of the last processed frame
        self._last_stamp = None          # (sec, nanosec) of the last processed frame
        self._last_frame_walltime = None  # rclpy Time of the last fresh frame
        self._last_error_walltime = None
        self._set_param_client = None     # created in on_configure (ROS 2 service client)
        self._file_cam_to_robot = None    # extrinsics from the calibration file, if valid
        self._global_time_done = False
        self._global_time_attempts = 0
        self._global_time_last_try = None

    # -- Parameter validation -----------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            if name == "calibration_file":
                return True  # empty = the cal_* values
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "vel_filter_alpha":
            value = parameter.get_value()
            if value < 0.0 or value > 1.0:
                self.get_logger().warn("vel_filter_alpha must be within 0-1")
                return False
        if name in ("conveyor_z_dist", "max_obj_height_mm", "min_contour_area",
                    "track_max_match_distance_mm"):
            if parameter.get_value() <= 0.0:
                self.get_logger().warn(f"{name} must be positive")
                return False
        return True

    # -- Lifecycle ------------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        # Service clients are not abstracted by AICA; create them here (not in
        # __init__) so the UI parameters are already applied. See ARCHITECTURE.md §3.
        node_name = self.get_parameter("camera_node").get_value().strip()
        if node_name:
            if not node_name.startswith("/"):
                node_name = "/" + node_name
            self._set_param_client = self.create_client(
                SetParameters, f"{node_name}/set_parameters")
        return True

    def on_activate_callback(self) -> bool:
        # Which calibration the detection runs on, read once and logged.
        value = self.get_parameter("calibration_file").get_value() or ""
        record, line = load_camera_calibration(value)
        self._file_cam_to_robot = None if record is None else record.world_T_cam
        # one call site per severity: rclpy refuses a site whose severity changes
        if record is None and value.strip():
            self.get_logger().warn(f"base_cam: {line}")
        else:
            self.get_logger().info(f"base_cam: {line}")
        # Fresh tracking state per activation; parameters stay as configured.
        self._tracker = VisionTracker()
        self._filtered_velocity_y = 0.0
        self._last_t = 0.0
        self._last_stamp = None
        self._last_frame_walltime = None
        self._objects = pack_tracked_objects(0.0, 0.0, [])
        self._global_time_done = False
        self._global_time_attempts = 0
        self._global_time_last_try = None
        self.set_predicate("is_receiving_frames", False)
        self.set_predicate("has_objects", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Helpers ----------------------------------------------------------------------

    def _detection_params(self) -> DetectionParams:
        return DetectionParams(
            roi=(int(self.get_parameter("roi_x").get_value()),
                 int(self.get_parameter("roi_y").get_value()),
                 int(self.get_parameter("roi_width").get_value()),
                 int(self.get_parameter("roi_height").get_value())),
            conveyor_z_dist=self.get_parameter("conveyor_z_dist").get_value(),
            belt_surface_z_mm=self.get_parameter("belt_surface_z_mm").get_value(),
            top_depth_bias_mm=self.get_parameter("top_depth_bias_mm").get_value(),
            min_obj_height=self.get_parameter("min_obj_height").get_value(),
            max_obj_height_mm=self.get_parameter("max_obj_height_mm").get_value(),
            z_offset=self.get_parameter("z_offset").get_value(),
            min_contour_area=self.get_parameter("min_contour_area").get_value(),
            cam_to_robot=self._cam_to_robot(),
            x_scale=self.get_parameter("x_scale").get_value(),
            y_scale=self.get_parameter("y_scale").get_value(),
            x_offset_mm=self.get_parameter("x_offset_mm").get_value(),
            y_offset_mm=self.get_parameter("y_offset_mm").get_value(),
            search_area_y_min=self.get_parameter("search_area_y_min").get_value(),
            search_area_y_max=self.get_parameter("search_area_y_max").get_value(),
            erosion_px=int(self.get_parameter("erosion_px").get_value()),
        )

    def _cam_to_robot(self) -> np.ndarray:
        if self._file_cam_to_robot is not None:
            return self._file_cam_to_robot
        return build_cam_to_robot(
            self.get_parameter("cal_x").get_value(),
            self.get_parameter("cal_y").get_value(),
            self.get_parameter("cal_z").get_value(),
            self.get_parameter("cal_roll").get_value(),
            self.get_parameter("cal_pitch").get_value(),
            self.get_parameter("cal_yaw").get_value())

    def _sync_tracker_params(self) -> None:
        t = self._tracker
        t.max_match_distance_mm = self.get_parameter("track_max_match_distance_mm").get_value()
        t.min_tracked_y_mm = self.get_parameter("track_min_y_mm").get_value()
        t.max_tracked_y_mm = self.get_parameter("track_max_y_mm").get_value()
        t.max_missed_in_region = int(self.get_parameter("track_max_missed_in_region").get_value())
        t.velocity_region_y_min = self.get_parameter("track_velocity_region_y_min").get_value()
        t.velocity_region_y_max = self.get_parameter("track_velocity_region_y_max").get_value()

    def _log_error_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if (self._last_error_walltime is None
                or (now - self._last_error_walltime).nanoseconds / 1e9 >= ERROR_LOG_PERIOD_S):
            self.get_logger().error(message)
            self._last_error_walltime = now

    def _handle_stale(self) -> None:
        """No fresh frame: drop the objects but keep sending the header.

        ``t`` stops advancing, and that is how a receiver tells "camera works,
        sees nothing" from "camera no longer delivers" (contract rule 2).
        """
        if self._last_frame_walltime is None:
            return
        age_s = (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9
        if age_s > STALE_TIMEOUT_S:
            self._objects = pack_tracked_objects(
                self._last_t, self._filtered_velocity_y, [])
            self.set_predicate("is_receiving_frames", False)
            self.set_predicate("has_objects", False)

    # -- Camera clock domain ----------------------------------------------------------

    def _ensure_global_time(self) -> None:
        """Enforce ROS-time stamps on the camera node; retried until it sticks.

        Non-blocking as required in a step callback: readiness is polled via
        ``service_is_ready()`` and the call goes out via ``call_async`` with a done
        callback (ARCHITECTURE.md §3). The camera node may come up after this
        component, hence the retries.
        """
        if self._global_time_done or self._set_param_client is None:
            return
        if self._global_time_attempts >= GLOBAL_TIME_MAX_ATTEMPTS:
            return
        now = self.get_clock().now()
        if (self._global_time_last_try is not None
                and (now - self._global_time_last_try).nanoseconds / 1e9
                < GLOBAL_TIME_RETRY_PERIOD_S):
            return
        self._global_time_last_try = now
        if not self._set_param_client.service_is_ready():
            return  # camera node not up yet; try again after the retry period
        self._global_time_attempts += 1

        request = SetParameters.Request()
        for name in GLOBAL_TIME_PARAMETERS:
            parameter = RosParameter()
            parameter.name = name
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                             bool_value=True)
            request.parameters.append(parameter)
        future = self._set_param_client.call_async(request)
        future.add_done_callback(self._on_global_time_response)

    def _on_global_time_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"global_time_enabled konnte nicht gesetzt werden: {exc}")
            return
        rejected = [r.reason for r in response.results if not r.successful]
        if rejected:
            self.get_logger().warn(
                "Kamera hat global_time_enabled abgelehnt: " + "; ".join(rejected)
                + " — Bildstempel bleiben in der Hardwareuhr und driften.")
            return
        self._global_time_done = True
        self.get_logger().info(
            "global_time_enabled auf der Kamera gesetzt; Bildstempel laufen in ROS-Zeit.")

    # -- Periodic processing ----------------------------------------------------------

    def on_step_callback(self):
        self._ensure_global_time()
        if self._depth_msg.width == 0 or self._color_msg.width == 0:
            self._handle_stale()
            return
        if len(self._info_msg.k) < 9 or self._info_msg.k[0] <= 0.0 or self._info_msg.k[4] <= 0.0:
            # Depth/color arriving but intrinsics not. A default CameraInfo() has
            # an all-zero k of length 9, so the length check alone misses it —
            # this almost always means color_camera_info is not wired.
            self._log_error_throttled(
                "Warte auf gültige color_camera_info (fx=0) — ist der "
                "CameraInfo-Eingang mit dem Kamera-Topic verdrahtet?")
            self._handle_stale()
            return

        # Frame gating: process only when a new frame arrived (stamp changed).
        stamp = (self._depth_msg.header.stamp.sec, self._depth_msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            self._handle_stale()
            return
        self._last_stamp = stamp
        self._last_frame_walltime = self.get_clock().now()
        self.set_predicate("is_receiving_frames", True)

        try:
            fx = self._info_msg.k[0]
            cx = self._info_msg.k[2]
            fy = self._info_msg.k[4]
            cy = self._info_msg.k[5]

            color_img = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8")
            depth_img = self._bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
            if depth_img.dtype == np.float32:
                depth_mm = depth_img * 1000.0  # 32FC1 already in meters
            else:
                # 16UC1 raw depth units -> mm (1.0 for D400; e.g. 0.25 for L515)
                depth_mm = depth_img.astype(np.float32) \
                    * self.get_parameter("depth_scale_to_mm").get_value()

            params = self._detection_params()
            detections, debug_infos = detect_objects(
                color_img, depth_mm, fx, fy, cx, cy, params)

            # Tracker sequence as in the C++ main loop: predict with the filtered
            # velocity, update, low-pass the new global velocity, apply globally.
            self._sync_tracker_params()
            timestamp = stamp[0] + stamp[1] / 1e9
            self._tracker.set_prediction_velocity(self._filtered_velocity_y)
            global_vy = self._tracker.update(detections, timestamp)
            alpha = self.get_parameter("vel_filter_alpha").get_value()
            self._filtered_velocity_y = (alpha * global_vy
                                         + (1.0 - alpha) * self._filtered_velocity_y)
            self._tracker.set_global_velocity(self._filtered_velocity_y)

            tracks = self._tracker.get_active_objects()
            self._last_t = timestamp
            self._objects = pack_tracked_objects(
                timestamp, self._filtered_velocity_y, tracks)
            self.set_predicate("has_objects", len(tracks) > 0)

            if self.get_parameter("debug_enable").get_value():
                self._publish_debug(color_img, depth_mm, params, detections, debug_infos)

        except Exception as exc:
            self._log_error_throttled(f"base_cam pipeline error: {exc}")

    def _publish_debug(self, color_img, depth_mm, params, detections, debug_infos) -> None:
        """Debug overlay for tuning: ROI rectangle, conveyor-height mask (red),
        depth readouts to calibrate ``conveyor_z_dist``, and detection boxes."""
        debug_img = color_img.copy()
        rx0, ry0, rw, rh = roi_bounds(depth_mm.shape, params)
        cv2.rectangle(debug_img, (rx0, ry0), (rx0 + rw, ry0 + rh), (255, 255, 0), 1)

        # Tint the pixels the depth threshold currently selects (what becomes a
        # contour) so a wrong conveyor_z_dist is immediately visible.
        mask = conveyor_mask(depth_mm[ry0:ry0 + rh, rx0:rx0 + rw], params)
        roi_view = debug_img[ry0:ry0 + rh, rx0:rx0 + rw]
        roi_view[mask > 0] = (0, 0, 255)

        # Depth readouts to calibrate conveyor_z_dist (set it to the belt depth).
        ch, cw = depth_mm.shape[:2]
        center_d = float(depth_mm[ch // 2, cw // 2])
        valid = depth_mm[depth_mm > 0]
        median_d = float(np.median(valid)) if valid.size else 0.0
        cv2.putText(
            debug_img,
            f"center={center_d:.0f}mm  median={median_d:.0f}mm  "
            f"conveyor_z={params.conveyor_z_dist:.0f}  window=[{params.conveyor_z_dist - params.max_obj_height_mm:.0f},"
            f"{params.conveyor_z_dist - params.min_obj_height - params.z_offset:.0f}]  n={len(detections)}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        for det, info in zip(detections, debug_infos):
            cv2.polylines(debug_img, [info.box_px.reshape((-1, 1, 2))], True, (0, 255, 0), 2)
            cv2.putText(debug_img, f"ID:{det.id} {COLOR_NAMES[det.color]}", info.center_px,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.publish_output("debug_image")
