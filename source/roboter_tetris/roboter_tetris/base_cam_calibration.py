"""AICA lifecycle component: calibration of the base camera (camera -> world).

Thin shell around :mod:`roboter_tetris.calibration_run` (sequence) and
:mod:`roboter_tetris.basecam_extrinsics` (mathematics). A run starts when the
block is activated -- in the calibration application a button loads it -- and
ends in FERTIG or FEHLER; the block then idles until it is unloaded.

Mode ``stufe1`` moves the robot: the target pose goes to the attractor input of
the Signal Point Attractor. Before the first frame is judged the block commands
the flange where it stands, and it moves only after the plan passed every
clearance check. Unloading the block stops the run; the attractor holds the
last target. Modes ``stufe2`` and ``pruefen`` never send a target.

The result is written to ``output_file`` inside the container. It enters the
package -- and with it base_cam -- only when it is copied into the repository
(``roboter_tetris/Extrinsics/base_cam_extrinsics.json``) and the package is
built again: every calibration in force is a file under version control.
"""

import json

import cv2
import numpy as np
from cv_bridge import CvBridge
from clproto import MessageType
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
import state_representation as sr
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image

from .basecam_extrinsics import (
    L6_CAL, load_camera_calibration, make_tag_detector, matrix_from_cal, pose_from_quaternion,
    quaternion_from_matrix, replace_calibration, required_start_height, resolve_calibration_path,
)
from .calibration_run import (
    FEHLER, FERTIG, PARAMETERS, Camera, CalibrationRun, Frame, run_params,
)
from .contracts import S6_REFERENCE_FRAME

#: Flange pose older than this counts as missing (as in object_follower).
ROBOT_STATE_MAX_AGE_S = 0.5
GRID_COLOR = (0, 200, 0)
REFERENCE_COLOR = (255, 120, 0)
_OWN = {name for name, _, _ in PARAMETERS}


def _parameter_type(default):
    if isinstance(default, bool):
        return sr.ParameterType.BOOL
    if isinstance(default, int):
        return sr.ParameterType.INT
    if isinstance(default, float):
        return sr.ParameterType.DOUBLE
    return sr.ParameterType.STRING


class BaseCamCalibration(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()
        for name, default, description in PARAMETERS:
            self.add_parameter(sr.Parameter(name, default, _parameter_type(default)), description)

        # -- Inputs: newest image only (queue depth 1, as base_cam) ----------------
        default_qos = self.get_qos()
        self.set_qos(QoSProfile(depth=1))
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._depth_msg = Image()
        self.add_input("depth_image", "_depth_msg", Image)
        self.set_qos(default_qos)
        self._robot_state = sr.CartesianState()
        self.add_input("robot_state", "_robot_state", EncodedState,
                       user_callback=self._on_robot_state)

        # -- Outputs ------------------------------------------------------------------
        # Empty until the run commands: an empty state is not published, so the
        # attractor keeps what it had (and stufe2/pruefen never send anything).
        self._target_pose = sr.CartesianPose("target_pose", S6_REFERENCE_FRAME)
        self.add_output("target_pose", "_target_pose", EncodedState,
                        MessageType.CARTESIAN_POSE_MESSAGE)
        self._debug_msg = Image()
        self.add_output("debug_image", "_debug_msg", Image, publish_on_step=False)

        self.add_predicate("is_running", False)
        self.add_predicate("is_finished", False)
        self.add_predicate("has_failed", False)
        self.add_predicate("camera_moved", False)

        self._run = None
        self._detector = None
        self._detector_name = None
        self._flange = None          # (receive time s, 4x4)
        self._last_stamp = None
        self._written = False
        self._raw_written = False
        self._frame_reported = False

    # -- Parameters ---------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        # Only our own: modulo adds a "<signal>_topic" string per signal, and a
        # check on those left a component without ports once (Nachtrag 13 / L17).
        name = parameter.get_name()
        if name not in _OWN:
            return True
        if parameter.is_empty():
            if name == "calibration_file":
                return True
            self.get_logger().warn(f"{name} darf nicht leer sein")
            return False
        value = parameter.get_value()
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if name != "belt_z_m" and not name.startswith("depth_error_") and value < 0:
                self.get_logger().warn(f"{name} darf nicht negativ sein")
                return False
        return True

    def _values(self) -> dict:
        return {name: self.get_parameter(name).get_value() for name in _OWN}

    # -- Lifecycle ----------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        values = self._values()
        params = run_params(values)
        problems = params.problems()
        if problems:
            self.get_logger().error("base_cam_calibration: " + "; ".join(problems))
            return False
        try:
            if values["tag_dictionary"] != self._detector_name:
                self._detector = make_tag_detector(values["tag_dictionary"])
                self._detector_name = values["tag_dictionary"]
        except ValueError as exc:
            self.get_logger().error(f"base_cam_calibration: {exc}")
            return False
        stored, line = load_camera_calibration(values["calibration_file"])
        # one call site per severity: rclpy refuses a site whose severity changes
        if stored is not None:
            self.get_logger().info(f"base_cam_calibration: {line}")
        else:
            self.get_logger().warn(f"base_cam_calibration: {line}")
        self._run = CalibrationRun(params, stored, matrix_from_cal(*L6_CAL))
        self._target_pose = sr.CartesianPose("target_pose", S6_REFERENCE_FRAME)
        self._written = False
        self._raw_written = False
        self._last_stamp = None
        for name in ("is_running", "is_finished", "has_failed", "camera_moved"):
            self.set_predicate(name, False)
        self.set_predicate("is_running", True)
        start = ""
        if params.mode == "stufe1":
            start = (f" Der Roboter bewegt sich. Startpose: Werkzeug waagerecht, Board flach "
                     f"und mittig im Bild, Board-Mitte mindestens z = "
                     f"{required_start_height(params.plan):.2f} m (world).")
        self.get_logger().info(f"base_cam_calibration: Lauf {params.mode} beginnt.{start}")
        return True

    def on_deactivate_callback(self) -> bool:
        if self._run is not None and self._run.running:
            self.get_logger().warn("base_cam_calibration: Lauf abgebrochen - der Attractor hält "
                                   "die letzte Zielpose")
        self._run = None
        self.set_predicate("is_running", False)
        return True

    # -- Inputs -------------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_robot_state(self) -> None:
        if self._robot_state.is_empty():
            return
        try:
            pose = pose_from_quaternion(self._robot_state.get_position(),
                                        self._robot_state.get_orientation_coefficients())
        except ValueError:
            return
        self._flange = (self._now_s(), pose)
        if not self._frame_reported:
            self._frame_reported = True
            frame = self._robot_state.get_reference_frame()
            if frame != S6_REFERENCE_FRAME:
                self.get_logger().warn(f"base_cam_calibration: robot_state kommt in '{frame}', "
                                       f"erwartet '{S6_REFERENCE_FRAME}'")

    def _current_flange(self):
        if self._flange is None:
            return None
        received, pose = self._flange
        return pose if self._now_s() - received <= ROBOT_STATE_MAX_AGE_S else None

    def _camera(self):
        info = self._info_msg
        if len(info.k) < 9 or info.k[0] <= 0.0 or info.width == 0:
            return None
        D = np.asarray(info.d, dtype=np.float64) if len(info.d) else np.zeros(5)
        return Camera(np.asarray(info.k, dtype=np.float64).reshape(3, 3), D,
                      (int(info.width), int(info.height)))

    def _depth_m(self):
        if self._depth_msg.width == 0:
            return None
        depth = self._bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
        if depth.dtype == np.float32:
            return depth.astype(np.float64)
        return depth.astype(np.float64) * self.get_parameter("depth_scale_to_mm").get_value() / 1000.0

    # -- Step ---------------------------------------------------------------------

    def on_step_callback(self):
        run = self._run
        if run is None:
            return
        debug = bool(self.get_parameter("debug_enable").get_value())
        frame = None
        if self._color_msg.width > 0:
            stamp = (self._color_msg.header.stamp.sec, self._color_msg.header.stamp.nanosec)
            if stamp != self._last_stamp and (run.wants_frames() or debug):
                self._last_stamp = stamp
                try:
                    color = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8")
                    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                    detections = self._detector(gray)
                    if run.wants_frames():
                        depth = self._depth_m() if run.needs_depth else None
                        frame = Frame(stamp[0] + stamp[1] / 1e9, detections, depth, gray)
                    if debug:
                        self._publish_debug(color, detections, run)
                except Exception as exc:
                    self.get_logger().error(f"base_cam_calibration: Bild: {exc}")

        target = run.step(self._now_s(), self._current_flange(), self._camera(), frame)
        if target is not None:
            self._target_pose.set_position(*(float(v) for v in target[:3, 3]))
            self._target_pose.set_orientation(list(quaternion_from_matrix(target[:3, :3])))
        for line in run.pop_events():
            if line.startswith("FEHLER"):
                self.get_logger().error(f"base_cam_calibration: {line}")
            else:
                self.get_logger().info(f"base_cam_calibration: {line}")

        if run.state == FERTIG and run.result is not None and not self._written:
            self._write(run)
        if not run.running and not self._raw_written:
            self._write_raw(run)
        self.set_predicate("is_running", run.running)
        self.set_predicate("is_finished", run.state == FERTIG)
        self.set_predicate("has_failed", run.state == FEHLER)
        self.set_predicate("camera_moved", bool(run.moved))

    def _write(self, run) -> None:
        self._written = True
        path = resolve_calibration_path(self.get_parameter("output_file").get_value())
        try:
            backup = replace_calibration(path, run.result)
        except OSError as exc:
            self.get_logger().error(f"base_cam_calibration: {path} nicht schreibbar: {exc}")
            self.set_predicate("has_failed", True)
            return
        cal = run.result.cal()
        kept = f" Die bisherige liegt als {backup}." if backup else ""
        self.get_logger().info(
            f"base_cam_calibration: Kalibrierung für base_cam geschrieben: {path} - "
            + ", ".join(f"{k} {v:.4f}" for k, v in cal.items())
            + f". base_cam nutzt sie ab dem nächsten Aktivieren.{kept} Dauerhaft erst "
              "nach docker cp ins Repo (roboter_tetris/Extrinsics/) und Build.")

    def _write_raw(self, run) -> None:
        """Raw samples next to the result, whatever the outcome: a refused run
        can be evaluated without driving it again."""
        self._raw_written = True
        data = run.raw_data()
        if data is None:
            return
        path = self.get_parameter("raw_file").get_value()
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f)
        except OSError as exc:
            self.get_logger().error(f"base_cam_calibration: {path} nicht schreibbar: {exc}")
            return
        self.get_logger().info(f"base_cam_calibration: Rohdaten geschrieben: {path}")

    def _publish_debug(self, color, detections, run) -> None:
        image = color.copy()
        first_ref = run.params.reference_first_id
        for tag_id, corners in detections.items():
            c = GRID_COLOR if tag_id < first_ref else REFERENCE_COLOR
            pts = np.round(corners).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(image, [pts], True, c, 2)
            if tag_id >= first_ref:
                cv2.putText(image, str(tag_id), tuple(int(v) for v in corners[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        board = sum(1 for i in detections if i < first_ref)
        refs = len(detections) - board
        done, total = run.progress
        lines = [f"{run.params.mode}  {run.state}  Pose {done}/{total}",
                 f"Board-Tags {board}  Referenzmarken {refs}"]
        for n, text in enumerate(lines):
            cv2.putText(image, text, (10, 28 + 26 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 4)
            cv2.putText(image, text, (10, 28 + 26 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 1)
        self._debug_msg = self._bridge.cv2_to_imgmsg(image, "bgr8")
        self._debug_msg.header = self._color_msg.header
        self.publish_output("debug_image")
