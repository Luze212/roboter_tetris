"""AICA lifecycle component: ChArUco board detection for roboter_tetris.

The component detects a ChArUco board in a color image and returns corner
positions along with depth information from an aligned depth stream.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from ..vision.board import (
    BoardParams, build_board,
    detect_board, draw_board_debug,
)

STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0


class BoardDetection(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()

        self.add_parameter(
            sr.Parameter("aruco_dictionary", "DICT_6x6_250", sr.ParameterType.STRING),
            "ArUco-Dictionary für das Charuco-Board, z.B. DICT_6x6_250."
        )
        self.add_parameter(
            sr.Parameter("board_rows", 5, sr.ParameterType.INT),
            "Anzahl der Marker in Y-Richtung des Charuco-Boards."
        )
        self.add_parameter(
            sr.Parameter("board_cols", 7, sr.ParameterType.INT),
            "Anzahl der Marker in X-Richtung des Charuco-Boards."
        )
        self.add_parameter(
            sr.Parameter("checker_size_mm", 35.0, sr.ParameterType.DOUBLE),
            "Kantenlänge eines Checker-Felds des ChArUco-Boards in Millimetern, z.B. 35.0 für 35 mm."
        )
        self.add_parameter(
            sr.Parameter("marker_size_mm", 26.0, sr.ParameterType.DOUBLE),
            "Kantenlänge eines ArUco-Markers des ChArUco-Boards in Millimetern, z.B. 26.0 für 26 mm."
        )
        self.add_parameter(
            sr.Parameter("min_detected_markers", 4, sr.ParameterType.INT),
            "Minimale Anzahl erkannter Marker, damit eine Detektion als gültig gilt."
        )
        self.add_parameter(
            sr.Parameter("debug_enable", True, sr.ParameterType.BOOL),
            "Debug-Bild erzeugen und publizieren."
        )

        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._aligned_depth_msg = Image()
        self.add_input("aligned_depth_image", "_aligned_depth_msg", Image)

        self._board_corners = []
        self.add_output("board_corners", "_board_corners", Float64MultiArray)
        # [tx, ty, tz, rx, ry, rz]: board origin in the camera frame.
        self._board_pose = []
        self.add_output("board_pose", "_board_pose", Float64MultiArray)
        
        # Output for board center depth information [depth_mean_mm, depth_median_mm, valid_pixels_count]
        self._board_depth = []
        self.add_output("board_depth", "_board_depth", Float64MultiArray)

        self._debug_msg = Image()
        self.add_output("debug_image", "_debug_msg", Image)

        self.add_predicate("has_board", False)
        self.add_predicate("has_pose", False)
        self.add_predicate("is_receiving_frames", False)

        self._last_stamp = None
        self._last_frame_walltime = None
        self._last_error_walltime = None
        self._board_params_hash = None
        self._board = None
        self._dictionary = None
        self._last_warn_times: dict = {}

    def _warn_throttle(self, key: str, interval_s: float, msg: str):
        """Log a warning at most once per interval_s for a given key."""
        try:
            now = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            now = 0.0
        if now - self._last_warn_times.get(key, 0.0) >= interval_s:
            self._last_warn_times[key] = now
            self.get_logger().warning(msg)

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "board_rows" or name == "board_cols":
            if parameter.get_value() <= 0:
                self.get_logger().warn(f"{name} must be positive")
                return False
        if name == "checker_size_mm" or name == "marker_size_mm":
            if parameter.get_value() <= 0.0:
                self.get_logger().warn(f"{name} must be positive")
                return False
        if name == "min_detected_markers" and parameter.get_value() < 0:
            self.get_logger().warn("min_detected_markers must be non-negative")
            return False
        return True

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._last_stamp = None
        self._last_frame_walltime = None
        self._debug_msg = Image()
        self._board_corners = []
        self._board_pose = []
        self._board_depth = []
        self.set_predicate("has_board", False)
        self.set_predicate("has_pose", False)
        self.set_predicate("is_receiving_frames", False)
        self._board_params_hash = None
        self._board = None
        self._dictionary = None
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _params(self) -> BoardParams:
        aruco_dictionary = self.get_parameter("aruco_dictionary").get_value()
        board_rows = int(self.get_parameter("board_rows").get_value())
        board_cols = int(self.get_parameter("board_cols").get_value())
        checker_size_mm = float(self.get_parameter("checker_size_mm").get_value())
        marker_size_mm = float(self.get_parameter("marker_size_mm").get_value())
        min_detected_markers = int(self.get_parameter("min_detected_markers").get_value())

        if aruco_dictionary is None:
            aruco_dictionary = "DICT_6x6_250"
        if board_rows <= 0:
            board_rows = 5
        if board_cols <= 0:
            board_cols = 7
        if checker_size_mm <= 0.0:
            checker_size_mm = 35.0
        if marker_size_mm <= 0.0:
            marker_size_mm = 26.0
        if checker_size_mm <= marker_size_mm:
            raise ValueError("checker_size_mm must be greater than marker_size_mm")

        checker_size_m = checker_size_mm / 1000.0
        marker_size_m = marker_size_mm / 1000.0

        return BoardParams(
            aruco_dictionary=aruco_dictionary,
            board_rows=board_rows,
            board_cols=board_cols,
            marker_spacing_m=checker_size_m - marker_size_m,
            marker_length_m=marker_size_m,
            min_detected_markers=min_detected_markers,
        )

    def _log_error_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if (self._last_error_walltime is None
                or (now - self._last_error_walltime).nanoseconds / 1e9 >= ERROR_LOG_PERIOD_S):
            self.get_logger().error(message)
            self._last_error_walltime = now

    def _handle_stale(self) -> None:
        if self._last_frame_walltime is None:
            return
        age_s = (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9
        if age_s > STALE_TIMEOUT_S:
            self._board_corners = []
            self._board_pose = []
            self._board_depth = []
            self.set_predicate("has_board", False)
            self.set_predicate("has_pose", False)
            self.set_predicate("is_receiving_frames", False)

    def _ensure_board(self):
        params = self._params()
        board_hash = (
            params.aruco_dictionary,
            params.board_rows,
            params.board_cols,
            params.marker_spacing_m,
            params.marker_length_m,
            params.min_detected_markers,
        )
        if board_hash != self._board_params_hash:
            self._board, self._dictionary = build_board(params)
            self._board_params_hash = board_hash
        return params, self._board, self._dictionary

    def _camera_parameters(self):
        if len(self._info_msg.k) != 9 or not any(self._info_msg.k):
            return None
        camera_matrix = np.asarray(self._info_msg.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(self._info_msg.d, dtype=np.float64).reshape(-1, 1)
        if distortion.size == 0:
            distortion = np.zeros((5, 1), dtype=np.float64)
        return camera_matrix, distortion

    def _estimate_pose(self, board_corners, board_ids, board):
        camera_parameters = self._camera_parameters()
        if camera_parameters is None or len(board_ids) < 4:
            return None

        camera_matrix, distortion = camera_parameters
        valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            board_corners.reshape(-1, 1, 2).astype(np.float32),
            board_ids.reshape(-1, 1).astype(np.int32),
            board, camera_matrix, distortion, None, None)
        if not valid:
            return None

        rvec = rvec.reshape(3)
        tvec = tvec.reshape(3)

        return rvec, tvec, camera_matrix, distortion

    def on_step_callback(self):
        if self._color_msg.width == 0:
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
            # Exactly like the working legacy version: direct bgr8 loading
            color_img = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8")
            ch, cw = color_img.shape[:2]

            # Optional aligned depth extraction
            depth_mm = None
            if self._aligned_depth_msg.width > 0:
                depth_img = self._bridge.imgmsg_to_cv2(self._aligned_depth_msg, desired_encoding="passthrough")
                if depth_img.dtype == np.float32:
                    depth_mm = depth_img * 1000.0
                else:
                    depth_mm = depth_img.astype(np.float32)
                dh, dw = depth_mm.shape[:2]
                if (dw, dh) != (cw, ch):
                    depth_mm = cv2.resize(depth_mm, (cw, ch), interpolation=cv2.INTER_NEAREST)

            params, board, dictionary = self._ensure_board()
            detection = detect_board(color_img, board, dictionary)
            pose = None

            if detection is None:
                self._board_corners = []
                self._board_pose = []
                self._board_depth = []
                self.set_predicate("has_board", False)
                self.set_predicate("has_pose", False)
                self._warn_throttle("no_board", 2.0, "Kein ChArUco-Board im Kamerabild erkannt (0 Marker).")
            else:
                marker_corners, marker_ids, board_corners, board_ids = detection
                num_markers = len(marker_ids) if marker_ids is not None else 0
                self._board_corners = []
                for corner_id, corner in zip(board_ids, board_corners):
                    self._board_corners.extend([
                        float(int(corner_id)),
                        float(corner[0]),
                        float(corner[1]),
                    ])
                has_enough = num_markers >= params.min_detected_markers
                self.set_predicate("has_board", has_enough)

                if not has_enough:
                    self._warn_throttle(
                        "too_few", 2.0,
                        f"Zu wenige ChArUco-Marker erkannt: {num_markers}/{params.min_detected_markers} Mindest-Marker."
                    )

                pose = self._estimate_pose(board_corners, board_ids, board)
                if pose is None:
                    self._board_pose = []
                    self._board_depth = []
                    self.set_predicate("has_pose", False)
                    if has_enough:
                        self._warn_throttle("no_pose", 2.0, "Board-Pose konnte trotz ausreichend Marker nicht berechnet werden.")
                else:
                    rvec, tvec, _, _ = pose
                    self._board_pose = [*map(float, tvec), *map(float, rvec)]
                    self.set_predicate("has_pose", True)

                    # Tiefenwerte aus der Board-Region auslesen
                    if depth_mm is not None and len(board_corners) > 0:
                        pts = board_corners.reshape(-1, 2).astype(np.int32)
                        x_min, y_min = np.clip(pts.min(axis=0), 0, [cw - 1, ch - 1])
                        x_max, y_max = np.clip(pts.max(axis=0), 0, [cw - 1, ch - 1])
                        
                        roi_depth = depth_mm[y_min:y_max+1, x_min:x_max+1]
                        valid_depths = roi_depth[(roi_depth > 0) & (~np.isnan(roi_depth))]
                        
                        if valid_depths.size > 0:
                            d_mean = float(np.mean(valid_depths))
                            d_median = float(np.median(valid_depths))
                            valid_count = float(valid_depths.size)
                            self._board_depth = [d_mean, d_median, valid_count]
                        else:
                            self._board_depth = []
                    else:
                        self._board_depth = []

            if self.get_parameter("debug_enable").get_value():
                self._publish_debug(color_img, detection, pose, params.marker_length_m + params.marker_spacing_m)

        except Exception as exc:
            self._board_pose = []
            self._board_depth = []
            self.set_predicate("has_pose", False)
            self._log_error_throttled(f"board_detection pipeline error: {exc}")

    def _publish_debug(self, color_img, detection, pose, checker_size_m):
        debug_img = color_img.copy()

        # Mittelpunkt-Markierung (Dunkles Gelb)
        height, width = debug_img.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        dark_yellow = (0, 180, 180)
        line_thickness = 1

        cv2.line(debug_img, (0, center_y), (width, center_y), dark_yellow, line_thickness)
        cv2.line(debug_img, (center_x, 0), (center_x, height), dark_yellow, line_thickness)

        if detection is not None:
            marker_corners, marker_ids, board_corners, board_ids = detection
            debug_img = draw_board_debug(
                debug_img, marker_corners, marker_ids, board_corners, board_ids)
            if pose is not None:
                rvec, tvec, camera_matrix, distortion = pose
                cv2.drawFrameAxes(
                    debug_img, camera_matrix, distortion, rvec, tvec,
                    float(3.0 * checker_size_m), 2)
            cv2.putText(debug_img, "CHARUCO board detected", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "NO BOARD detected", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self._debug_msg.header = self._color_msg.header