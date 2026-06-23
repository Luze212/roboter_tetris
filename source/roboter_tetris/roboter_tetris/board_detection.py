"""AICA lifecycle component: board detection for roboter_tetris.

The component detects CHARUCO or GRID boards in a color image and returns
marker/corner positions without any robot-frame transformation.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from .vision.board import (
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
            sr.Parameter("board_type", "CHARUCO", sr.ParameterType.STRING),
            "Board-Typ: CHARUCO oder GRID. GRID ist für AprilGrid/AprilTag-Arrays geeignet."
        )
        self.add_parameter(
            sr.Parameter("aruco_dictionary", "DICT_6X6_250", sr.ParameterType.STRING),
            "ArUco- oder AprilTag-Dictionary, z.B. DICT_6X6_250 oder t36h11."
        )
        self.add_parameter(
            sr.Parameter("board_rows", 5, sr.ParameterType.INT),
            "Anzahl der Quadrate bzw. Marker in Y-Richtung."
        )
        self.add_parameter(
            sr.Parameter("board_cols", 7, sr.ParameterType.INT),
            "Anzahl der Quadrate bzw. Marker in X-Richtung."
        )
        self.add_parameter(
            sr.Parameter("marker_spacing_m", 0.04, sr.ParameterType.DOUBLE),
            "Abstand zwischen Markern in Metern (für GRID) bzw. Quadratseitenlänge = Marke + Abstand (für CHARUCO)."
        )
        self.add_parameter(
            sr.Parameter("marker_length_m", 0.02, sr.ParameterType.DOUBLE),
            "Markerlänge in Metern, z.B. 0.020 für 20 mm."
        )
        self.add_parameter(
            sr.Parameter("min_detected_markers", 4, sr.ParameterType.INT),
            "Minimale Anzahl erkannter Marker, damit eine Detektion als gültig gilt."
        )
        self.add_parameter(
            sr.Parameter("debug_enable", False, sr.ParameterType.BOOL),
            "Debug-Bild erzeugen und publizieren."
        )

        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)

        self._board_corners = []
        self.add_output("board_corners", "_board_corners", Float64MultiArray)
        self._debug_msg = Image()
        self.add_output("debug_image", "_debug_msg", Image)

        self.add_predicate("has_board", False)
        self.add_predicate("is_receiving_frames", False)

        self._last_stamp = None
        self._last_frame_walltime = None
        self._last_error_walltime = None
        self._board_params_hash = None
        self._board = None
        self._dictionary = None

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        if name == "board_rows" or name == "board_cols":
            if parameter.get_value() <= 0:
                self.get_logger().warn(f"{name} must be positive")
                return False
        if name == "marker_length_m" or name == "marker_spacing_m":
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
        self.set_predicate("has_board", False)
        self.set_predicate("is_receiving_frames", False)
        self._board_params_hash = None
        self._board = None
        self._dictionary = None
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _params(self) -> BoardParams:
        return BoardParams(
            board_type=self.get_parameter("board_type").get_value(),
            aruco_dictionary=self.get_parameter("aruco_dictionary").get_value(),
            board_rows=int(self.get_parameter("board_rows").get_value()),
            board_cols=int(self.get_parameter("board_cols").get_value()),
            marker_spacing_m=self.get_parameter("marker_spacing_m").get_value(),
            marker_length_m=self.get_parameter("marker_length_m").get_value(),
            min_detected_markers=int(self.get_parameter("min_detected_markers").get_value()),
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
            self.set_predicate("has_board", False)
            self.set_predicate("is_receiving_frames", False)

    def _ensure_board(self):
        params = self._params()
        board_hash = (
            params.board_type,
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
            color_img = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8")
            params, board, dictionary = self._ensure_board()
            detection = detect_board(color_img, board, dictionary, params.board_type)

            if detection is None:
                self._board_corners = []
                self.set_predicate("has_board", False)
            else:
                marker_corners, marker_ids, board_corners, board_ids = detection
                self._board_corners = []
                try:
                    if board_ids is not None and len(board_ids) > 0:
                        # CHARUCO: use interpolated board corners
                        for marker_id, corner in zip(board_ids, board_corners):
                            self._board_corners.extend([float(int(marker_id)), float(corner[0]), float(corner[1])])
                    else:
                        # GRID: use marker centroids
                        if marker_corners is not None and len(marker_corners) > 0:
                            for i, marker_id in enumerate(marker_ids.flatten()):
                                # Handle different possible formats from cv2.aruco.detectMarkers
                                corners = marker_corners[i]
                                if len(corners.shape) == 3:  # Shape (1, 4, 2)
                                    corners = corners[0]
                                # corners should now be (4, 2) - 4 corners with x,y coords
                                center = np.mean(corners, axis=0)
                                self._board_corners.extend([float(int(marker_id)), float(center[0]), float(center[1])])
                    self.set_predicate("has_board", len(marker_ids) >= params.min_detected_markers)
                except Exception as grid_err:
                    self.get_logger().error(f"GRID processing error: {grid_err}")
                    self._board_corners = []
                    self.set_predicate("has_board", False)

            if self.get_parameter("debug_enable").get_value():
                self._publish_debug(color_img, detection, params.board_type)

        except Exception as exc:
            self._log_error_throttled(f"board_detection pipeline error: {exc}")

    def _publish_debug(self, color_img, detection, board_type):
        debug_img = color_img.copy()
        if detection is not None:
            marker_corners, marker_ids, board_corners, board_ids = detection
            debug_img = draw_board_debug(
                debug_img, marker_corners, marker_ids, board_corners, board_ids)
            label = f"{board_type} board detected"
            cv2.putText(debug_img, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "NO BOARD detected", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
