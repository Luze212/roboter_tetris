"""AICA Lifecycle Component: Calibration Test Drive for roboter_tetris.

Validates the extrinsic calibration by reading calibration.yaml/json (T_robot_conveyor & T_ee_robot_cam),
and moving the robot camera/end-effector back and forth along the Y-axis of the conveyor frame
when triggered via a Service / Event button in AICA Studio.
"""

import json
import math
import os
from typing import Optional, Tuple

import numpy as np
import yaml
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from modulo_interfaces.srv import StringTrigger


def rpy_to_rotation_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0, math.sin(roll_rad), math.cos(roll_rad)],
    ], dtype=np.float64)
    ry = np.array([
        [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
        [0, 1, 0],
        [-math.sin(pitch_rad), 0, math.cos(pitch_rad)],
    ], dtype=np.float64)
    rz = np.array([
        [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
        [math.sin(yaw_rad), math.cos(yaw_rad), 0],
        [0, 0, 1],
    ], dtype=np.float64)
    return rz @ ry @ rx


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = np.dot(q1, q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        res = q1 + t * (q2 - q1)
        return res / np.linalg.norm(res)
    theta_0 = math.acos(min(max(dot, -1.0), 1.0))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s1 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s2 = sin_theta / sin_theta_0
    res = s1 * q1 + s2 * q2
    return res / np.linalg.norm(res)


class CalibrationTestDrive(LifecycleComponent):
    """AICA component for executing a test drive along the conveyor Y-axis to validate calibration."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Parameters
        self.add_parameter(
            sr.Parameter(
                "calibration_file_path",
                "/home/tetripick/Desktop/AICA/roboter_tetris/calibration.yaml",
                sr.ParameterType.STRING
            ),
            "Pfad zur calibration.yaml oder calibration.json"
        )
        self.add_parameter(
            sr.Parameter("test_distance_y_mm", 200.0, sr.ParameterType.DOUBLE),
            "Fahrstrecke entlang der Y-Achse des Förderbandes in mm"
        )
        self.add_parameter(
            sr.Parameter("drive_speed_m_s", 0.05, sr.ParameterType.DOUBLE),
            "Geschwindigkeit der Testfahrt in m/s"
        )
        self.add_parameter(
            sr.Parameter("center_over_board", True, sr.ParameterType.BOOL),
            "Zentriert das Fadenkreuz der Kamera vor Start über dem berechneten ChArUco-Board-Zentrum"
        )
        self.add_parameter(
            sr.Parameter("phase_pause_s", 2.0, sr.ParameterType.DOUBLE),
            "Pausenzeit (s) zwischen den einzelnen Phasen der Testfahrt"
        )

        # Inputs
        self._robot_ee_pose = sr.CartesianState("end_effector", "world")
        self.add_input(
            "robot_ee_pose",
            "_robot_ee_pose",
            EncodedState
        )
        # Board geometry [board_rows, board_cols, checker_size_mm] published by board_detection
        self._board_geometry_msg = []
        self.add_input("board_geometry", "_board_geometry_msg", Float64MultiArray)

        # Outputs
        self._target_pose = sr.CartesianPose("calibration_test_target", "world")
        self.add_output(
            "target_ee_pose",
            "_target_pose",
            EncodedState,
            MessageType.CARTESIAN_POSE_MESSAGE
        )

        # Predicates
        self.add_predicate("is_running", False)
        self.add_predicate("has_failed", False)

        # Service
        try:
            self.add_service(
                "start_test_drive",
                StringTrigger,
                self._on_start_test_drive_service
            )
        except Exception:
            pass

        # State machine
        self._state = "IDLE"
        self._state_start_time = None
        self._start_position_robot = None
        self._start_orientation_robot = None
        self._initial_position_robot = None
        self._initial_orientation_robot = None
        self._T_robot_conveyor = None
        self._T_ee_robot_cam = None

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._state_start_time = None
        self.set_predicate("is_running", False)
        self.set_predicate("has_failed", False)

        self._on_start_test_drive()
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _get_ee_position_m(self) -> np.ndarray:
        """Liest die Roboter-EE-Position und garantiert Rückgabe in Metern."""
        pos = self._robot_ee_pose.get_position()
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if abs(x) > 2.0 or abs(y) > 2.0 or abs(z) > 2.0:
            x /= 1000.0
            y /= 1000.0
            z /= 1000.0
        return np.array([x, y, z], dtype=np.float64)

    def _get_ee_rotation_matrix(self) -> np.ndarray:
        ori = self._robot_ee_pose.get_orientation()
        if hasattr(ori, "to_rotation_matrix"):
            return np.array(ori.to_rotation_matrix(), dtype=np.float64)
        else:
            qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
            return np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
            ], dtype=np.float64)

    def _load_calibration_data(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        calib_path = self.get_parameter("calibration_file_path").get_value()
        if not os.path.exists(calib_path):
            candidate_fallbacks = [
                "/home/tetripick/Desktop/AICA/roboter_tetris/calibration.yaml",
                "/home/tetripick/Desktop/AICA/roboter_tetris/calibration.json",
                "/tmp/calibration.yaml",
                "/tmp/calibration.json"
            ]
            fallback_found = None
            for fb in candidate_fallbacks:
                if os.path.exists(fb):
                    fallback_found = fb
                    break

            if fallback_found:
                self.get_logger().info(
                    f"Configured calibration file {calib_path} not found. "
                    f"Using fallback: {fallback_found}"
                )
                calib_path = fallback_found
            else:
                self.get_logger().error(f"Calibration file not found at {calib_path} or fallbacks {candidate_fallbacks}")
                return None, None

        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                if calib_path.endswith((".yaml", ".yml")):
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)

            T_robot_conveyor = None
            T_ee_robot_cam = None

            if isinstance(data, dict):
                self._calib_data = data
                
                # 1. T_robot_conveyor laden
                if "transformations" in data and "T_robot_conveyor" in data["transformations"]:
                    mat = data["transformations"]["T_robot_conveyor"]["homogeneous_matrix"]
                    T_robot_conveyor = np.array(mat, dtype=np.float64)
                elif "conveyor_frame" in data and "matrix_4x4" in data["conveyor_frame"]:
                    T_robot_conveyor = np.array(data["conveyor_frame"]["matrix_4x4"], dtype=np.float64)
                elif "homogeneous_matrix" in data:
                    T_robot_conveyor = np.array(data["homogeneous_matrix"], dtype=np.float64)
                elif "T_robot_conveyor" in data:
                    T_robot_conveyor = np.array(data["T_robot_conveyor"], dtype=np.float64)

                # 2. T_ee_robot_cam laden
                if "transformations" in data and "T_ee_robot_cam" in data["transformations"]:
                    mat_cam = data["transformations"]["T_ee_robot_cam"]["homogeneous_matrix"]
                    T_ee_robot_cam = np.array(mat_cam, dtype=np.float64)
                elif "T_ee_robot_cam" in data:
                    T_ee_robot_cam = np.array(data["T_ee_robot_cam"], dtype=np.float64)

            if T_robot_conveyor is None:
                self.get_logger().error(f"No valid T_robot_conveyor matrix found in {calib_path}")

            if T_ee_robot_cam is None:
                self.get_logger().warn(
                    f"T_ee_robot_cam not found in {calib_path}. Assuming camera is at EE origin."
                )
                T_ee_robot_cam = np.eye(4, dtype=np.float64)
            elif np.linalg.norm(T_ee_robot_cam[:3, 3]) > 0.15:
                self.get_logger().warn(
                    f"Unphysical T_ee_robot_cam translation detected ({np.linalg.norm(T_ee_robot_cam[:3, 3])*1000:.1f}mm). "
                    "Defaulting camera translation to flange origin."
                )
                T_ee_robot_cam[:3, 3] = 0.0

            return T_robot_conveyor, T_ee_robot_cam

        except Exception as e:
            self.get_logger().error(f"Failed to parse calibration file {calib_path}: {e}")
            return None, None

    def _load_board_center_conveyor(self) -> Optional[tuple]:
        data = getattr(self, "_calib_data", None)
        if data is None:
            return None
        try:
            bc = data.get("board_center_conveyor_mm", {})
            if bc and bc.get("x") is not None and bc.get("y") is not None:
                x_m = float(bc["x"]) / 1000.0
                y_m = float(bc["y"]) / 1000.0
                z_m = float(bc.get("z", 0.0)) / 1000.0

                # Determine board dimensions
                geom = []
                if hasattr(self._board_geometry_msg, "data") and len(self._board_geometry_msg.data) >= 3:
                    geom = list(self._board_geometry_msg.data)
                elif isinstance(self._board_geometry_msg, (list, tuple, np.ndarray)) and len(self._board_geometry_msg) >= 3:
                    geom = list(self._board_geometry_msg)

                if len(geom) >= 3 and geom[1] > 0 and geom[2] > 0:
                    board_rows = int(round(geom[0]))
                    board_cols = int(round(geom[1]))
                    checker_size_m = float(geom[2]) / 1000.0
                    board_w_m = board_cols * checker_size_m
                    board_h_m = board_rows * checker_size_m
                else:
                    # Default 5x7 @ 35mm board: 245mm x 175mm
                    board_w_m = 0.245
                    board_h_m = 0.175

                # Check if loaded x_m is the board corner origin (e.g. x_m >= -0.18 m, such as -0.13m / -0.128m)
                # If so, convert corner origin to geometric board center in conveyor frame:
                # X_center = X_origin - board_w / 2.0
                # Y_center = Y_origin + board_h / 2.0
                if x_m > -0.18:
                    x_orig_m, y_orig_m = x_m, y_m
                    x_m = x_orig_m - board_w_m / 2.0
                    y_m = y_orig_m + board_h_m / 2.0
                    self.get_logger().info(
                        f"Detected board corner origin in calibration file (X={x_orig_m*1000:.1f} mm, Y={y_orig_m*1000:.1f} mm). "
                        f"Automatically converted to geometric board center: X={x_m*1000:.1f} mm, Y={y_m*1000:.1f} mm"
                    )
                else:
                    self.get_logger().info(
                        f"Board center from calibration: X={x_m*1000:.1f} mm, Y={y_m*1000:.1f} mm "
                        f"(in conveyor_frame)"
                    )
                return (x_m, y_m, z_m)
        except Exception as e:
            self.get_logger().warn(f"Error loading board center: {e}")
            pass
        return None

    def _on_start_test_drive_service(
        self,
        request: StringTrigger.Request
    ) -> StringTrigger.Response:
        response = StringTrigger.Response()

        if self._state not in ("IDLE", "FINISHED", "FAILED"):
            response.success = False
            response.message = f"Testfahrt läuft bereits (Zustand: {self._state})."
            return response

        success = self._on_start_test_drive()
        response.success = success
        response.message = (
            "Testfahrt gestartet."
            if success
            else "Testfahrt konnte nicht gestartet werden (Fehler in calibration.yaml/json)."
        )
        return response

    def _on_start_test_drive(self) -> bool:
        T_robot_conveyor, T_ee_robot_cam = self._load_calibration_data()

        if T_robot_conveyor is None:
            self.set_predicate("has_failed", True)
            return False

        self._T_robot_conveyor = T_robot_conveyor
        self._T_ee_robot_cam = T_ee_robot_cam

        X_conv = T_robot_conveyor[:3, 0]
        Y_conv = T_robot_conveyor[:3, 1]
        Z_conv = T_robot_conveyor[:3, 2]
        self.get_logger().info("=== Conveyor Frame Axes in robot_base (world) ===")
        self.get_logger().info(f"  X_conv (Width) : [{X_conv[0]:.4f}, {X_conv[1]:.4f}, {X_conv[2]:.4f}]")
        self.get_logger().info(f"  Y_conv (Flow)  : [{Y_conv[0]:.4f}, {Y_conv[1]:.4f}, {Y_conv[2]:.4f}]")
        self.get_logger().info(f"  Z_conv (Height): [{Z_conv[0]:.4f}, {Z_conv[1]:.4f}, {Z_conv[2]:.4f}]")

        curr_ee_pos = self._get_ee_position_m()
        R_ee_curr = self._get_ee_rotation_matrix()

        self._initial_position_robot = curr_ee_pos.copy()
        self._initial_orientation_robot = rotation_matrix_to_quaternion(R_ee_curr)

        # Target camera orientation parallel to conveyor (looking straight down at conveyor surface)
        R_conv = T_robot_conveyor[:3, :3]
        R_y_180 = rpy_to_rotation_matrix(0.0, math.radians(180.0), 0.0)
        R_cam_target = R_conv @ R_y_180

        # Target EE Flange orientation
        R_ee_cam = T_ee_robot_cam[:3, :3]
        R_ee_target = R_cam_target @ R_ee_cam.T
        quat_ee_target = rotation_matrix_to_quaternion(R_ee_target)

        center_over_board = bool(self.get_parameter("center_over_board").get_value())
        board_center = self._load_board_center_conveyor()

        if center_over_board and board_center is not None:
            bc_x_m, bc_y_m, _ = board_center

            # t_ee_cam: Translation of camera optical center relative to EE flange, in meters
            t_ee_cam = T_ee_robot_cam[:3, 3]
            # Sanity check: if the norm is unrealistically large (e.g. stored in mm), scale it
            if np.linalg.norm(t_ee_cam) > 0.5:
                self.get_logger().warn(
                    f"t_ee_cam norm ({np.linalg.norm(t_ee_cam)*1000:.1f}mm) seems large, scaling from mm to m."
                )
                t_ee_cam = t_ee_cam / 1000.0

            # Current camera optical center position in world frame
            curr_cam_world = curr_ee_pos + R_ee_curr @ t_ee_cam

            # Transform current camera pos to conveyor frame to get current Z height above conveyor
            T_inv = np.linalg.inv(T_robot_conveyor)
            curr_cam_conv = T_inv @ np.array([curr_cam_world[0], curr_cam_world[1], curr_cam_world[2], 1.0], dtype=np.float64)

            # Target camera position: board center in XY, same height Z above conveyor
            p_cam_target_conv = np.array([bc_x_m, bc_y_m, float(curr_cam_conv[2]), 1.0], dtype=np.float64)
            p_cam_target_world = (T_robot_conveyor @ p_cam_target_conv)[:3]

            # Required EE flange position so camera optical center is above board center
            # EE_pos = cam_target_world - R_ee_target * t_ee_cam
            target_ee_world = p_cam_target_world - R_ee_target @ t_ee_cam

            delta_dist = np.linalg.norm(target_ee_world - curr_ee_pos)

            self.get_logger().info(
                f"=== Phase 0: Camera Centering ===")
            self.get_logger().info(
                f"  t_ee_cam offset: [{t_ee_cam[0]*1000:.1f}, {t_ee_cam[1]*1000:.1f}, {t_ee_cam[2]*1000:.1f}] mm")
            self.get_logger().info(
                f"  Current EE pos (world): [{curr_ee_pos[0]*1000:.1f}, {curr_ee_pos[1]*1000:.1f}, {curr_ee_pos[2]*1000:.1f}] mm")
            self.get_logger().info(
                f"  Current Camera pos (conveyor): X={curr_cam_conv[0]*1000:.1f} mm, Y={curr_cam_conv[1]*1000:.1f} mm, Z={curr_cam_conv[2]*1000:.1f} mm")
            self.get_logger().info(
                f"  Board center target (conveyor): X={bc_x_m*1000:.1f} mm, Y={bc_y_m*1000:.1f} mm")
            self.get_logger().info(
                f"  Target cam pos (world): [{p_cam_target_world[0]*1000:.1f}, {p_cam_target_world[1]*1000:.1f}, {p_cam_target_world[2]*1000:.1f}] mm")
            self.get_logger().info(
                f"  Target EE pos (world): [{target_ee_world[0]*1000:.1f}, {target_ee_world[1]*1000:.1f}, {target_ee_world[2]*1000:.1f}] mm")
            self.get_logger().info(
                f"  Travel distance: {delta_dist*1000:.1f} mm")

            self._start_position_robot = target_ee_world.copy()
            self._start_orientation_robot = quat_ee_target
            self._state = "CENTERING"
        else:
            self._start_position_robot = curr_ee_pos.copy()
            self._start_orientation_robot = quat_ee_target
            self._state = "MOVING_FORWARD"

        self._state_start_time = self.get_clock().now()
        self.set_predicate("is_running", True)
        self.set_predicate("has_failed", False)
        return True

    def on_step_callback(self):
        if self._state == "IDLE":
            return

        now_time = self.get_clock().now()
        dist_m = float(self.get_parameter("test_distance_y_mm").get_value()) / 1000.0
        speed_m_s = float(self.get_parameter("drive_speed_m_s").get_value())
        duration_s = max(dist_m / max(speed_m_s, 0.001), 1.0)

        dt = (now_time - self._state_start_time).nanoseconds / 1e9
        curr_pos = self._get_ee_position_m()

        def smooth_s_curve(progress: float) -> float:
            s = min(max(progress, 0.0), 1.0)
            return 0.5 * (1.0 - math.cos(math.pi * s))

        if self._state == "CENTERING":
            dist_total = np.linalg.norm(self._start_position_robot - self._initial_position_robot)
            duration_centering = max(1.5, dist_total / max(speed_m_s, 0.01))
            progress_raw = min(dt / duration_centering, 1.0)
            p = smooth_s_curve(progress_raw)

            target_pos = self._initial_position_robot + p * (self._start_position_robot - self._initial_position_robot)
            target_quat = quaternion_slerp(self._initial_orientation_robot, self._start_orientation_robot, p)

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(target_quat)

            pause_s = float(self.get_parameter("phase_pause_s").get_value())

            if progress_raw >= 1.0 or dist_total < 0.002:
                self._state = "PAUSING_BEFORE_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info(f"Phase 1 Complete: Camera optical center arrived over board center. Settling {pause_s:.1f}s...")

        elif self._state == "PAUSING_BEFORE_FORWARD":
            pause_s = float(self.get_parameter("phase_pause_s").get_value())
            self._target_pose.set_position(self._start_position_robot)
            self._target_pose.set_orientation(self._start_orientation_robot)
            if dt >= pause_s:
                self._state = "MOVING_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info(f"Phase 2: Moving {dist_m*1000:.0f} mm forward along Conveyor Y-axis...")

        elif self._state == "MOVING_FORWARD":
            pause_s = float(self.get_parameter("phase_pause_s").get_value())
            progress_raw = min(dt / duration_s, 1.0)
            p = smooth_s_curve(progress_raw)
            offset_conveyor = np.array([0.0, p * dist_m, 0.0, 0.0])
            current_offset = (self._T_robot_conveyor @ offset_conveyor)[:3]
            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(self._start_orientation_robot)

            try:
                now_s = dt
                last_log = getattr(self, "_last_drive_log_time", 0.0)
                if now_s - last_log >= 0.2:
                    self._last_drive_log_time = now_s
                    T_inv = np.linalg.inv(self._T_robot_conveyor)
                    p_world = np.array([curr_pos[0], curr_pos[1], curr_pos[2], 1.0], dtype=np.float64)
                    p_conv = T_inv @ p_world
                    self.get_logger().info(
                        f"[Test Drive] Actual EE in Conveyor Frame: X={p_conv[0]*1000.0:.1f} mm, Y={p_conv[1]*1000.0:.1f} mm, Z={p_conv[2]*1000.0:.1f} mm"
                    )
            except Exception:
                pass

            if progress_raw >= 1.0:
                self._state = "PAUSING_AFTER_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info(f"Phase 2 Complete: Reached end of forward test drive. Pausing {pause_s:.1f}s...")

        elif self._state == "PAUSING_AFTER_FORWARD":
            pause_s = float(self.get_parameter("phase_pause_s").get_value())
            offset_conveyor = np.array([0.0, dist_m, 0.0, 0.0])
            current_offset = (self._T_robot_conveyor @ offset_conveyor)[:3]
            self._target_pose.set_position(self._start_position_robot + current_offset)
            self._target_pose.set_orientation(self._start_orientation_robot)
            if dt >= pause_s:
                self._state = "MOVING_BACKWARD"
                self._state_start_time = now_time
                self.get_logger().info("Phase 3: Moving back along Conveyor Y-axis...")

        elif self._state == "MOVING_BACKWARD":
            pause_s = float(self.get_parameter("phase_pause_s").get_value())
            progress_raw = min(dt / duration_s, 1.0)
            p = smooth_s_curve(progress_raw)
            offset_conveyor = np.array([0.0, (1.0 - p) * dist_m, 0.0, 0.0])
            current_offset = (self._T_robot_conveyor @ offset_conveyor)[:3]
            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(self._start_orientation_robot)

            if progress_raw >= 1.0:
                if bool(self.get_parameter("center_over_board").get_value()):
                    self._state = "PAUSING_AFTER_BACKWARD"
                    self._state_start_time = now_time
                    self.get_logger().info(f"Phase 3 Complete: Returned to board center position. Pausing {pause_s:.1f}s...")
                else:
                    self._state = "FINISHED"
                    self.set_predicate("is_running", False)
                    self.get_logger().info("Test drive completed successfully!")

        elif self._state == "PAUSING_AFTER_BACKWARD":
            pause_s = float(self.get_parameter("phase_pause_s").get_value())
            self._target_pose.set_position(self._start_position_robot)
            self._target_pose.set_orientation(self._start_orientation_robot)
            if dt >= pause_s:
                self._state = "RETURNING_INITIAL"
                self._state_start_time = now_time
                self.get_logger().info("Phase 4: Returning to original user start position...")

        elif self._state == "RETURNING_INITIAL":
            dist_total = np.linalg.norm(self._initial_position_robot - self._start_position_robot)
            duration_return = max(1.5, dist_total / max(speed_m_s, 0.01))
            progress_raw = min(dt / duration_return, 1.0)
            p = smooth_s_curve(progress_raw)

            target_pos = self._start_position_robot + p * (self._initial_position_robot - self._start_position_robot)
            target_quat = quaternion_slerp(self._start_orientation_robot, self._initial_orientation_robot, p)

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(target_quat)

            if progress_raw >= 1.0:
                self._state = "FINISHED"
                self.set_predicate("is_running", False)
                self.get_logger().info("Test drive completed successfully! Robot back at original user start position.")