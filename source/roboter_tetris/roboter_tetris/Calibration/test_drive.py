"""AICA Lifecycle Component: Calibration Test Drive for roboter_tetris.

Validates the extrinsic calibration by reading calibration.yaml/json (T_robot_conveyor & T_ee_robot_cam),
and moving the robot camera/end-effector back and forth along the Y-axis of the conveyor frame
when triggered via a Service / Event button in AICA Studio.
"""

import json
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


class CalibrationTestDrive(LifecycleComponent):
    """AICA component for executing a test drive along the conveyor Y-axis to validate calibration."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Parameters
        self.add_parameter(
            sr.Parameter(
                "calibration_file_path",
                "/tmp/calibration.yaml",
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

        # Inputs
        self._robot_ee_pose = sr.CartesianState("end_effector", "world")
        self.add_input(
            "robot_ee_pose",
            "_robot_ee_pose",
            EncodedState
        )

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

    def _load_calibration_data(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Load T_robot_conveyor and T_ee_robot_cam from calibration YAML/JSON.
        
        Returns (T_robot_conveyor, T_ee_robot_cam).
        """
        calib_path = self.get_parameter("calibration_file_path").get_value()
        if not os.path.exists(calib_path):
            candidate_fallbacks = ["/tmp/calibration.yaml", "/tmp/calibration.json"]
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

                # 2. T_ee_robot_cam laden (Eye-in-Hand Kamera zu Endeffektor)
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

            return T_robot_conveyor, T_ee_robot_cam

        except Exception as e:
            self.get_logger().error(f"Failed to parse calibration file {calib_path}: {e}")
            return None, None

    def _load_board_center_conveyor(self) -> Optional[tuple]:
        """Read board_center_conveyor_mm from cached calibration data. Returns (x_m, y_m, z_m) or None."""
        data = getattr(self, "_calib_data", None)
        if data is None:
            return None
        try:
            bc = data.get("board_center_conveyor_mm", {})
            if bc and bc.get("x") is not None and bc.get("y") is not None:
                x_m = float(bc["x"]) / 1000.0
                y_m = float(bc["y"]) / 1000.0
                z_m = float(bc.get("z", 0.0)) / 1000.0
                self.get_logger().info(
                    f"Board center from calibration: X={x_m*1000:.1f} mm, Y={y_m*1000:.1f} mm "
                    f"(in conveyor_frame)"
                )
                return (x_m, y_m, z_m)
        except Exception:
            pass
        self.get_logger().warn(
            "board_center_conveyor_mm not found in calibration file. "
            "Run AutoCalibration first to generate this value."
        )
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
            else "Testfahrt konnte nicht gestartet werden "
                 "(Fehler in calibration.yaml/json)."
        )
        return response

    def _on_start_test_drive(self) -> bool:
        T_robot_conveyor, T_ee_robot_cam = self._load_calibration_data()

        if T_robot_conveyor is None:
            self.set_predicate("has_failed", True)
            return False

        self._T_robot_conveyor = T_robot_conveyor
        self._T_ee_robot_cam = T_ee_robot_cam

        # Log Conveyor Frame unit vectors in robot_base (world)
        X_conv = T_robot_conveyor[:3, 0]
        Y_conv = T_robot_conveyor[:3, 1]
        Z_conv = T_robot_conveyor[:3, 2]
        self.get_logger().info("=== Conveyor Frame Axes in robot_base (world) ===")
        self.get_logger().info(f"  X_conv (Width) : [{X_conv[0]:.4f}, {X_conv[1]:.4f}, {X_conv[2]:.4f}]")
        self.get_logger().info(f"  Y_conv (Flow)  : [{Y_conv[0]:.4f}, {Y_conv[1]:.4f}, {Y_conv[2]:.4f}]")
        self.get_logger().info(f"  Z_conv (Height): [{Z_conv[0]:.4f}, {Z_conv[1]:.4f}, {Z_conv[2]:.4f}]")

        curr_ee_pos = self._get_ee_position_m()
        self._initial_position_robot = curr_ee_pos

        center_over_board = bool(self.get_parameter("center_over_board").get_value())
        if center_over_board:
            board_center = self._load_board_center_conveyor()
            if board_center is None:
                self.get_logger().warn(
                    "center_over_board=True but no board_center_conveyor_mm in calibration file. "
                    "Skipping centering; starting from current position."
                )
                self._start_position_robot = curr_ee_pos
                self._state = "MOVING_FORWARD"
            else:
                bc_x_m, bc_y_m, _ = board_center
                
                # Berechne die aktuelle Kamera-Position im conveyor_frame
                R_ee = np.eye(3, dtype=np.float64)
                ori = self._robot_ee_pose.get_orientation()
                if hasattr(ori, "to_rotation_matrix"):
                    R_ee = np.array(ori.to_rotation_matrix(), dtype=np.float64)
                else:
                    qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
                    R_ee = np.array([
                        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
                    ], dtype=np.float64)

                t_ee_cam = T_ee_robot_cam[:3, 3]
                curr_cam_world = curr_ee_pos + R_ee @ t_ee_cam

                T_inv = np.linalg.inv(T_robot_conveyor)
                curr_cam_conv = T_inv @ np.array([curr_cam_world[0], curr_cam_world[1], curr_cam_world[2], 1.0], dtype=np.float64)

                delta_x_conv = bc_x_m - curr_cam_conv[0]
                delta_y_conv = bc_y_m - curr_cam_conv[1]

                self.get_logger().info(
                    f"Board center target (conveyor_frame): X={bc_x_m*1000:.1f} mm, Y={bc_y_m*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Current Camera pos (conveyor_frame): X={curr_cam_conv[0]*1000:.1f} mm, Y={curr_cam_conv[1]*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Phase 0 (Camera Centering): delta_X={delta_x_conv*1000:.1f} mm, delta_Y={delta_y_conv*1000:.1f} mm"
                )

                # Toleranz für Bewegung erhöhen (bis zu 0.5m Versatz zentrieren)
                if abs(delta_x_conv) < 0.5 and abs(delta_y_conv) < 0.5:
                    X_conv_unit = T_robot_conveyor[:3, 0]
                    Y_conv_unit = T_robot_conveyor[:3, 1]
                    
                    self._start_position_robot = curr_ee_pos + delta_x_conv * X_conv_unit + delta_y_conv * Y_conv_unit
                    self._state = "CENTERING"
                    self._target_pose.set_position(self._start_position_robot)
                    self._target_pose.set_orientation(self._robot_ee_pose.get_orientation())
                else:
                    self.get_logger().warn(
                        f"Centering delta too large (delta_X={delta_x_conv*1000:.0f} mm, "
                        f"delta_Y={delta_y_conv*1000:.0f} mm). Skipping centering."
                    )
                    self._start_position_robot = curr_ee_pos
                    self._state = "MOVING_FORWARD"
        else:
            self._start_position_robot = curr_ee_pos
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
        duration_s = dist_m / max(speed_m_s, 0.001)

        dt = (now_time - self._state_start_time).nanoseconds / 1e9
        curr_pos = self._get_ee_position_m()

        if self._state == "CENTERING":
            dist_to_center = np.linalg.norm(curr_pos - self._start_position_robot)
            if dist_to_center < 0.002 or dt >= 4.0:
                self._state = "PAUSING_BEFORE_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info("Phase 1 Complete: Camera optical center arrived over board center. Settling 0.8s...")

        elif self._state == "PAUSING_BEFORE_FORWARD":
            if dt >= 0.8:
                self._state = "MOVING_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info(f"Phase 2: Moving {dist_m*1000:.0f} mm forward along Conveyor Y-axis...")

        elif self._state == "MOVING_FORWARD":
            progress = min(dt / duration_s, 1.0)
            offset_conveyor = np.array([0.0, progress * dist_m, 0.0, 0.0])
            current_offset = (self._T_robot_conveyor @ offset_conveyor)[:3]
            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(self._robot_ee_pose.get_orientation())

            try:
                T_inv = np.linalg.inv(self._T_robot_conveyor)
                p_world = np.array([curr_pos[0], curr_pos[1], curr_pos[2], 1.0], dtype=np.float64)
                p_conv = T_inv @ p_world
                self.get_logger().info(
                    f"[Test Drive] Actual EE in Conveyor Frame: X={p_conv[0]*1000.0:.1f} mm, Y={p_conv[1]*1000.0:.1f} mm, Z={p_conv[2]*1000.0:.1f} mm"
                )
            except Exception:
                pass

            if progress >= 1.0 and (np.linalg.norm(curr_pos - target_pos) < 0.010 or dt >= duration_s + 1.0):
                self._state = "PAUSING"
                self._state_start_time = now_time
                self.get_logger().info("Phase 2 Complete: Reached end of forward test drive. Pausing 1.0s...")

        elif self._state == "PAUSING":
            if dt >= 1.0:
                self._state = "MOVING_BACKWARD"
                self._state_start_time = now_time
                self.get_logger().info("Phase 3: Moving back along Conveyor Y-axis...")

        elif self._state == "MOVING_BACKWARD":
            progress = min(dt / duration_s, 1.0)
            offset_conveyor = np.array([0.0, (1.0 - progress) * dist_m, 0.0, 0.0])
            current_offset = (self._T_robot_conveyor @ offset_conveyor)[:3]
            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(self._robot_ee_pose.get_orientation())

            if progress >= 1.0 and (np.linalg.norm(curr_pos - target_pos) < 0.010 or dt >= duration_s + 1.0):
                if bool(self.get_parameter("center_over_board").get_value()):
                    self._state = "RETURNING_INITIAL"
                    self._state_start_time = now_time
                    self._target_pose.set_position(self._initial_position_robot)
                    self.get_logger().info("Phase 4: Returning to original user start position...")
                else:
                    self._state = "FINISHED"
                    self.set_predicate("is_running", False)
                    self.get_logger().info("Test drive completed successfully!")

        elif self._state == "RETURNING_INITIAL":
            dist_to_init = np.linalg.norm(curr_pos - self._initial_position_robot)
            if dist_to_init < 0.003 or dt >= 4.0:
                self._state = "FINISHED"
                self.set_predicate("is_running", False)
                self.get_logger().info("Test drive completed successfully! Robot back at original user start position.")