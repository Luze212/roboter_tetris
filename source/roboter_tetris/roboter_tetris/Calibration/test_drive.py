"""AICA Lifecycle Component: Calibration Test Drive for roboter_tetris.

Validates the extrinsic calibration by reading calibration.json (T_robot_conveyor),
and moving the robot end-effector back and forth along the Y-axis of the conveyor frame
when triggered via a Service / Event button in AICA Studio.
"""

import json
import os
from typing import Optional

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
            "Zentriert den Greifer vor Start über dem berechneten ChArUco-Board-Zentrum (Bediener-Kontrolle der Kalibriergenauigkeit)"
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

    def _load_conveyor_transform(self) -> Optional[np.ndarray]:
        """Load T_robot_conveyor from calibration YAML/JSON. Returns 4x4 matrix or None."""
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
                return None

        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            if isinstance(data, dict):
                # Store full data for board center lookup
                self._calib_data = data
                if "transformations" in data and "T_robot_conveyor" in data["transformations"]:
                    mat = data["transformations"]["T_robot_conveyor"]["homogeneous_matrix"]
                    return np.array(mat, dtype=np.float64)
                elif "homogeneous_matrix" in data:
                    return np.array(data["homogeneous_matrix"], dtype=np.float64)

            self.get_logger().error(f"No valid transformation matrix found in {calib_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to parse calibration file {calib_path}: {e}")
            return None

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
                 "(Fehler in calibration.json)."
        )
        return response

    def _on_start_test_drive(self) -> bool:
        T_robot_conveyor = self._load_conveyor_transform()

        if T_robot_conveyor is None:
            self.set_predicate("has_failed", True)
            return False

        self._T_robot_conveyor = T_robot_conveyor

        # Log Conveyor Frame unit vectors in robot_base (world)
        X_conv = T_robot_conveyor[:3, 0]
        Y_conv = T_robot_conveyor[:3, 1]
        Z_conv = T_robot_conveyor[:3, 2]
        self.get_logger().info("=== Conveyor Frame Axes in robot_base (world) ===")
        self.get_logger().info(f"  X_conv (Width) : [{X_conv[0]:.4f}, {X_conv[1]:.4f}, {X_conv[2]:.4f}]")
        self.get_logger().info(f"  Y_conv (Flow)  : [{Y_conv[0]:.4f}, {Y_conv[1]:.4f}, {Y_conv[2]:.4f}]")
        self.get_logger().info(f"  Z_conv (Height): [{Z_conv[0]:.4f}, {Z_conv[1]:.4f}, {Z_conv[2]:.4f}]")

        pos = self._robot_ee_pose.get_position()
        curr_pos = np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=np.float64)
        self._initial_position_robot = curr_pos

        center_over_board = bool(self.get_parameter("center_over_board").get_value())
        if center_over_board:
            board_center = self._load_board_center_conveyor()
            if board_center is None:
                self.get_logger().warn(
                    "center_over_board=True but no board_center_conveyor_mm in calibration file. "
                    "Skipping centering; starting from current position. "
                    "Re-run AutoCalibration to generate board center data."
                )
                self._start_position_robot = curr_pos
                self._state = "MOVING_FORWARD"
            else:
                bc_x_m, bc_y_m, _ = board_center
                T_inv = np.linalg.inv(T_robot_conveyor)
                curr_conv = T_inv @ np.array([curr_pos[0], curr_pos[1], curr_pos[2], 1.0], dtype=np.float64)

                delta_x_conv = bc_x_m - curr_conv[0]
                delta_y_conv = bc_y_m - curr_conv[1]

                self.get_logger().info(
                    f"Board center target (conveyor_frame): X={bc_x_m*1000:.1f} mm, Y={bc_y_m*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Current EE pos (conveyor_frame): X={curr_conv[0]*1000:.1f} mm, Y={curr_conv[1]*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Phase 0 (Centering): delta_X={delta_x_conv*1000:.1f} mm, delta_Y={delta_y_conv*1000:.1f} mm "
                    f"(should be ~0 mm if calibration is correct)"
                )

                # Only allow centering if delta is reasonable (< 0.5 m)
                if abs(delta_x_conv) < 0.5 and abs(delta_y_conv) < 0.5:
                    X_conv_unit = T_robot_conveyor[:3, 0]
                    Y_conv_unit = T_robot_conveyor[:3, 1]
                    self._start_position_robot = curr_pos + delta_x_conv * X_conv_unit + delta_y_conv * Y_conv_unit
                    self._state = "CENTERING"
                    self._target_pose.set_position(self._start_position_robot)
                    self._target_pose.set_orientation(self._robot_ee_pose.get_orientation())
                else:
                    self.get_logger().warn(
                        f"Centering delta too large (delta_X={delta_x_conv*1000:.0f} mm, "
                        f"delta_Y={delta_y_conv*1000:.0f} mm). "
                        "Skipping centering – calibration error or robot not positioned near board."
                    )
                    self._start_position_robot = curr_pos
                    self._state = "MOVING_FORWARD"
        else:
            self._start_position_robot = curr_pos
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
        curr_pos = np.array(self._robot_ee_pose.get_position(), dtype=np.float64)

        if self._state == "CENTERING":
            dist_to_center = np.linalg.norm(curr_pos - self._start_position_robot)
            if dist_to_center < 0.002 or dt >= 4.0:
                self._state = "PAUSING_BEFORE_FORWARD"
                self._state_start_time = now_time
                self.get_logger().info("Phase 1 Complete: Arrived over board center. Settling 0.8s...")

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

            # Log live ACTUAL position in conveyor_frame coordinates
            try:
                T_inv = np.linalg.inv(self._T_robot_conveyor)
                p_world = np.array([curr_pos[0], curr_pos[1], curr_pos[2], 1.0], dtype=np.float64)
                p_conv = T_inv @ p_world
                self.get_logger().info(
                    f"[Test Drive] Actual EE in Conveyor Frame: X={p_conv[0]*1000.0:.1f} mm, Y={p_conv[1]*1000.0:.1f} mm, Z={p_conv[2]*1000.0:.1f} mm"
                )
            except Exception:
                pass

            if progress >= 1.0 and np.linalg.norm(curr_pos - target_pos) < 0.003:
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

            if progress >= 1.0 and np.linalg.norm(curr_pos - target_pos) < 0.003:
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