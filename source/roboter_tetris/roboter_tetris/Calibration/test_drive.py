"""AICA Lifecycle Component: Calibration Test Drive for roboter_tetris.

Validates the extrinsic calibration by reading calibration.json (T_robot_conveyor),
and moving the robot end-effector back and forth along the Y-axis of the conveyor frame
when triggered via a Service / Event button in AICA Studio.
"""

import json
import os
from typing import Optional

import numpy as np
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
                "/home/tetripick/Desktop/AICA/roboter_tetris/source/roboter_tetris/roboter_tetris/Calibration/calibration.json",
                sr.ParameterType.STRING
            ),
            "Pfad zur calibration.json"
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
            "Positioniert den Greifer vor Start mittig über dem ChArUco-Board"
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
        calib_path = self.get_parameter("calibration_file_path").get_value()
        if not os.path.exists(calib_path):
            fallback_path = "/tmp/calibration.json"
            if os.path.exists(fallback_path):
                self.get_logger().info(
                    f"Configured calibration file {calib_path} not found. "
                    f"Using fallback: {fallback_path}"
                )
                calib_path = fallback_path
            else:
                self.get_logger().error(f"Calibration file not found at {calib_path} or {fallback_path}")
                return None

        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            if "transformations" in data and "T_robot_conveyor" in data["transformations"]:
                mat = data["transformations"]["T_robot_conveyor"]["homogeneous_matrix"]
                return np.array(mat, dtype=np.float64)
            elif "homogeneous_matrix" in data:
                return np.array(data["homogeneous_matrix"], dtype=np.float64)

            self.get_logger().error("No transformation matrix found in calibration.json")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to parse calibration.json: {e}")
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

        center_over_board = bool(self.get_parameter("center_over_board").get_value())
        if center_over_board:
            # Board center in conveyor frame: [-250 mm, 506 mm] (XY only)
            # We compute the delta to the board center using only the conveyor X and Y
            # basis vectors. Z is NEVER modified – this avoids errors from a poorly
            # calibrated Z component in T_robot_conveyor.
            T_inv = np.linalg.inv(T_robot_conveyor)
            curr_conv = T_inv @ np.array([curr_pos[0], curr_pos[1], curr_pos[2], 1.0], dtype=np.float64)

            # Delta in conveyor XY space to reach board center
            delta_x_conv = -0.250 - curr_conv[0]
            delta_y_conv = 0.506 - curr_conv[1]

            # Apply delta in world frame using conveyor basis vectors (no Z change)
            X_conv_unit = T_robot_conveyor[:3, 0]
            Y_conv_unit = T_robot_conveyor[:3, 1]
            self._start_position_robot = curr_pos + delta_x_conv * X_conv_unit + delta_y_conv * Y_conv_unit
            self.get_logger().info(
                f"Centering EE over board center: delta_X={delta_x_conv*1000:.1f} mm, "
                f"delta_Y={delta_y_conv*1000:.1f} mm in conveyor_frame (Z unchanged)."
            )
        else:
            self._start_position_robot = curr_pos

        self._state = "MOVING_FORWARD"
        self._state_start_time = self.get_clock().now()
        self.set_predicate("is_running", True)
        self.set_predicate("has_failed", False)

        dist_m = float(self.get_parameter("test_distance_y_mm").get_value()) / 1000.0
        self.get_logger().info(
            f"Starting test drive: moving {dist_m*1000:.0f} mm "
            "forward along Conveyor Y-axis (Green line)..."
        )
        return True

    def on_step_callback(self):
        if self._state == "IDLE":
            return

        now_time = self.get_clock().now()
        dist_m = (
            float(self.get_parameter("test_distance_y_mm").get_value())
            / 1000.0
        )
        speed_m_s = float(
            self.get_parameter("drive_speed_m_s").get_value()
        )
        duration_s = dist_m / max(speed_m_s, 0.001)

        dt = (
            now_time - self._state_start_time
        ).nanoseconds / 1e9

        if self._state == "MOVING_FORWARD":
            progress = min(dt / duration_s, 1.0)

            # +Y im conveyor_frame -> robot_base/world
            offset_conveyor = np.array([0.0, progress * dist_m, 0.0, 0.0])

            current_offset = (
                self._T_robot_conveyor @ offset_conveyor
            )[:3]

            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(
                self._robot_ee_pose.get_orientation()
            )

            # Log live position in conveyor_frame coordinates
            try:
                T_inv = np.linalg.inv(self._T_robot_conveyor)
                p_world = np.array([target_pos[0], target_pos[1], target_pos[2], 1.0], dtype=np.float64)
                p_conv = T_inv @ p_world
                self.get_logger().info(
                    f"[Test Drive] EE in Conveyor Frame: X={p_conv[0]*1000.0:.1f} mm, Y={p_conv[1]*1000.0:.1f} mm, Z={p_conv[2]*1000.0:.1f} mm"
                )
            except Exception:
                pass

            if progress >= 1.0:
                self._state = "PAUSING"
                self._state_start_time = now_time
                self.get_logger().info(
                    "Reached end of forward test drive. Pausing 1 second..."
                )

        elif self._state == "PAUSING":
            if dt >= 1.0:
                self._state = "MOVING_BACKWARD"
                self._state_start_time = now_time
                self.get_logger().info(
                    "Moving back to starting position..."
                )

        elif self._state == "MOVING_BACKWARD":
            progress = min(dt / duration_s, 1.0)

            # Zurück zum Ausgangspunkt
            offset_conveyor = np.array([0.0, (1.0 - progress) * dist_m, 0.0, 0.0])

            current_offset = (
                self._T_robot_conveyor @ offset_conveyor
            )[:3]

            target_pos = self._start_position_robot + current_offset

            self._target_pose.set_position(target_pos)
            self._target_pose.set_orientation(
                self._robot_ee_pose.get_orientation()
            )

            if progress >= 1.0:
                self._state = "FINISHED"
                self.set_predicate("is_running", False)
                self.get_logger().info(
                    "Test drive completed successfully! "
                    "Robot back at start position."
                )