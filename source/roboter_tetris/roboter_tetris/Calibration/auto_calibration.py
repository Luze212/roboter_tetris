"""AICA Lifecycle Component: Automatic Extrinsic Calibration for roboter_tetris."""

import json
import math
from typing import List

import numpy as np
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from modulo_interfaces.srv import StringTrigger

from .extrinsic_calibration import (
    CalibrationResult, CalibrationSample,
    pose_to_matrix, rotation_matrix_to_quaternion,
    rpy_to_rotation_matrix, save_calibration_json, solve_eye_in_hand,
)

STALE_TIMEOUT_S = 1.0


class AutoCalibration(LifecycleComponent):
    """AICA component for automated extrinsic camera-robot calibration."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Parameters
        self.add_parameter(
            sr.Parameter(
                "calibration_file_path",
                "/tmp/calibration.json",
                sr.ParameterType.STRING
            ),
            "Zielpfad für die generierte calibration.json (Standard: /tmp/calibration.json)"
        )
        self.add_parameter(
            sr.Parameter("settle_time_s", 0.8, sr.ParameterType.DOUBLE),
            "Wartezeit (Sekunden) nach Anfahren einer Pose vor der Bildaufnahme"
        )
        self.add_parameter(
            sr.Parameter("samples_per_waypoint", 5, sr.ParameterType.INT),
            "Anzahl zu mittelnder Detektionen pro Wegpunkt"
        )
        self.add_parameter(
            sr.Parameter("num_waypoints", 9, sr.ParameterType.INT),
            "Anzahl der Kalibrier-Wegpunkte (Standard: 9, min: 3)"
        )
        self.add_parameter(
            sr.Parameter("max_translation_offset_m", 0.05, sr.ParameterType.DOUBLE),
            "Maximaler kartesischer Versatz (in m) um die Startposition"
        )
        self.add_parameter(
            sr.Parameter("max_rotation_angle_deg", 8.0, sr.ParameterType.DOUBLE),
            "Maximaler Kippwinkel (in Grad) für die Wegpunkte"
        )
        self.add_parameter(
            sr.Parameter("align_to_board", True, sr.ParameterType.BOOL),
            "Automatisch parallel zum ChArUco-Board ausrichten, falls vorab erkannt"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_x_mm", -375.0, sr.ParameterType.DOUBLE),
            "X-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_y_mm", 416.0, sr.ParameterType.DOUBLE),
            "Y-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm (Förderband-Laufrichtung)"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_z_mm", 0.0, sr.ParameterType.DOUBLE),
            "Z-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm (Höhe Förderband)"
        )

        # Inputs
        self._base_cam_board_pose_msg = []
        self.add_input(
            "base_cam_board_pose",
            "_base_cam_board_pose_msg",
            Float64MultiArray
        )

        self._robot_cam_board_pose_msg = []
        self.add_input(
            "robot_cam_board_pose",
            "_robot_cam_board_pose_msg",
            Float64MultiArray
        )

        self._robot_ee_pose = sr.CartesianState("end_effector", "world")
        self.add_input(
            "robot_ee_pose",
            "_robot_ee_pose",
            EncodedState
        )

        # Target EE pose
        self._target_pose = sr.CartesianPose(
            "calibration_target", "world"
        )
        self.add_output(
            "target_ee_pose",
            "_target_pose",
            EncodedState,
            MessageType.CARTESIAN_POSE_MESSAGE
        )

        # Outputs
        self._calibration_matrix = []
        self.add_output(
            "calibration_matrix",
            "_calibration_matrix",
            Float64MultiArray
        )

        self._calibration_rpy = []
        self.add_output(
            "calibration_rpy",
            "_calibration_rpy",
            Float64MultiArray
        )

        # Predicates
        self.add_predicate("is_running", False)
        self.add_predicate("is_calibrated", False)
        self.add_predicate("has_failed", False)

        # Service
        try:
            self.add_service(
                "start_calibration",
                StringTrigger,
                self._on_start_calibration_service
            )
        except Exception:
            pass

        # State machine
        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._start_ee_transform = None
        self._collected_samples: List[CalibrationSample] = []
        self._sample_buffer_robot_cam: List[List[float]] = []
        self._sample_buffer_base_cam: List[List[float]] = []

        self._waypoint_labels = []
        self._waypoint_offsets = []

    def _generate_waypoints(self):
        """Generiert N dynamische Wegpunkte mit kombinierten Translations- und Rotations-Offsets."""
        num_wp = max(3, int(self.get_parameter("num_waypoints").get_value()))
        max_trans = float(self.get_parameter("max_translation_offset_m").get_value())
        max_rot_deg = float(self.get_parameter("max_rotation_angle_deg").get_value())
        max_rot_rad = math.radians(max_rot_deg)

        base_pattern = [
            ("center", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ("tilt_pitch_p", max_trans, 0.0, 0.0, 0.0, max_rot_rad, 0.0),
            ("tilt_pitch_m", -max_trans, 0.0, 0.0, 0.0, -max_rot_rad, 0.0),
            ("tilt_roll_p", 0.0, max_trans, 0.02, max_rot_rad, 0.0, 0.0),
            ("tilt_roll_m", 0.0, -max_trans, -0.02, -max_rot_rad, 0.0, 0.0),
            ("high_yaw_p", 0.0, 0.0, max_trans * 0.8, 0.0, 0.0, max_rot_rad),
            ("low_yaw_m", 0.0, 0.0, -max_trans * 0.8, 0.0, 0.0, -max_rot_rad),
            ("corner_pp", max_trans * 0.7, max_trans * 0.7, 0.01, max_rot_rad * 0.7, max_rot_rad * 0.7, 0.0),
            ("corner_mm", -max_trans * 0.7, -max_trans * 0.7, -0.01, -max_rot_rad * 0.7, -max_rot_rad * 0.7, 0.0),
            ("corner_pm", max_trans * 0.7, -max_trans * 0.7, 0.01, -max_rot_rad * 0.7, max_rot_rad * 0.7, 0.0),
            ("corner_mp", -max_trans * 0.7, max_trans * 0.7, -0.01, max_rot_rad * 0.7, -max_rot_rad * 0.7, 0.0),
            ("center_high", 0.0, 0.0, max_trans * 1.2, 0.0, 0.0, 0.0),
        ]

        self._waypoint_labels = []
        self._waypoint_offsets = []

        for i in range(num_wp):
            if i < len(base_pattern):
                label, dx, dy, dz, dr, dp, dyaw = base_pattern[i]
            else:
                factor = ((i % 5) + 1) / 5.0
                sign = 1.0 if (i % 2 == 0) else -1.0
                label = f"extra_{i+1}"
                dx = sign * max_trans * factor * 0.8
                dy = -sign * max_trans * factor * 0.6
                dz = (factor - 0.5) * max_trans * 0.5
                dr = sign * max_rot_rad * factor * 0.5
                dp = -sign * max_rot_rad * factor * 0.5
                dyaw = sign * max_rot_rad * factor * 0.3

            self._waypoint_labels.append(label)
            self._waypoint_offsets.append((dx, dy, dz, dr, dp, dyaw))

        self._waypoints = self._waypoint_labels

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._collected_samples.clear()

        self.set_predicate("is_running", False)
        self.set_predicate("is_calibrated", False)
        self.set_predicate("has_failed", False)

        # Trigger calibration sequence upon activation (via button / trigger event)
        self._on_start_calibration()

        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _get_current_ee_transform(self) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        try:
            pos = self._robot_ee_pose.get_position()
            T[0, 3] = float(pos[0])
            T[1, 3] = float(pos[1])
            T[2, 3] = float(pos[2])

            ori = self._robot_ee_pose.get_orientation()
            if hasattr(ori, "to_rotation_matrix"):
                T[:3, :3] = np.array(ori.to_rotation_matrix(), dtype=np.float64)
            else:
                qx = float(ori.x)
                qy = float(ori.y)
                qz = float(ori.z)
                qw = float(ori.w)
                T[:3, :3] = np.array([
                    [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                    [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                    [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
                ], dtype=np.float64)

        except Exception as e:
            self.get_logger().warn(f"Could not extract current robot_ee_pose: {e}")
        return T

    def _on_start_calibration_service(
        self,
        request: StringTrigger.Request
    ) -> StringTrigger.Response:
        response = StringTrigger.Response()

        if self._state not in ("IDLE", "FINISHED", "FAILED"):
            response.success = False
            response.message = f"Kalibrierung läuft bereits (Zustand: {self._state})."
            return response

        self._on_start_calibration()

        response.success = True
        response.message = "Kalibriersequenz gestartet."
        return response

    def _on_start_calibration(self):
        if self._state in ("IDLE", "FINISHED", "FAILED"):
            self._generate_waypoints()
            self.get_logger().info(
                f"Starting automatic calibration sequence with {len(self._waypoint_labels)} waypoints..."
            )

            self._state = "MOVING"
            self._current_waypoint_idx = 0
            self._collected_samples.clear()

            self.set_predicate("is_running", True)
            self.set_predicate("is_calibrated", False)
            self.set_predicate("has_failed", False)

            self._start_ee_transform = self._get_current_ee_transform()
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        idx = self._current_waypoint_idx
        label = self._waypoint_labels[idx]
        dx, dy, dz, dr, dp, dyaw = self._waypoint_offsets[idx]

        self.get_logger().info(
            f"Moving to waypoint {idx + 1}/{len(self._waypoint_labels)}: {label} "
            f"(offset trans=[{dx:.3f}, {dy:.3f}, {dz:.3f}] m, rot_rpy=[{math.degrees(dr):.1f}°, {math.degrees(dp):.1f}°, {math.degrees(dyaw):.1f}°])"
        )

        if self._start_ee_transform is None:
            self.get_logger().warn("No start EE transform available — cannot set target pose.")
            return

        T_target = self._start_ee_transform.copy()
        T_target[0, 3] += dx
        T_target[1, 3] += dy
        T_target[2, 3] += dz

        # Calculate rotation tilt: R_target = R_start @ R_tilt
        R_tilt = rpy_to_rotation_matrix(dr, dp, dyaw)
        R_target = T_target[:3, :3] @ R_tilt
        T_target[:3, :3] = R_target

        try:
            pos = T_target[:3, 3].tolist()
            self._target_pose.set_position(np.array(pos, dtype=np.float64))

            # Safely copy orientation directly from robot EE pose (in frame world)
            current_ori = self._robot_ee_pose.get_orientation()
            self._target_pose.set_orientation(current_ori)

            self.get_logger().info(
                f"Target pose set successfully: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not set target EE pose: {e}")

        # Robot moves automatically via Signal Point Attractor → IK Velocity Controller


    def on_step_callback(self):
        now_time = self.get_clock().now()

        if self._state == "IDLE":
            return

        if self._state == "MOVING":
            if self._state_start_time is None:
                self._state_start_time = now_time

            dt = (
                now_time - self._state_start_time
            ).nanoseconds / 1e9

            if dt >= 3.0:
                self._state = "SETTLING"
                self._state_start_time = now_time

        elif self._state == "SETTLING":
            settle_target = float(
                self.get_parameter("settle_time_s").get_value()
            )

            dt = (
                now_time - self._state_start_time
            ).nanoseconds / 1e9

            if dt >= settle_target:
                self._state = "SAMPLING"
                self._sample_buffer_robot_cam.clear()
                self._sample_buffer_base_cam.clear()
                self._state_start_time = now_time

        elif self._state == "SAMPLING":
            target_samples = int(
                self.get_parameter(
                    "samples_per_waypoint"
                ).get_value()
            )

            robot_cam_data = (
                list(self._robot_cam_board_pose_msg.data)
                if hasattr(self._robot_cam_board_pose_msg, "data")
                else list(self._robot_cam_board_pose_msg)
            )

            base_cam_data = (
                list(self._base_cam_board_pose_msg.data)
                if hasattr(self._base_cam_board_pose_msg, "data")
                else list(self._base_cam_board_pose_msg)
            )

            if len(robot_cam_data) >= 6:
                self._sample_buffer_robot_cam.append(
                    robot_cam_data
                )

            if len(base_cam_data) >= 6:
                self._sample_buffer_base_cam.append(
                    base_cam_data
                )

            if len(self._sample_buffer_robot_cam) >= target_samples:
                avg_robot_cam = np.mean(
                    self._sample_buffer_robot_cam,
                    axis=0
                ).tolist()

                avg_base_cam = (
                    np.mean(
                        self._sample_buffer_base_cam,
                        axis=0
                    ).tolist()
                    if self._sample_buffer_base_cam
                    else None
                )

                T_ee = self._get_current_ee_transform()

                sample = CalibrationSample(
                    T_robot_ee=T_ee,
                    robot_cam_board_pose=avg_robot_cam,
                    base_cam_board_pose=avg_base_cam
                )

                self._collected_samples.append(sample)

                self.get_logger().info(
                    f"Sampled Waypoint "
                    f"{self._current_waypoint_idx + 1}/"
                    f"{len(self._waypoints)}"
                )

                self._current_waypoint_idx += 1

                if self._current_waypoint_idx < len(self._waypoints):
                    self._state = "MOVING"
                    self._state_start_time = None
                    self._send_next_waypoint()
                else:
                    self.get_logger().info("Alle Wegpunkte abgetastet. Kehre zurück zur Startposition...")
                    self._state = "RETURNING_HOME"
                    self._state_start_time = None
                    self._send_home_waypoint()
            else:
                # Log warning if sampling takes too long without receiving board pose
                dt_sample = (now_time - self._state_start_time).nanoseconds / 1e9
                if dt_sample > 2.0 and int(dt_sample) % 2 == 0:
                    self.get_logger().warning(
                        f"Wegpunkt {self._current_waypoint_idx + 1}/{len(self._waypoints)}: "
                        f"Warte auf ChArUco-Erkennung der RobotCam ({len(self._sample_buffer_robot_cam)}/{target_samples} Samples)..."
                    )

        elif self._state == "RETURNING_HOME":
            if self._state_start_time is None:
                self._state_start_time = now_time

            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            if dt >= 3.0:
                self._state = "SOLVING"

        elif self._state == "SOLVING":
            self.get_logger().info(
                f"Computing Extrinsic Calibration from "
                f"{len(self._collected_samples)} samples..."
            )

            try:
                off_x_m = float(self.get_parameter("conveyor_offset_x_mm").get_value()) / 1000.0
                off_y_m = float(self.get_parameter("conveyor_offset_y_mm").get_value()) / 1000.0
                off_z_m = float(self.get_parameter("conveyor_offset_z_mm").get_value()) / 1000.0

                result: CalibrationResult = solve_eye_in_hand(
                    self._collected_samples,
                    conveyor_offset_m=(off_x_m, off_y_m, off_z_m)
                )

                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                save_path = self.get_parameter(
                    "calibration_file_path"
                ).get_value()

                save_calibration_json(save_path, result)

                self.get_logger().info(
                    f"Calibration successful! "
                    f"RMSE: {result.position_rmse_mm:.2f} mm. "
                    f"Saved to {save_path}"
                )

                self.set_predicate("is_calibrated", True)
                self.set_predicate("is_running", False)
                self._state = "FINISHED"

            except Exception as e:
                self.get_logger().error(
                    f"Calibration failed: {e}"
                )
                self.set_predicate("has_failed", True)
                self.set_predicate("is_running", False)
                self._state = "FAILED"

        elif self._state in ("FINISHED", "FAILED"):
            pass

    def _send_home_waypoint(self):
        """Send EE target back to the initial start position."""
        if self._start_ee_transform is not None:
            try:
                pos = self._start_ee_transform[:3, 3].tolist()
                self._target_pose.set_position(np.array(pos, dtype=np.float64))
                current_ori = self._robot_ee_pose.get_orientation()
                self._target_pose.set_orientation(current_ori)
                self.get_logger().info(
                    f"Target pose reset to start position: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                )
            except Exception as e:
                self.get_logger().warn(f"Could not reset target EE pose to home: {e}")