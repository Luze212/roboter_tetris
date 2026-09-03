"""AICA Lifecycle Component: Automatic Extrinsic Orbit Calibration for roboter_tetris."""

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
    rpy_to_rotation_matrix, save_calibration_json, save_calibration_yaml, solve_eye_in_hand,
)

STALE_TIMEOUT_S = 1.0


class AutoCalibration(LifecycleComponent):
    """AICA component for automated extrinsic camera-robot orbit calibration."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Parameters
        self.add_parameter(
            sr.Parameter(
                "calibration_file_path",
                "/tmp/calibration.yaml",
                sr.ParameterType.STRING
            ),
            "Zielpfad für die generierte calibration.yaml (Standard: /tmp/calibration.yaml)"
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
            "Anzahl der Kalibrier-Wegpunkte (Standard: 9)"
        )
        self.add_parameter(
            sr.Parameter("max_orbit_radius_mm", 50.0, sr.ParameterType.DOUBLE),
            "Maximaler kartesischer Orbit-Radius (in mm) um das Board-Zentrum"
        )
        self.add_parameter(
            sr.Parameter("max_rotation_angle_deg", 8.0, sr.ParameterType.DOUBLE),
            "Maximaler Neigungswinkel (in Grad) während der Orbit-Schwenks"
        )
        self.add_parameter(
            sr.Parameter("board_distance_mm", 350.0, sr.ParameterType.DOUBLE),
            "Geschätzter Abstand vom Greifer/Kamera zum ChArUco-Board in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_x_mm", -375.0, sr.ParameterType.DOUBLE),
            "X-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_y_mm", 416.0, sr.ParameterType.DOUBLE),
            "Y-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_z_mm", 0.0, sr.ParameterType.DOUBLE),
            "Z-Offset des ChArUco Board-Ursprungs im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("board_rows", 5, sr.ParameterType.INT),
            "Anzahl der Zeilen des ChArUco-Boards (Checker, nicht Marker)"
        )
        self.add_parameter(
            sr.Parameter("board_cols", 7, sr.ParameterType.INT),
            "Anzahl der Spalten des ChArUco-Boards (Checker, nicht Marker)"
        )
        self.add_parameter(
            sr.Parameter("square_size_mm", 35.0, sr.ParameterType.DOUBLE),
            "Seitenlänge eines Schachfeldes auf dem ChArUco-Board in mm"
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

        # Conveyor Frame Pose (Output für 3D-Visualisierung in AICA Studio via TF)
        self._conveyor_pose = sr.CartesianPose(
            "conveyor_frame", "world"
        )
        self.add_output(
            "conveyor_pose",
            "_conveyor_pose",
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
        """Generiert eine geometrisch zentrierte Orbit-Trajektorie auf einem Halbkugelsegment."""
        num_wp = max(3, int(self.get_parameter("num_waypoints").get_value()))
        
        self._waypoint_labels = []
        self._waypoint_offsets = []

        # 1. Zentrum
        self._waypoint_labels.append("center")
        self._waypoint_offsets.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        # 2. Äußerer Orbit-Ring
        num_ring_points = min(8, num_wp - 1)
        for i in range(num_ring_points):
            angle = (2.0 * math.pi / num_ring_points) * i
            label = f"orbit_ring_{i+1}"
            self._waypoint_labels.append(label)
            self._waypoint_offsets.append((angle, 1.0, 0.0, 0.0, 0.0, 0.0))

        # 3. Innerer Kreis
        if len(self._waypoint_labels) < num_wp:
            remaining = num_wp - len(self._waypoint_labels)
            for j in range(remaining):
                angle = (2.0 * math.pi / remaining) * j + (math.pi / 4.0)
                label = f"orbit_inner_{j+1}"
                self._waypoint_labels.append(label)
                self._waypoint_offsets.append((angle, 0.5, 0.0, 0.0, 0.0, 0.0))

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
                f"Starting automatic orbit calibration sequence with {len(self._waypoint_labels)} waypoints..."
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

        if self._start_ee_transform is None:
            self.get_logger().warn("No start EE transform available — cannot set target pose.")
            return

        T_home = self._start_ee_transform.copy()
        P_home = T_home[:3, 3]
        R_home = T_home[:3, :3]

        if label == "center":
            pos_target = P_home
            R_target = R_home
            self.get_logger().info(f"Moving to orbit waypoint {idx + 1}/{len(self._waypoint_labels)}: center (home position)")
        else:
            angle, radius_scale, _, _, _, _ = self._waypoint_offsets[idx]
            max_r = float(self.get_parameter("max_orbit_radius_mm").get_value()) / 1000.0
            r = max_r * radius_scale

            live_dist_m = None
            try:
                cam_data = list(self._robot_cam_board_pose_msg.data) if hasattr(self._robot_cam_board_pose_msg, "data") else list(self._robot_cam_board_pose_msg)
                if len(cam_data) >= 6 and float(cam_data[2]) > 0.1:
                    live_dist_m = float(cam_data[2])
            except Exception:
                pass

            d = live_dist_m if live_dist_m is not None else float(self.get_parameter("board_distance_mm").get_value()) / 1000.0
            d = max(d, 0.1)

            dx = r * math.cos(angle)
            dy = r * math.sin(angle)
            dz = d - math.sqrt(max(d * d - r * r, 0.001))

            tilt_rad = math.asin(min(r / d, 0.99))
            dr = -tilt_rad * math.sin(angle)
            dp = tilt_rad * math.cos(angle)

            R_tilt = rpy_to_rotation_matrix(dr, dp, 0.0)

            pos_target = P_home + R_home @ np.array([dx, dy, dz], dtype=np.float64)
            R_target = R_home @ R_tilt

            self.get_logger().info(
                f"Moving to orbit waypoint {idx + 1}/{len(self._waypoint_labels)}: {label} "
                f"(radius={r*1000:.1f}mm, tilt={math.degrees(tilt_rad):.1f}° focused on board at {d*1000:.0f}mm)"
            )

        try:
            self._target_pose.set_position(np.array(pos_target, dtype=np.float64))

            quat_target = rotation_matrix_to_quaternion(R_target)
            self._target_pose.set_orientation(np.array(quat_target, dtype=np.float64))

            self.get_logger().info(
                f"Orbit target pose set: pos=({pos_target[0]:.3f}, {pos_target[1]:.3f}, {pos_target[2]:.3f})"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not set target EE pose: {e}")

    def on_step_callback(self):
        now_time = self.get_clock().now()

        if self._state == "IDLE":
            return

        if self._state == "MOVING":
            if self._state_start_time is None:
                self._state_start_time = now_time

            try:
                current_pos = np.array(self._robot_ee_pose.get_position(), dtype=np.float64)
                target_pos = np.array(self._target_pose.get_position(), dtype=np.float64)
                distance_m = np.linalg.norm(current_pos - target_pos)

                dt = (now_time - self._state_start_time).nanoseconds / 1e9
                if distance_m < 0.002 or dt >= 5.0:
                    self._state = "SETTLING"
                    self._state_start_time = now_time
            except Exception:
                dt = (now_time - self._state_start_time).nanoseconds / 1e9
                if dt >= 3.0:
                    self._state = "SETTLING"
                    self._state_start_time = now_time

        elif self._state == "SETTLING":
            settle_target = float(self.get_parameter("settle_time_s").get_value())
            dt = (now_time - self._state_start_time).nanoseconds / 1e9

            if dt >= settle_target:
                self._state = "SAMPLING"
                self._sample_buffer_robot_cam.clear()
                self._sample_buffer_base_cam.clear()
                self._state_start_time = now_time

        elif self._state == "SAMPLING":
            target_samples = int(self.get_parameter("samples_per_waypoint").get_value())

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
                self._sample_buffer_robot_cam.append(robot_cam_data)

            if len(base_cam_data) >= 6:
                self._sample_buffer_base_cam.append(base_cam_data)

            if len(self._sample_buffer_robot_cam) >= target_samples:
                avg_robot_cam = np.mean(self._sample_buffer_robot_cam, axis=0).tolist()
                avg_base_cam = (
                    np.mean(self._sample_buffer_base_cam, axis=0).tolist()
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
                    f"Sampled Orbit Waypoint {self._current_waypoint_idx + 1}/{len(self._waypoints)}"
                )

                self._current_waypoint_idx += 1

                if self._current_waypoint_idx < len(self._waypoints):
                    self._state = "MOVING"
                    self._state_start_time = None
                    self._send_next_waypoint()
                else:
                    self.get_logger().info("Alle Orbit-Wegpunkte abgetastet. Kehre zurück zur Startposition...")
                    self._state = "RETURNING_HOME"
                    self._state_start_time = None
                    self._send_home_waypoint()
            else:
                dt_sample = (now_time - self._state_start_time).nanoseconds / 1e9
                if dt_sample > 2.0 and int(dt_sample) % 2 == 0:
                    self.get_logger().warning(
                        f"Wegpunkt {self._current_waypoint_idx + 1}/{len(self._waypoints)}: "
                        f"Warte auf ChArUco-Erkennung der RobotCam ({len(self._sample_buffer_robot_cam)}/{target_samples} Samples)..."
                    )

        elif self._state == "RETURNING_HOME":
            if self._state_start_time is None:
                self._state_start_time = now_time

            try:
                current_pos = np.array(self._robot_ee_pose.get_position(), dtype=np.float64)
                target_pos = np.array(self._target_pose.get_position(), dtype=np.float64)
                distance_m = np.linalg.norm(current_pos - target_pos)

                dt = (now_time - self._state_start_time).nanoseconds / 1e9
                if distance_m < 0.002 or dt >= 5.0:
                    self._state = "SOLVING"
            except Exception:
                dt = (now_time - self._state_start_time).nanoseconds / 1e9
                if dt >= 3.0:
                    self._state = "SOLVING"

        elif self._state == "SOLVING":
            self.get_logger().info(
                f"Computing Extrinsic Calibration from {len(self._collected_samples)} samples..."
            )

            try:
                off_x_m = float(self.get_parameter("conveyor_offset_x_mm").get_value()) / 1000.0
                off_y_m = float(self.get_parameter("conveyor_offset_y_mm").get_value()) / 1000.0
                off_z_m = float(self.get_parameter("conveyor_offset_z_mm").get_value()) / 1000.0

                board_rows = int(self.get_parameter("board_rows").get_value())
                board_cols = int(self.get_parameter("board_cols").get_value())
                square_size_m = float(self.get_parameter("square_size_mm").get_value()) / 1000.0

                # Board-Zentrum im gedrehten conveyor_frame (-X wegen Ry(180°))
                board_center_x_m = off_x_m - (board_cols * square_size_m) / 2.0
                board_center_y_m = off_y_m + (board_rows * square_size_m) / 2.0

                self.get_logger().info(
                    f"Board center in conveyor_frame: X={board_center_x_m*1000:.1f} mm, "
                    f"Y={board_center_y_m*1000:.1f} mm"
                )

                result: CalibrationResult = solve_eye_in_hand(
                    self._collected_samples,
                    conveyor_offset_m=(off_x_m, off_y_m, off_z_m)
                )

                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                # Sende berechnetes T_robot_conveyor als CartesianPose-Signal für AICA Studio
                if result.T_robot_conveyor is not None:
                    T_conv = result.T_robot_conveyor
                    self._conveyor_pose.set_position(T_conv[:3, 3])
                    quat_conv = rotation_matrix_to_quaternion(T_conv[:3, :3])
                    self._conveyor_pose.set_orientation(quat_conv)

                save_path = self.get_parameter("calibration_file_path").get_value()
                save_calibration_yaml(
                    save_path, result,
                    board_center_conveyor_mm=(board_center_x_m * 1000.0, board_center_y_m * 1000.0, off_z_m * 1000.0)
                )

                self.get_logger().info(
                    f"Calibration successful! RMSE: {result.position_rmse_mm:.2f} mm. Saved to {save_path}"
                )

                self.set_predicate("is_calibrated", True)
                self.set_predicate("is_running", False)
                self._state = "FINISHED"

            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
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
                quat_target = rotation_matrix_to_quaternion(self._start_ee_transform[:3, :3])
                self._target_pose.set_orientation(np.array(quat_target, dtype=np.float64))
                self.get_logger().info(
                    f"Target pose reset to start position: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                )
            except Exception as e:
                self.get_logger().warn(f"Could not reset target EE pose to home: {e}")