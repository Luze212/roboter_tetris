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
    rpy_to_rotation_matrix, save_calibration_json, solve_eye_in_hand,
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
            "Anzahl der Kalibrier-Wegpunkte (Standard: 9)"
        )
        # PARAMETER: Radius des Orbits um das Board-Zentrum in mm (Standard: 50.0 mm)
        self.add_parameter(
            sr.Parameter("max_orbit_radius_mm", 50.0, sr.ParameterType.DOUBLE),
            "Maximaler kartesischer Orbit-Radius (in mm) um das Board-Zentrum"
        )
        # PARAMETER: Max. Kippwinkel in Grad zum Board-Zentrum (Standard: 5.0°)
        self.add_parameter(
            sr.Parameter("max_rotation_angle_deg", 5.0, sr.ParameterType.DOUBLE),
            "Maximaler Neigungswinkel (in Grad) während der Orbit-Schwenks"
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
        """Generiert eine geometrisch zentrierte Orbit-Trajektorie auf einem Halbkugelsegment."""
        num_wp = max(3, int(self.get_parameter("num_waypoints").get_value()))
        
        # Radius in mm -> Meter
        radius_m = float(self.get_parameter("max_orbit_radius_mm").get_value()) / 1000.0
        tilt_deg = float(self.get_parameter("max_rotation_angle_deg").get_value())
        tilt_rad = math.radians(tilt_deg)

        self._waypoint_labels = []
        self._waypoint_offsets = []

        # 1. Zentrum (Blick senkrecht von oben auf den eingelernten Startpunkt)
        self._waypoint_labels.append("center")
        self._waypoint_offsets.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        # 2. Äußerer Orbit-Ring mit kontinuierlicher Verkippung zur Mitte
        num_ring_points = min(8, num_wp - 1)
        for i in range(num_ring_points):
            angle = (2.0 * math.pi / num_ring_points) * i
            
            dx = radius_m * math.cos(angle)
            dy = radius_m * math.sin(angle)
            dz = 0.01 * (1.0 if i % 2 == 0 else -1.0)  # Leichte Entkopplung in Z

            # Entgegengesetzte Verkippung zur Beibehaltung des Bildfokus
            dr = -tilt_rad * math.sin(angle)  # Roll
            dp = tilt_rad * math.cos(angle)   # Pitch
            dyaw = 0.0

            label = f"orbit_ring_{i+1}"
            self._waypoint_labels.append(label)
            self._waypoint_offsets.append((dx, dy, dz, dr, dp, dyaw))

        # 3. Innerer Kreis (falls mehr als 9 Punkte konfiguriert werden)
        if len(self._waypoint_labels) < num_wp:
            remaining = num_wp - len(self._waypoint_labels)
            for j in range(remaining):
                angle = (2.0 * math.pi / remaining) * j + (math.pi / 4.0)
                dx = (radius_m * 0.5) * math.cos(angle)
                dy = (radius_m * 0.5) * math.sin(angle)
                dz = 0.015
                
                dr = -(tilt_rad * 0.5) * math.sin(angle)
                dp = (tilt_rad * 0.5) * math.cos(angle)
                dyaw = 0.0

                label = f"orbit_inner_{j+1}"
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

            # Fixieren der Teached-In/Startpose als relativen Nullpunkt
            self._start_ee_transform = self._get_current_ee_transform()
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        idx = self._current_waypoint_idx
        label = self._waypoint_labels[idx]
        dx, dy, dz, dr, dp, dyaw = self._waypoint_offsets[idx]

        self.get_logger().info(
            f"Moving to orbit waypoint {idx + 1}/{len(self._waypoint_labels)}: {label} "
            f"(offset trans=[{dx:.3f}, {dy:.3f}, {dz:.3f}] m, rot_rpy=[{math.degrees(dr):.1f}°, {math.degrees(dp):.1f}°, {math.degrees(dyaw):.1f}°])"
        )

        if self._start_ee_transform is None:
            self.get_logger().warn("No start EE transform available — cannot set target pose.")
            return

        T_target = self._start_ee_transform.copy()
        T_target[0, 3] += dx
        T_target[1, 3] += dy
        T_target[2, 3] += dz

        R_tilt = rpy_to_rotation_matrix(dr, dp, dyaw)
        R_target = T_target[:3, :3] @ R_tilt
        T_target[:3, :3] = R_target

        try:
            pos = T_target[:3, 3].tolist()
            self._target_pose.set_position(np.array(pos, dtype=np.float64))

            quat_target = rotation_matrix_to_quaternion(T_target[:3, :3])
            self._target_pose.set_orientation(np.array(quat_target, dtype=np.float64))

            self.get_logger().info(
                f"Orbit target pose set: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
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

            # Ereignisbasierter Stopp-Check (< 2.0 mm Ist-Ziel-Abstand)
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

                result: CalibrationResult = solve_eye_in_hand(
                    self._collected_samples,
                    conveyor_offset_m=(off_x_m, off_y_m, off_z_m)
                )

                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                save_path = self.get_parameter("calibration_file_path").get_value()
                save_calibration_json(save_path, result)

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