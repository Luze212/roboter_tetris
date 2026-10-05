"""AICA component for robot-camera hand-eye calibration."""

import json
import math
from typing import List

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
import state_representation as sr
from std_msgs.msg import Float64MultiArray, Int32
from modulo_interfaces.srv import StringTrigger

from .handeye_solver import (
    HandEyeCalibrationResult, HandEyeCalibrationSample,
    average_rotation_matrices,
    orthonormalize_rotation,
    rotation_matrix_to_quaternion,
    rpy_to_rotation_matrix,
    save_handeye_calibration_json, solve_robot_cam_handeye,
)


def build_zero_yaw_tilt(r: float, d: float, angle: float) -> np.ndarray:
    """Compute minimal zero-yaw rotation matrix to tilt optical axis directly toward board center."""
    v_focus = np.array([-r * math.cos(angle), -r * math.sin(angle), d], dtype=np.float64)
    z_desired = v_focus / np.linalg.norm(v_focus)
    z_home = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    axis = np.cross(z_home, z_desired)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        return np.eye(3, dtype=np.float64)

    axis = axis / axis_norm
    cos_phi = np.dot(z_home, z_desired)
    phi = math.acos(min(max(cos_phi, -1.0), 1.0))

    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ], dtype=np.float64)

    R_tilt = np.eye(3, dtype=np.float64) + math.sin(phi) * K + (1.0 - math.cos(phi)) * (K @ K)
    return orthonormalize_rotation(R_tilt)


def quaternion_slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two quaternions [w, x, y, z]."""
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


class RobotCamHandEyeCalibration(LifecycleComponent):
    """Calibrates the flange-mounted robot camera against the static base camera."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Parameters
        self.add_parameter(
            sr.Parameter(
                "robot_cam_handeye_file_path",
                "/data/robot_cam_handeye_calibration.json",
                sr.ParameterType.STRING
            ),
            "Absoluter Pfad zur Ergebnisdatei der Robot-Kamera-Hand-Auge-Kalibrierung (persistent unter /data)."
        )
        self.add_parameter(
            sr.Parameter("settle_time_s", 0.8, sr.ParameterType.DOUBLE),
            "Zeit (s), die der Flansch innerhalb der Toleranzen an der Zielpose stehen muss"
        )
        self.add_parameter(
            sr.Parameter("settle_timeout_s", 10.0, sr.ParameterType.DOUBLE),
            "Maximale Wartezeit (s) bis der Flansch die Zielpose erreicht"
        )
        self.add_parameter(
            sr.Parameter("position_tolerance_mm", 2.0, sr.ParameterType.DOUBLE),
            "Zulässiger Flansch-Abstand zur Zielpose während der Messung"
        )
        self.add_parameter(
            sr.Parameter("orientation_tolerance_deg", 2.0, sr.ParameterType.DOUBLE),
            "Zulässige Flansch-Winkelabweichung zur Zielpose während der Messung"
        )
        self.add_parameter(
            sr.Parameter("sampling_timeout_s", 5.0, sr.ParameterType.DOUBLE),
            "Maximale Wartezeit (s) auf genügend neue gültige Board-Beobachtungen"
        )
        self.add_parameter(
            sr.Parameter("max_position_rmse_mm", 10.0, sr.ParameterType.DOUBLE),
            "Maximaler Positions-RMSE des rekonstruierten Boards für ein gültiges Ergebnis"
        )
        self.add_parameter(
            sr.Parameter("max_rotation_rmse_deg", 3.0, sr.ParameterType.DOUBLE),
            "Maximaler Orientierungs-RMSE des rekonstruierten Boards für ein gültiges Ergebnis"
        )
        self.add_parameter(
            sr.Parameter("samples_per_waypoint", 5, sr.ParameterType.INT),
            "Anzahl zu mittelnder Detektionen pro Wegpunkt"
        )
        self.add_parameter(
            sr.Parameter("num_waypoints", 9, sr.ParameterType.INT),
            "Anzahl der Kalibrier-Wegpunkte"
        )
        self.add_parameter(
            sr.Parameter("max_orbit_radius_mm", 50.0, sr.ParameterType.DOUBLE),
            "Maximaler kartesischer Orbit-Radius (in mm)"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_x_mm", -130.0, sr.ParameterType.DOUBLE),
            "X-Offset im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_y_mm", 243.0, sr.ParameterType.DOUBLE),
            "Y-Offset im conveyor_frame in mm"
        )
        self.add_parameter(
            sr.Parameter("conveyor_offset_z_mm", 0.0, sr.ParameterType.DOUBLE),
            "Z-Offset im conveyor_frame in mm"
        )

        # Inputs
        self._base_cam_board_pose_msg = []
        self.add_input("static_base_cam_board_pose", "_base_cam_board_pose_msg", Float64MultiArray)

        self._base_cam_observation_id = -1
        self.add_input("static_base_cam_board_observation_id", "_base_cam_observation_id", Int32)

        self._robot_cam_board_pose_msg = []
        self.add_input("robot_cam_board_pose", "_robot_cam_board_pose_msg", Float64MultiArray)

        self._robot_cam_observation_id = -1
        self.add_input("robot_cam_board_observation_id", "_robot_cam_observation_id", Int32)

        self._robot_cam_board_depth_msg = []
        self.add_input("robot_cam_board_depth", "_robot_cam_board_depth_msg", Float64MultiArray)

        # Board geometry [rows, cols, checker_size_mm] published by board_detection
        self._board_geometry_msg = []
        self.add_input("robot_cam_board_geometry", "_board_geometry_msg", Float64MultiArray)

        # Contract: input is world_T_ur_tool0 from the robot hardware.  It is
        # the flange pose, never a TCP or gripper-point pose.
        self._robot_ee_pose = sr.CartesianState("ur_tool0", "world")
        self.add_input("robot_flange_state", "_robot_ee_pose", EncodedState)

        # Outputs
        self._target_pose = sr.CartesianPose("calibration_target", "world")
        self.add_output("robot_cam_handeye_target_flange_pose", "_target_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._conveyor_pose = sr.CartesianPose("conveyor_frame", "world")
        self.add_output("robot_cam_handeye_conveyor_pose", "_conveyor_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._board_pose_out = sr.CartesianPose("board_frame", "world")
        self.add_output("robot_cam_handeye_board_pose", "_board_pose_out", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._robot_cam_pose_out = sr.CartesianPose("robot_cam_frame", "world")
        self.add_output("robot_cam_handeye_camera_pose", "_robot_cam_pose_out", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._base_cam_pose_out = sr.CartesianPose("base_cam_frame", "world")
        self.add_output("robot_cam_handeye_static_base_cam_pose", "_base_cam_pose_out", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._calibration_matrix = []
        self.add_output("robot_cam_handeye_matrix", "_calibration_matrix", Float64MultiArray)

        self._calibration_rpy = []
        self.add_output("robot_cam_handeye_rpy", "_calibration_rpy", Float64MultiArray)

        # Predicates
        self.add_predicate("is_running", False)
        self.add_predicate("is_calibrated", False)
        self.add_predicate("has_failed", False)

        try:
            self.add_service("start_robot_cam_handeye_calibration", StringTrigger, self._on_start_calibration_service)
        except Exception:
            pass

        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._start_ee_transform = None
        self._collected_samples: List[HandEyeCalibrationSample] = []
        self._sample_buffer_robot_cam: List[List[float]] = []
        self._sample_buffer_base_cam: List[List[float]] = []
        self._sample_buffer_robot_ee: List[np.ndarray] = []
        self._last_robot_cam_observation_id = -1
        self._last_base_cam_observation_id = -1
        self._settled_since_time = None

    def _generate_waypoints(self):
        num_wp = max(4, int(self.get_parameter("num_waypoints").get_value()))
        self._waypoint_labels = ["center"]
        self._waypoint_offsets = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

        num_ring_points = num_wp - 1
        for i in range(num_ring_points):
            angle = (2.0 * math.pi / num_ring_points) * i
            self._waypoint_labels.append(f"orbit_ring_{i+1}")
            self._waypoint_offsets.append((angle, 1.0, 0.0, 0.0, 0.0, 0.0))

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
        """Return world_T_ur_tool0 from the required hardware input pose."""
        T = np.eye(4, dtype=np.float64)
        try:
            pos = self._robot_ee_pose.get_position()
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            # Falls Werte in mm vorliegen (> 2.0m Distanz), strikt in Meter konvertieren!
            if abs(x) > 2.0 or abs(y) > 2.0 or abs(z) > 2.0:
                x /= 1000.0
                y /= 1000.0
                z /= 1000.0

            T[0, 3] = x
            T[1, 3] = y
            T[2, 3] = z

            ori = self._robot_ee_pose.get_orientation()
            if hasattr(ori, "to_rotation_matrix"):
                T[:3, :3] = np.array(ori.to_rotation_matrix(), dtype=np.float64)
            else:
                qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
                T[:3, :3] = np.array([
                    [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                    [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                    [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
                ], dtype=np.float64)
        except Exception as e:
            self.get_logger().warn(f"Could not extract current robot_ee_pose: {e}")
        return T

    @staticmethod
    def _observation_id(value) -> int:
        try:
            if hasattr(value, "data"):
                value = value.data
            return int(value)
        except (TypeError, ValueError):
            return -1

    def _is_ee_at_target(self) -> bool:
        """Check measured flange pose against the currently commanded target."""
        current = self._get_current_ee_transform()
        position_error_mm = float(np.linalg.norm(
            current[:3, 3] - self._moving_target_pos
        ) * 1000.0)
        current_quat = rotation_matrix_to_quaternion(current[:3, :3])
        target_quat = self._moving_target_quat
        dot = abs(float(np.dot(current_quat, target_quat)))
        orientation_error_deg = math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))
        return (
            position_error_mm <= float(self.get_parameter("position_tolerance_mm").get_value())
            and orientation_error_deg <= float(self.get_parameter("orientation_tolerance_deg").get_value())
        )

    def _fail_calibration(self, message: str) -> None:
        self.get_logger().error(message)
        self.set_predicate("has_failed", True)
        self.set_predicate("is_running", False)
        self._state = "FAILED"

    def _on_start_calibration_service(self, request: StringTrigger.Request) -> StringTrigger.Response:
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
            self.get_logger().info(f"Starting robot-camera hand-eye calibration sequence with {len(self._waypoint_labels)} waypoints...")
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
            self.get_logger().warn("No start EE transform available.")
            return
        T_curr = self._get_current_ee_transform()
        self._moving_start_pos = T_curr[:3, 3].copy()
        self._moving_start_quat = rotation_matrix_to_quaternion(T_curr[:3, :3])

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

            # Live distance from camera if available, else fallback 0.35m
            live_dist_m = None
            try:
                cam_data = list(self._robot_cam_board_pose_msg.data) if hasattr(self._robot_cam_board_pose_msg, "data") else list(self._robot_cam_board_pose_msg)
                if len(cam_data) >= 6 and float(cam_data[2]) > 0.1:
                    live_dist_m = float(cam_data[2])
            except Exception:
                pass

            d = live_dist_m if live_dist_m is not None else 0.35
            d = max(d, 0.1)            # Radial position on sphere dome
            dx = r * math.cos(angle)
            dy = r * math.sin(angle)
            dz = d - math.sqrt(max(d * d - r * r, 0.001))

            # Minimal zero-yaw tilt so camera optical axis (+Z_cam) points dead-center at board
            R_tilt = build_zero_yaw_tilt(r, d, angle)

            pos_target = P_home + R_home @ np.array([dx, dy, dz], dtype=np.float64)
            R_target = R_home @ R_tilt

            tilt_deg = math.degrees(math.acos(min(max(d / math.sqrt(r * r + d * d), -1.0), 1.0)))
            self.get_logger().info(
                f"Moving to orbit waypoint {idx + 1}/{len(self._waypoint_labels)}: {label} "
                f"(radius={r*1000:.1f}mm, tilt={tilt_deg:.1f}° focused on board at {d*1000:.0f}mm)"
            )

        self._moving_target_pos = np.array(pos_target, dtype=np.float64)
        self._moving_target_quat = rotation_matrix_to_quaternion(R_target)
        dist_move = np.linalg.norm(self._moving_target_pos - self._moving_start_pos)
        self._moving_duration = max(1.5, dist_move / 0.04)

        try:
            self._target_pose.set_position(self._moving_start_pos)
            self._target_pose.set_orientation(self._moving_start_quat)
        except Exception as e:
            self.get_logger().warn(f"Could not set target EE pose: {e}")

    def on_step_callback(self):
        now_time = self.get_clock().now()
        if self._state == "IDLE":
            return

        if self._state == "MOVING":
            if self._state_start_time is None:
                self._state_start_time = now_time

            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            duration = getattr(self, "_moving_duration", 2.0)
            progress_raw = min(dt / max(duration, 0.1), 1.0)
            p = 0.5 * (1.0 - math.cos(math.pi * progress_raw))

            pos_curr = self._moving_start_pos + p * (self._moving_target_pos - self._moving_start_pos)
            quat_curr = quaternion_slerp(self._moving_start_quat, self._moving_target_quat, p)

            try:
                self._target_pose.set_position(pos_curr)
                self._target_pose.set_orientation(quat_curr)
            except Exception:
                pass

            if progress_raw >= 1.0:
                self._state = "SETTLING"
                self._state_start_time = now_time
                self._settled_since_time = None

        elif self._state == "SETTLING":
            try:
                self._target_pose.set_position(self._moving_target_pos)
                self._target_pose.set_orientation(self._moving_target_quat)
            except Exception:
                pass
            settle_target = float(self.get_parameter("settle_time_s").get_value())
            settle_timeout = float(self.get_parameter("settle_timeout_s").get_value())
            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            if dt >= settle_timeout:
                self._fail_calibration(
                    f"Wegpunkt {self._current_waypoint_idx + 1}: Flansch erreichte die Zielpose nicht "
                    f"innerhalb von {settle_timeout:.1f} s."
                )
                return

            if self._is_ee_at_target():
                if self._settled_since_time is None:
                    self._settled_since_time = now_time
                stable_time = (now_time - self._settled_since_time).nanoseconds / 1e9
                if stable_time >= settle_target:
                    self._state = "SAMPLING"
                    self._sample_buffer_robot_cam.clear()
                    self._sample_buffer_base_cam.clear()
                    self._sample_buffer_robot_ee.clear()
                    self._last_robot_cam_observation_id = self._observation_id(
                        self._robot_cam_observation_id
                    )
                    self._last_base_cam_observation_id = self._observation_id(
                        self._base_cam_observation_id
                    )
                    self._state_start_time = now_time
            else:
                self._settled_since_time = None

        elif self._state == "SAMPLING":
            try:
                self._target_pose.set_position(self._moving_target_pos)
                self._target_pose.set_orientation(self._moving_target_quat)
            except Exception:
                pass
            target_samples = max(1, int(self.get_parameter("samples_per_waypoint").get_value()))
            sampling_timeout = float(self.get_parameter("sampling_timeout_s").get_value())
            elapsed_sampling = (now_time - self._state_start_time).nanoseconds / 1e9
            if elapsed_sampling >= sampling_timeout:
                self._fail_calibration(
                    f"Wegpunkt {self._current_waypoint_idx + 1}: Nur "
                    f"{len(self._sample_buffer_robot_cam)}/{target_samples} neue gültige "
                    f"Roboterkamera-Beobachtungen innerhalb von {sampling_timeout:.1f} s."
                )
                return

            robot_cam_data = list(self._robot_cam_board_pose_msg.data) if hasattr(self._robot_cam_board_pose_msg, "data") else list(self._robot_cam_board_pose_msg)
            base_cam_data = list(self._base_cam_board_pose_msg.data) if hasattr(self._base_cam_board_pose_msg, "data") else list(self._base_cam_board_pose_msg)

            robot_observation_id = self._observation_id(self._robot_cam_observation_id)
            if robot_observation_id > self._last_robot_cam_observation_id:
                self._last_robot_cam_observation_id = robot_observation_id
                if len(robot_cam_data) >= 6 and np.linalg.norm(robot_cam_data[:3]) > 1e-3:
                    self._sample_buffer_robot_cam.append(robot_cam_data[:6])
                    self._sample_buffer_robot_ee.append(self._get_current_ee_transform())

            base_observation_id = self._observation_id(self._base_cam_observation_id)
            if base_observation_id > self._last_base_cam_observation_id:
                self._last_base_cam_observation_id = base_observation_id
                if len(base_cam_data) >= 6 and np.linalg.norm(base_cam_data[:3]) > 1e-3:
                    self._sample_buffer_base_cam.append(base_cam_data[:6])

            if len(self._sample_buffer_robot_cam) >= target_samples:
                avg_robot_cam = np.mean(self._sample_buffer_robot_cam, axis=0).tolist()
                avg_base_cam = np.mean(self._sample_buffer_base_cam, axis=0).tolist() if self._sample_buffer_base_cam else None

                T_ee = np.eye(4, dtype=np.float64)
                T_ee[:3, 3] = np.mean(
                    [transform[:3, 3] for transform in self._sample_buffer_robot_ee], axis=0
                )
                T_ee[:3, :3] = average_rotation_matrices(
                    [transform[:3, :3] for transform in self._sample_buffer_robot_ee]
                )
                
                if np.linalg.norm(avg_robot_cam[:3]) > 1e-3:
                    sample = HandEyeCalibrationSample(
                        T_robot_ee=T_ee,
                        robot_cam_board_pose=avg_robot_cam,
                        base_cam_board_pose=avg_base_cam
                    )
                    self._collected_samples.append(sample)
                    self.get_logger().info(f"Sample {len(self._collected_samples)}/{len(self._waypoints)} erfolgreich aufgenommen.")
                else:
                    self.get_logger().warn("Wegpunkt-Sample verworfen: Ungültige Kamera-Pose.")

                self._current_waypoint_idx += 1
                if self._current_waypoint_idx < len(self._waypoints):
                    self._state = "MOVING"
                    self._state_start_time = None
                    self._send_next_waypoint()
                else:
                    self._state = "RETURNING_HOME"
                    self._state_start_time = None
                    self._send_home_waypoint()

        elif self._state == "RETURNING_HOME":
            if self._state_start_time is None:
                self._state_start_time = now_time
                self._send_home_waypoint()

            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            duration = getattr(self, "_moving_duration", 2.0)
            progress_raw = min(dt / max(duration, 0.1), 1.0)
            p = 0.5 * (1.0 - math.cos(math.pi * progress_raw))

            pos_curr = self._moving_start_pos + p * (self._moving_target_pos - self._moving_start_pos)
            quat_curr = quaternion_slerp(self._moving_start_quat, self._moving_target_quat, p)

            try:
                self._target_pose.set_position(pos_curr)
                self._target_pose.set_orientation(quat_curr)
            except Exception:
                pass

            if progress_raw >= 1.0:
                self._state = "SOLVING"

        elif self._state == "SOLVING":
            try:
                off_x_m = float(self.get_parameter("conveyor_offset_x_mm").get_value()) / 1000.0
                off_y_m = float(self.get_parameter("conveyor_offset_y_mm").get_value()) / 1000.0
                off_z_m = float(self.get_parameter("conveyor_offset_z_mm").get_value()) / 1000.0

                # Read board geometry from board_detection's board_geometry output.
                # Format: [rows, cols, checker_size_mm]  (no parameter duplication needed)
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
                    self.get_logger().info(
                        f"Board geometry from board_detection: "
                        f"{board_rows}x{board_cols} @ {geom[2]:.1f}mm "
                        f"-> {board_w_m*1000:.1f}x{board_h_m*1000:.1f}mm"
                    )
                else:
                    raise ValueError(
                        "board_geometry fehlt oder ist ungültig. Die Board-Abmessungen werden "
                        "nicht geschätzt; bitte die Board-Geometry-Verbindung prüfen."
                    )

                result: HandEyeCalibrationResult = solve_robot_cam_handeye(
                    self._collected_samples, conveyor_offset_m=(off_x_m, off_y_m, off_z_m)
                )

                max_position_rmse_mm = float(self.get_parameter("max_position_rmse_mm").get_value())
                max_rotation_rmse_deg = float(self.get_parameter("max_rotation_rmse_deg").get_value())
                if result.position_rmse_mm > max_position_rmse_mm:
                    raise ValueError(
                        f"Positions-RMSE {result.position_rmse_mm:.3f} mm überschreitet den "
                        f"Grenzwert von {max_position_rmse_mm:.3f} mm."
                    )
                if result.rotation_rmse_deg > max_rotation_rmse_deg:
                    raise ValueError(
                        f"Orientierungs-RMSE {result.rotation_rmse_deg:.3f} Grad überschreitet den "
                        f"Grenzwert von {max_rotation_rmse_deg:.3f} Grad."
                    )

                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                if result.T_robot_conveyor is not None:
                    T_conv = result.T_robot_conveyor
                    self._conveyor_pose.set_position(T_conv[:3, 3].astype(np.float64))
                    quat_conv = rotation_matrix_to_quaternion(T_conv[:3, :3])
                    self._conveyor_pose.set_orientation(np.array(quat_conv, dtype=np.float64))

                if result.T_robot_board is not None:
                    T_b = result.T_robot_board
                    self._board_pose_out.set_position(T_b[:3, 3].astype(np.float64))
                    quat_b = rotation_matrix_to_quaternion(T_b[:3, :3])
                    self._board_pose_out.set_orientation(np.array(quat_b, dtype=np.float64))

                if result.T_robot_base_cam is not None:
                    T_rc = result.T_robot_base_cam
                    self._robot_cam_pose_out.set_position(T_rc[:3, 3].astype(np.float64))
                    quat_rc = rotation_matrix_to_quaternion(T_rc[:3, :3])
                    self._robot_cam_pose_out.set_orientation(np.array(quat_rc, dtype=np.float64))

                if result.T_robot_base_static_cam is not None:
                    T_bc = result.T_robot_base_static_cam
                    self._base_cam_pose_out.set_position(T_bc[:3, 3].astype(np.float64))
                    quat_bc = rotation_matrix_to_quaternion(T_bc[:3, :3])
                    self._base_cam_pose_out.set_orientation(np.array(quat_bc, dtype=np.float64))

                # Board center depends only on geometry and the configured origin
                # in conveyor coordinates, not on observations of the base camera.
                # Ry(180): board +X = conveyor -X, board +Y = conveyor +Y.
                board_center_mm = (
                    round((off_x_m - board_w_m / 2.0) * 1000.0, 2),
                    round((off_y_m + board_h_m / 2.0) * 1000.0, 2),
                    round(off_z_m * 1000.0, 2),
                )
                self.get_logger().info(
                    f"Board KS-Ursprung (conveyor): X={off_x_m*1000:.1f} mm, Y={off_y_m*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Board-Abmessungen: {board_w_m*1000:.1f} x {board_h_m*1000:.1f} mm"
                )
                self.get_logger().info(
                    f"Berechnetes Board-Zentrum (conveyor): "
                    f"X={board_center_mm[0]:.1f} mm, Y={board_center_mm[1]:.1f} mm"
                )

                save_path = self.get_parameter("robot_cam_handeye_file_path").get_value()

                save_handeye_calibration_json(
                    save_path, result,
                    board_center_conveyor_mm=board_center_mm
                )

                self.get_logger().info("==================================================")
                self.get_logger().info(" ROBOT-KAMERA-HAND-AUGE-KALIBRIERUNG ERFOLGREICH BEENDET ")
                self.get_logger().info("==================================================")
                self.get_logger().info(f"Verwendete Samples: {result.sample_count}")
                self.get_logger().info(f"Position RMSE:    {result.position_rmse_mm:.3f} mm")
                self.get_logger().info(f"Rotation RMSE:    {result.rotation_rmse_deg:.3f} deg")
                self.get_logger().info(
                    f"Flansch-Rotationsspanne: {result.flange_rotation_span_deg:.3f} deg"
                )
                
                t_cam = T[:3, 3]
                self.get_logger().info(f"Robot Cam Pos in Base: X={t_cam[0]*1000:.1f}mm, Y={t_cam[1]*1000:.1f}mm, Z={t_cam[2]*1000:.1f}mm")
                
                if result.T_robot_base_static_cam is not None:
                    t_bcam = result.T_robot_base_static_cam[:3, 3]
                    self.get_logger().info(f"Base Cam Pos in Base:  X={t_bcam[0]*1000:.1f}mm, Y={t_bcam[1]*1000:.1f}mm, Z={t_bcam[2]*1000:.1f}mm")
                else:
                    self.get_logger().info("Base Cam Pos in Base:  N/A (base_cam_board_pose nicht empfangen)")

                if result.T_robot_conveyor is not None:
                    t_c = result.T_robot_conveyor[:3, 3]
                    self.get_logger().info(f"Conveyor Origin:    X={t_c[0]*1000:.1f}mm, Y={t_c[1]*1000:.1f}mm, Z={t_c[2]*1000:.1f}mm")
                
                self.get_logger().info(f"Ergebnisse gespeichert unter: {save_path}")
                self.get_logger().info("==================================================")

                self.set_predicate("is_calibrated", True)
                self.set_predicate("is_running", False)
                self._state = "FINISHED"

            except Exception as e:
                self.get_logger().error(f"Robot-camera hand-eye calibration failed: {e}")
                self.set_predicate("has_failed", True)
                self.set_predicate("is_running", False)
                self._state = "FAILED"

    def _send_home_waypoint(self):
        if self._start_ee_transform is not None:
            try:
                T_curr = self._get_current_ee_transform()
                self._moving_start_pos = T_curr[:3, 3].copy()
                self._moving_start_quat = rotation_matrix_to_quaternion(T_curr[:3, :3])

                self._moving_target_pos = self._start_ee_transform[:3, 3].copy()
                self._moving_target_quat = rotation_matrix_to_quaternion(self._start_ee_transform[:3, :3])
                dist_move = np.linalg.norm(self._moving_target_pos - self._moving_start_pos)
                self._moving_duration = max(1.5, dist_move / 0.04)

                self._target_pose.set_position(self._moving_start_pos)
                self._target_pose.set_orientation(self._moving_start_quat)
                self.get_logger().info("Returning robot to home position (user start pose)...")
            except Exception as e:
                self.get_logger().warn(f"Could not reset target EE pose: {e}")
