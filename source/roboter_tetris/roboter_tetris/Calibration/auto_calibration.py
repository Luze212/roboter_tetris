"""AICA Lifecycle Component: Automatic Extrinsic Orbit Calibration for roboter_tetris."""

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
from std_msgs.msg import Float64MultiArray
from modulo_interfaces.srv import StringTrigger

from .extrinsic_calibration import (
    CalibrationResult, CalibrationSample,
    orthonormalize_rotation,
    rotation_matrix_to_quaternion,
    rpy_to_rotation_matrix,
    save_calibration_json, save_calibration_yaml, solve_eye_in_hand,
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
            "Zielpfad für die generierte calibration.yaml"
        )
        self.add_parameter(
            sr.Parameter("settle_time_s", 0.8, sr.ParameterType.DOUBLE),
            "Wartezeit (s) nach Anfahren einer Pose vor der Bildaufnahme"
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
        self.add_input("base_cam_board_pose", "_base_cam_board_pose_msg", Float64MultiArray)

        self._robot_cam_board_pose_msg = []
        self.add_input("robot_cam_board_pose", "_robot_cam_board_pose_msg", Float64MultiArray)

        self._robot_cam_board_depth_msg = []
        self.add_input("robot_cam_board_depth", "_robot_cam_board_depth_msg", Float64MultiArray)

        # Board geometry [rows, cols, checker_size_mm] published by board_detection
        self._board_geometry_msg = []
        self.add_input("board_geometry", "_board_geometry_msg", Float64MultiArray)

        self._robot_ee_pose = sr.CartesianState("end_effector", "world")
        self.add_input("robot_ee_pose", "_robot_ee_pose", EncodedState)

        # Outputs
        self._target_pose = sr.CartesianPose("calibration_target", "world")
        self.add_output("target_ee_pose", "_target_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._conveyor_pose = sr.CartesianPose("conveyor_frame", "world")
        self.add_output("conveyor_pose", "_conveyor_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

        self._calibration_matrix = []
        self.add_output("calibration_matrix", "_calibration_matrix", Float64MultiArray)

        self._calibration_rpy = []
        self.add_output("calibration_rpy", "_calibration_rpy", Float64MultiArray)

        # Predicates
        self.add_predicate("is_running", False)
        self.add_predicate("is_calibrated", False)
        self.add_predicate("has_failed", False)

        try:
            self.add_service("start_calibration", StringTrigger, self._on_start_calibration_service)
        except Exception:
            pass

        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._start_ee_transform = None
        self._collected_samples: List[CalibrationSample] = []
        self._sample_buffer_robot_cam: List[List[float]] = []
        self._sample_buffer_base_cam: List[List[float]] = []

    def _generate_waypoints(self):
        num_wp = max(3, int(self.get_parameter("num_waypoints").get_value()))
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
            self.get_logger().info(f"Starting calibration sequence with {len(self._waypoint_labels)} waypoints...")
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

        elif self._state == "SETTLING":
            try:
                self._target_pose.set_position(self._moving_target_pos)
                self._target_pose.set_orientation(self._moving_target_quat)
            except Exception:
                pass
            settle_target = float(self.get_parameter("settle_time_s").get_value())
            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            if dt >= settle_target:
                self._state = "SAMPLING"
                self._sample_buffer_robot_cam.clear()
                self._sample_buffer_base_cam.clear()
                self._state_start_time = now_time

        elif self._state == "SAMPLING":
            try:
                self._target_pose.set_position(self._moving_target_pos)
                self._target_pose.set_orientation(self._moving_target_quat)
            except Exception:
                pass
            target_samples = int(self.get_parameter("samples_per_waypoint").get_value())
            robot_cam_data = list(self._robot_cam_board_pose_msg.data) if hasattr(self._robot_cam_board_pose_msg, "data") else list(self._robot_cam_board_pose_msg)
            base_cam_data = list(self._base_cam_board_pose_msg.data) if hasattr(self._base_cam_board_pose_msg, "data") else list(self._base_cam_board_pose_msg)

            if len(robot_cam_data) >= 6 and np.linalg.norm(robot_cam_data[:3]) > 1e-3:
                self._sample_buffer_robot_cam.append(robot_cam_data)
            if len(base_cam_data) >= 6 and np.linalg.norm(base_cam_data[:3]) > 1e-3:
                self._sample_buffer_base_cam.append(base_cam_data)

            if len(self._sample_buffer_robot_cam) >= target_samples:
                avg_robot_cam = np.mean(self._sample_buffer_robot_cam, axis=0).tolist()
                avg_base_cam = np.mean(self._sample_buffer_base_cam, axis=0).tolist() if self._sample_buffer_base_cam else None

                T_ee = self._get_current_ee_transform()
                
                if np.linalg.norm(avg_robot_cam[:3]) > 1e-3:
                    sample = CalibrationSample(
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
                geom = list(self._board_geometry_msg) if self._board_geometry_msg else []
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
                    # Fallback: conveyor_offset is the board origin; no center correction possible
                    board_w_m = 0.0
                    board_h_m = 0.0
                    self.get_logger().warn(
                        "board_geometry not received from board_detection. "
                        "Board center will equal board origin (conveyor_offset). "
                        "Connect board_detection.board_geometry -> auto_calibration.board_geometry."
                    )

                result: CalibrationResult = solve_eye_in_hand(
                    self._collected_samples, conveyor_offset_m=(off_x_m, off_y_m, off_z_m)
                )

                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                if result.T_robot_conveyor is not None:
                    T_conv = result.T_robot_conveyor
                    self._conveyor_pose.set_position(T_conv[:3, 3].astype(np.float64))
                    quat_conv = rotation_matrix_to_quaternion(T_conv[:3, :3])
                    self._conveyor_pose.set_orientation(np.array(quat_conv, dtype=np.float64))

                    # Compute geometric board center:
                    # The conveyor_offset defines the Board KS origin (corner).
                    # The board center is at origin + half board dimensions along
                    # the board X and Y axes (which align with conveyor X and Y).
                    # In conveyor frame: center = origin + (board_w/2, board_h/2, 0)
                    center_conv = np.array([
                        off_x_m + board_w_m / 2.0,
                        off_y_m + board_h_m / 2.0,
                        off_z_m,
                        1.0
                    ], dtype=np.float64)
                    center_world = (T_conv @ center_conv)[:3]
                    T_inv = np.linalg.inv(T_conv)
                    # Verify round-trip (should equal center_conv[:3])
                    center_back = (T_inv @ np.append(center_world, 1.0))[:3]
                    board_center_mm = (
                        round(center_back[0] * 1000.0, 2),
                        round(center_back[1] * 1000.0, 2),
                        round(center_back[2] * 1000.0, 2)
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
                else:
                    # Fallback: no conveyor transform available, use origin as center
                    board_center_mm = (off_x_m * 1000.0, off_y_m * 1000.0, off_z_m * 1000.0)
                    self.get_logger().warn(
                        "No T_robot_conveyor in result – saving board origin as center (fallback)."
                    )

                save_path = self.get_parameter("calibration_file_path").get_value()

                save_calibration_json(
                    save_path, result,
                    board_center_conveyor_mm=board_center_mm
                )
                save_calibration_yaml(
                    save_path, result,
                    board_center_conveyor_mm=board_center_mm
                )

                self.get_logger().info("==================================================")
                self.get_logger().info("         KALIBRIERUNG ERFOLGREICH BEENDET         ")
                self.get_logger().info("==================================================")
                self.get_logger().info(f"Verwendete Samples: {result.sample_count}")
                self.get_logger().info(f"Position RMSE:    {result.position_rmse_mm:.3f} mm")
                self.get_logger().info(f"Rotation RMSE:    {result.rotation_rmse_deg:.3f} deg")
                
                t_cam = T[:3, 3]
                self.get_logger().info(f"Camera Pos in Base: X={t_cam[0]*1000:.1f}mm, Y={t_cam[1]*1000:.1f}mm, Z={t_cam[2]*1000:.1f}mm")
                
                if result.T_robot_conveyor is not None:
                    t_c = result.T_robot_conveyor[:3, 3]
                    self.get_logger().info(f"Conveyor Origin:    X={t_c[0]*1000:.1f}mm, Y={t_c[1]*1000:.1f}mm, Z={t_c[2]*1000:.1f}mm")
                
                self.get_logger().info(f"Ergebnisse gespeichert unter: {save_path}")
                self.get_logger().info("==================================================")

                self.set_predicate("is_calibrated", True)
                self.set_predicate("is_running", False)
                self._state = "FINISHED"

            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
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