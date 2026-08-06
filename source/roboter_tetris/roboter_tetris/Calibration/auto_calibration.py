"""AICA Lifecycle Component: Automatic Extrinsic Calibration for roboter_tetris.

Triggers an automatic multi-waypoint movement sequence, samples board detections
from both Eye-in-Hand (robot_cam) and static conveyor (base_cam) cameras, computes the
rigid transformation T_robot_base_cam, and writes calibration.json.
"""

import json
import os
from typing import List, Optional

import numpy as np
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from modulo_interfaces.srv import StringTrigger

from .extrinsic_calibration import (
    CalibrationResult, CalibrationSample, rpy_to_rotation_matrix,
    save_calibration_json, solve_eye_in_hand,
)

STALE_TIMEOUT_S = 1.0


class AutoCalibration(LifecycleComponent):
    """AICA component for automated extrinsic camera-robot calibration."""

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # -- Parameters --
        self.add_parameter(
            sr.Parameter("calibration_file_path",
                         "/home/tetripick/Desktop/AICA/roboter_tetris/source/roboter_tetris/roboter_tetris/Calibration/calibration.json",
                         sr.ParameterType.STRING),
            "Zielpfad für die generierte calibration.json"
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
            sr.Parameter("start_calibration", False, sr.ParameterType.BOOL),
            "Trigger-Parameter: Auf True setzen, um die Kalibrierung zu starten"
        )

        # -- Inputs --
        self._base_cam_board_pose_msg = Float64MultiArray()
        self.add_input("base_cam_board_pose", "_base_cam_board_pose_msg", Float64MultiArray)

        self._robot_cam_board_pose_msg = Float64MultiArray()
        self.add_input("robot_cam_board_pose", "_robot_cam_board_pose_msg", Float64MultiArray)

        self._robot_ee_pose = sr.CartesianState("end_effector", "robot_base")
        self.add_input("robot_ee_pose", "_robot_ee_pose", sr.CartesianState)

        # -- Outputs --
        self._calibration_matrix = []
        self.add_output("calibration_matrix", "_calibration_matrix", Float64MultiArray)

        self._calibration_rpy = []
        self.add_output("calibration_rpy", "_calibration_rpy", Float64MultiArray)

        # -- Predicates --
        self.add_predicate("is_running", False)
        self.add_predicate("is_calibrated", False)
        self.add_predicate("has_failed", False)

        # -- State machine internal variables --
        self._state = "IDLE"  # States: IDLE, MOVING, SETTLING, SAMPLING, SOLVING, FINISHED, FAILED
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._collected_samples: List[CalibrationSample] = []
        self._sample_buffer_robot_cam: List[List[float]] = []
        self._sample_buffer_base_cam: List[List[float]] = []

        # JTC Service Client
        self._jtc_client = None

        # Default calibration waypoints (TF frames or pose targets)
        # Note: Can be overridden or expanded for specific workspace setups
        self._waypoints = [
            "calib_pose_center",
            "calib_pose_tilt_left",
            "calib_pose_tilt_right",
            "calib_pose_high",
            "calib_pose_low",
        ]

    def on_configure_callback(self) -> bool:
        # Non-blocking service client setup according to AICA architecture guidelines
        self._jtc_client = self.create_client(StringTrigger, "/jtc/set_trajectory")
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._collected_samples.clear()
        self.set_predicate("is_running", False)
        self.set_predicate("is_calibrated", False)
        self.set_predicate("has_failed", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _get_current_ee_transform(self) -> np.ndarray:
        """Extract 4x4 matrix from sr.CartesianPose self._robot_ee_pose."""
        T = np.eye(4, dtype=np.float64)
        pos = self._robot_ee_pose.get_position()
        T[0, 3] = pos[0]
        T[1, 3] = pos[1]
        T[2, 3] = pos[2]

        ori = self._robot_ee_pose.get_orientation()
        # Quaternion [qx, qy, qz, qw] to R
        # In state_representation, get_orientation returns numpy array [qx, qy, qz, qw] or matrix
        if hasattr(ori, "to_rotation_matrix"):
            T[:3, :3] = ori.to_rotation_matrix()
        elif len(ori) == 4:
            qx, qy, qz, qw = ori
            T[:3, :3] = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
            ], dtype=np.float64)
        return T

    def _send_waypoint_command(self, frame_name: str):
        """Send trajectory command to JTC via StringTrigger non-blockingly."""
        if not self._jtc_client.service_is_ready():
            self.get_logger().warn("JTC service /jtc/set_trajectory not ready.")
            return False

        payload = json.dumps({"frames": [frame_name], "durations": [2.5]})
        req = StringTrigger.Request()
        req.payload = payload

        future = self._jtc_client.call_async(req)
        future.add_done_callback(self._jtc_response_callback)
        return True

    def _jtc_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f"JTC Rejected trajectory: {res.message}")
        except Exception as e:
            self.get_logger().error(f"JTC Service call failed: {e}")

    def on_step_callback(self):
        now_time = self.get_clock().now()

        # Check for trigger start parameter
        start_param = self.get_parameter("start_calibration").get_value()
        if start_param and self._state == "IDLE":
            self.set_parameter(sr.Parameter("start_calibration", False, sr.ParameterType.BOOL))
            self.get_logger().info("Starting automatic calibration sequence...")
            self._state = "MOVING"
            self._current_waypoint_idx = 0
            self._collected_samples.clear()
            self.set_predicate("is_running", True)
            self.set_predicate("is_calibrated", False)
            self.set_predicate("has_failed", False)
            self._send_next_waypoint()
            return

        if self._state == "IDLE":
            return

        # State machine execution
        if self._state == "MOVING":
            # Wait for motion duration / settlement (or predicate check if available)
            if self._state_start_time is None:
                self._state_start_time = now_time
            dt = (now_time - self._state_start_time).nanoseconds / 1e9
            if dt >= 3.0:  # Allow 3.0s total motion duration
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

            # Collect robot_cam board pose if present
            if len(self._robot_cam_board_pose_msg.data) >= 6:
                self._sample_buffer_robot_cam.append(list(self._robot_cam_board_pose_msg.data))

            # Collect base_cam board pose if present
            if len(self._base_cam_board_pose_msg.data) >= 6:
                self._sample_buffer_base_cam.append(list(self._base_cam_board_pose_msg.data))

            if len(self._sample_buffer_robot_cam) >= target_samples:
                # Average readings
                avg_robot_cam = np.mean(self._sample_buffer_robot_cam, axis=0).tolist()
                avg_base_cam = (np.mean(self._sample_buffer_base_cam, axis=0).tolist()
                                if self._sample_buffer_base_cam else None)

                T_ee = self._get_current_ee_transform()
                sample = CalibrationSample(
                    T_robot_ee=T_ee,
                    robot_cam_board_pose=avg_robot_cam,
                    base_cam_board_pose=avg_base_cam
                )
                self._collected_samples.append(sample)
                self.get_logger().info(f"Sampled Waypoint {self._current_waypoint_idx + 1}/{len(self._waypoints)}")

                self._current_waypoint_idx += 1
                if self._current_waypoint_idx < len(self._waypoints):
                    self._state = "MOVING"
                    self._state_start_time = None
                    self._send_next_waypoint()
                else:
                    self._state = "SOLVING"

        elif self._state == "SOLVING":
            self.get_logger().info(f"Computing Extrinsic Calibration from {len(self._collected_samples)} samples...")
            try:
                result: CalibrationResult = solve_eye_in_hand(self._collected_samples)

                # Output calibration matrix and rpy
                T = result.T_robot_base_cam
                self._calibration_matrix = T.flatten().tolist()

                save_path = self.get_parameter("calibration_file_path").get_value()
                save_calibration_json(save_path, result)

                self.get_logger().info(f"Calibration successful! RMSE: {result.position_rmse_mm:.2f} mm. Saved to {save_path}")
                self.set_predicate("is_calibrated", True)
                self.set_predicate("is_running", False)
                self._state = "FINISHED"

            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
                self.set_predicate("has_failed", True)
                self.set_predicate("is_running", False)
                self._state = "FAILED"

        elif self._state == "FINISHED" or self._state == "FAILED":
            # Idle until next manual trigger
            pass

    def _send_next_waypoint(self):
        target_frame = self._waypoints[self._current_waypoint_idx]
        self.get_logger().info(f"Moving to waypoint {self._current_waypoint_idx + 1}: {target_frame}")
        self._send_waypoint_command(target_frame)
