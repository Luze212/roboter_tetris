"""AICA Lifecycle Component: Automatic Extrinsic Calibration for roboter_tetris."""

import json
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
    save_calibration_json, solve_eye_in_hand,
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
                "/home/tetripick/Desktop/AICA/roboter_tetris/source/"
                "roboter_tetris/roboter_tetris/Calibration/calibration.json",
                sr.ParameterType.STRING
            ),
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
            sr.Parameter(
                "jtc_service_name",
                "/joint_trajectory_controller/set_trajectory",
                sr.ParameterType.STRING
            ),
            "Service-Name des AICA JTC Controllers"
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

        self._robot_ee_pose = sr.CartesianPose(
            "end_effector", "robot_base"
        )
        self.add_input(
            "robot_ee_pose",
            "_robot_ee_pose",
            EncodedState
        )

        # Target EE pose
        self._target_pose = sr.CartesianPose(
            "calibration_target", "robot_base"
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
        self._collected_samples: List[CalibrationSample] = []
        self._sample_buffer_robot_cam: List[List[float]] = []
        self._sample_buffer_base_cam: List[List[float]] = []
        self._jtc_client = None

        self._waypoints = [
            "calib_pose_center",
            "calib_pose_tilt_left",
            "calib_pose_tilt_right",
            "calib_pose_high",
            "calib_pose_low",
        ]

    def on_configure_callback(self) -> bool:
        srv_name = (
            self.get_parameter("jtc_service_name").get_value()
            or "/jtc/set_trajectory"
        )
        self._jtc_client = self.create_client(StringTrigger, srv_name)
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._current_waypoint_idx = 0
        self._state_start_time = None
        self._collected_samples.clear()

        self.set_predicate("is_running", False)
        self.set_predicate("is_calibrated", False)
        self.set_predicate("has_failed", False)

        # Start calibration immediately after activation
        self._on_start_calibration()

        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def _get_current_ee_transform(self) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)

        try:
            pos = self._robot_ee_pose.get_position()
            T[0, 3] = pos[0]
            T[1, 3] = pos[1]
            T[2, 3] = pos[2]

            ori = self._robot_ee_pose.get_orientation()

            if hasattr(ori, "to_rotation_matrix"):
                T[:3, :3] = ori.to_rotation_matrix()
            elif len(ori) == 4:
                qx, qy, qz, qw = ori
                T[:3, :3] = np.array([
                    [
                        1 - 2 * (qy**2 + qz**2),
                        2 * (qx*qy - qz*qw),
                        2 * (qx*qz + qy*qw),
                    ],
                    [
                        2 * (qx*qy + qz*qw),
                        1 - 2 * (qx**2 + qz**2),
                        2 * (qy*qz - qx*qw),
                    ],
                    [
                        2 * (qx*qz - qy*qw),
                        2 * (qy*qz + qx*qw),
                        1 - 2 * (qx**2 + qy**2),
                    ],
                ])

        except Exception as e:
            self.get_logger().warn(
                f"Could not extract current robot_ee_pose: {e}"
            )

        return T

    def _send_waypoint_command(self, waypoint_target):
        srv_name = (
            self.get_parameter("jtc_service_name").get_value()
            or "/jtc/set_trajectory"
        )

        if self._jtc_client is None:
            self._jtc_client = self.create_client(
                StringTrigger, srv_name
            )

        if not self._jtc_client.service_is_ready():
            self.get_logger().error(
                f"JTC service '{srv_name}' is NOT ready. "
                "Please check if the Joint Trajectory Controller is loaded "
                "and active."
            )
            return False

        if isinstance(waypoint_target, str):
            payload = json.dumps({
                "frames": [waypoint_target],
                "durations": [2.5]
            })
        elif isinstance(waypoint_target, dict):
            payload = json.dumps(waypoint_target)
        else:
            payload = json.dumps({
                "frames": [str(waypoint_target)],
                "durations": [2.5]
            })

        req = StringTrigger.Request()
        req.payload = payload

        future = self._jtc_client.call_async(req)
        future.add_done_callback(self._jtc_response_callback)
        return True

    def _jtc_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(
                    f"JTC Rejected trajectory: {res.message}"
                )
        except Exception as e:
            self.get_logger().error(
                f"JTC Service call failed: {e}"
            )

    def _on_start_calibration_service(
        self,
        request: StringTrigger.Request
    ) -> StringTrigger.Response:

        response = StringTrigger.Response()

        if self._state not in ("IDLE", "FINISHED", "FAILED"):
            response.success = False
            response.message = (
                f"Kalibrierung läuft bereits (Zustand: {self._state})."
            )
            return response

        self._on_start_calibration()

        response.success = True
        response.message = "Kalibriersequenz gestartet."
        return response

    def _on_start_calibration(self):
        if self._state in ("IDLE", "FINISHED", "FAILED"):
            self.get_logger().info(
                "Starting automatic calibration sequence..."
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
        target_frame = self._waypoints[self._current_waypoint_idx]

        self.get_logger().info(
            f"Moving to waypoint "
            f"{self._current_waypoint_idx + 1}/"
            f"{len(self._waypoints)}: {target_frame}"
        )

        offsets = [
            [0.00, 0.00, 0.00],
            [0.04, 0.00, 0.00],
            [-0.04, 0.00, 0.00],
            [0.00, 0.04, 0.02],
            [0.00, -0.04, -0.02],
        ]

        if self._start_ee_transform is not None:
            idx = min(
                self._current_waypoint_idx,
                len(offsets) - 1
            )

            dx, dy, dz = offsets[idx]
            T_target = self._start_ee_transform.copy()

            T_target[0, 3] += dx
            T_target[1, 3] += dy
            T_target[2, 3] += dz

            try:
                self._target_pose.set_position(T_target[:3, 3])
                self._target_pose.set_orientation(T_target[:3, :3])

                self.get_logger().info(
                    "Target EE Pose updated."
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Could not update target EE pose: {e}"
                )

        self._send_waypoint_command(target_frame)

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
                    self._state = "SOLVING"

        elif self._state == "SOLVING":
            self.get_logger().info(
                f"Computing Extrinsic Calibration from "
                f"{len(self._collected_samples)} samples..."
            )

            try:
                result: CalibrationResult = solve_eye_in_hand(
                    self._collected_samples
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