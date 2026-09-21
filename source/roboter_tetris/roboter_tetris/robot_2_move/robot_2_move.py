"""Manual Cartesian target generator for robot 2 in AICA.

Connect ``target_ee_pose`` to the ``attractor_pose`` input of a Signal Point
Attractor.  The component publishes a smooth Cartesian target trajectory; the
Point Attractor and the robot controller execute the physical motion.
"""

import json
import math
import os
from typing import Optional, Tuple

import numpy as np
import state_representation as sr
from clproto import MessageType
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from modulo_interfaces.srv import StringTrigger
from std_msgs.msg import Float64MultiArray


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return the normalized quaternion [w, x, y, z] for ``rotation``."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 2] - rotation[2, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


class Robot2Move(LifecycleComponent):
    """Move robot 2 to an operator-entered Cartesian position.

    Target coordinates are millimetres.  They are interpreted in ``conveyor``
    coordinates by default, or directly in the robot base ``world`` frame when
    ``coordinate_frame`` is set to ``world``.  The current EE orientation is
    retained for every move.
    """

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        self.add_parameter(sr.Parameter("target_x_mm", 0.0, sr.ParameterType.DOUBLE),
                           "Ziel-X in mm im gewählten Koordinatenframe.")
        self.add_parameter(sr.Parameter("target_y_mm", 0.0, sr.ParameterType.DOUBLE),
                           "Ziel-Y in mm im gewählten Koordinatenframe.")
        self.add_parameter(sr.Parameter("target_z_mm", 200.0, sr.ParameterType.DOUBLE),
                           "Ziel-Z in mm im gewählten Koordinatenframe.")
        self.add_parameter(sr.Parameter("coordinate_frame", "conveyor", sr.ParameterType.STRING),
                           "Zielkoordinaten: conveyor oder world.")
        self.add_parameter(
            sr.Parameter("calibration_file_path", "/tmp/calibration.json", sr.ParameterType.STRING),
            "Pfad zur calibration.json; nur für den conveyor-Frame erforderlich.")
        self.add_parameter(sr.Parameter("move_speed_m_s", 0.05, sr.ParameterType.DOUBLE),
                           "Geplante Geschwindigkeit der Zieltrajektorie in m/s.")
        self.add_parameter(sr.Parameter("max_travel_distance_m", 0.50, sr.ParameterType.DOUBLE),
                           "Sicherheitsgrenze: maximale Fahrdistanz eines einzelnen Befehls in m.")

        self._robot_ee_pose = sr.CartesianState("end_effector", "world")
        self.add_input("robot_ee_pose", "_robot_ee_pose", EncodedState)

        self._target_pose = sr.CartesianPose("robot_2_move_target", "world")
        self.add_output("target_ee_pose", "_target_pose", EncodedState,
                        MessageType.CARTESIAN_POSE_MESSAGE)
        self._current_position_world_mm = []
        self.add_output("current_position_world_mm", "_current_position_world_mm", Float64MultiArray)
        self._current_position_conveyor_mm = []
        self.add_output("current_position_conveyor_mm", "_current_position_conveyor_mm", Float64MultiArray)

        self.add_predicate("is_moving", False)
        self.add_predicate("at_target", False)
        self.add_predicate("has_failed", False)

        # The installed Modulo Python API infers the supported AICA trigger
        # service type from the callback.  Its signature is (name, callback).
        self.add_service("move_to_target", self._on_move_to_target)
        self.add_service("stop_motion", self._on_stop_motion)

        self._state = "IDLE"
        self._state_start_time = None
        self._start_position = None
        self._target_position = None
        self._held_orientation = None
        self._duration_s = 0.0
        self._T_robot_conveyor: Optional[np.ndarray] = None

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._state_start_time = None
        # Load this already on activation so the operator can inspect the
        # conveyor-frame position before issuing the first move command.  A
        # world-frame move is independent of the calibration file.
        frame = str(self.get_parameter("coordinate_frame").get_value()).lower()
        self._T_robot_conveyor = (
            self._load_robot_conveyor_transform() if frame == "conveyor" else None
        )
        self._current_position_world_mm = []
        self._current_position_conveyor_mm = []
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", False)
        self.set_predicate("has_failed", False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        value = parameter.get_value()
        if name == "coordinate_frame" and str(value).lower() not in ("conveyor", "world"):
            self.get_logger().warn("coordinate_frame must be 'conveyor' or 'world'.")
            return False
        if name in ("move_speed_m_s", "max_travel_distance_m") and float(value) <= 0.0:
            self.get_logger().warn(f"{name} must be greater than zero.")
            return False
        return True

    def _read_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Read the measured end-effector pose, normalizing position to metres."""
        position = self._robot_ee_pose.get_position()
        position_m = np.array([float(position[0]), float(position[1]), float(position[2])], dtype=np.float64)
        # Some robot integrations publish millimetres.  A robot workspace is
        # always smaller than 2 m in every base-frame axis here.
        if np.any(np.abs(position_m) > 2.0):
            position_m /= 1000.0
        orientation = self._robot_ee_pose.get_orientation()
        if hasattr(orientation, "to_rotation_matrix"):
            rotation = np.array(orientation.to_rotation_matrix(), dtype=np.float64)
        else:
            qx, qy, qz, qw = (float(orientation.x), float(orientation.y),
                              float(orientation.z), float(orientation.w))
            rotation = np.array([
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ], dtype=np.float64)
        return position_m, rotation_matrix_to_quaternion(rotation)

    def _load_robot_conveyor_transform(self) -> Optional[np.ndarray]:
        """Load and validate the conveyor-to-world homogeneous transform."""
        calibration_path = str(self.get_parameter("calibration_file_path").get_value())
        if not os.path.isfile(calibration_path):
            self.get_logger().error(f"Calibration file not found: {calibration_path}")
            return None
        try:
            with open(calibration_path, "r", encoding="utf-8") as file:
                data = json.load(file)
            matrix = data.get("transformations", {}).get("T_robot_conveyor", {}).get("homogeneous_matrix")
            if matrix is None:
                matrix = data.get("conveyor_frame", {}).get("matrix_4x4", data.get("T_robot_conveyor"))
            transform = np.asarray(matrix, dtype=np.float64)
            if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
                raise ValueError("T_robot_conveyor must be a finite 4x4 matrix")
            return transform
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Could not read T_robot_conveyor: {error}")
            return None

    def _target_world_position(self) -> Optional[np.ndarray]:
        target_mm = np.array([
            float(self.get_parameter("target_x_mm").get_value()),
            float(self.get_parameter("target_y_mm").get_value()),
            float(self.get_parameter("target_z_mm").get_value()),
        ], dtype=np.float64)
        if not np.all(np.isfinite(target_mm)):
            self.get_logger().error("Target coordinates must be finite numbers.")
            return None
        frame = str(self.get_parameter("coordinate_frame").get_value()).lower()
        if frame == "world":
            return target_mm / 1000.0
        self._T_robot_conveyor = self._load_robot_conveyor_transform()
        if self._T_robot_conveyor is None:
            return None
        return (self._T_robot_conveyor @ np.append(target_mm / 1000.0, 1.0))[:3]

    def _fail(self, message: str) -> None:
        self._state = "IDLE"
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", False)
        self.set_predicate("has_failed", True)
        self.get_logger().error(message)

    def _on_move_to_target(self, request: StringTrigger.Request) -> StringTrigger.Response:
        del request
        response = StringTrigger.Response()
        try:
            current_position, current_orientation = self._read_ee_pose()
        except Exception as error:
            self._fail(f"No valid robot_ee_pose received: {error}")
            response.success = False
            response.message = "Keine gültige Robot EE Pose verfügbar."
            return response

        target_position = self._target_world_position()
        if target_position is None:
            self._fail("Target position could not be calculated.")
            response.success = False
            response.message = "Zielposition konnte nicht berechnet werden."
            return response

        distance_m = float(np.linalg.norm(target_position - current_position))
        maximum_m = float(self.get_parameter("max_travel_distance_m").get_value())
        if distance_m > maximum_m:
            self._fail(f"Rejected target: {distance_m:.3f} m exceeds limit {maximum_m:.3f} m.")
            response.success = False
            response.message = "Ziel liegt außerhalb der eingestellten Sicherheitsdistanz."
            return response

        speed_m_s = float(self.get_parameter("move_speed_m_s").get_value())
        self._start_position = current_position
        self._target_position = target_position
        self._held_orientation = current_orientation
        self._duration_s = max(distance_m / speed_m_s, 0.1)
        self._state_start_time = self.get_clock().now()
        self._state = "MOVING"
        self.set_predicate("is_moving", True)
        self.set_predicate("at_target", distance_m < 0.001)
        self.set_predicate("has_failed", False)
        response.success = True
        response.message = f"Fahrt gestartet: {distance_m * 1000.0:.1f} mm."
        return response

    def _on_stop_motion(self, request: StringTrigger.Request) -> StringTrigger.Response:
        del request
        response = StringTrigger.Response()
        try:
            position, orientation = self._read_ee_pose()
            self._target_pose.set_position(position)
            self._target_pose.set_orientation(orientation)
        except Exception as error:
            self._fail(f"Stop failed because robot pose is unavailable: {error}")
            response.success = False
            response.message = "Stop nicht möglich: keine gültige Robot EE Pose."
            return response
        self._state = "IDLE"
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", True)
        response.success = True
        response.message = "Zielpose auf aktuelle Roboterposition gehalten."
        return response

    def on_step_callback(self) -> None:
        try:
            current_position, current_orientation = self._read_ee_pose()
        except Exception:
            return

        self._current_position_world_mm = (current_position * 1000.0).tolist()
        if self._T_robot_conveyor is not None:
            conveyor_position = np.linalg.inv(self._T_robot_conveyor) @ np.append(current_position, 1.0)
            self._current_position_conveyor_mm = (conveyor_position[:3] * 1000.0).tolist()
        else:
            self._current_position_conveyor_mm = []

        if self._state == "IDLE":
            # Holding the current measured pose avoids publishing an uninitialized
            # target to the Point Attractor before the first service call.
            self._target_pose.set_position(current_position)
            self._target_pose.set_orientation(current_orientation)
            return

        if self._state == "HOLDING_TARGET":
            # Keep the final target published.  The Point Attractor can then
            # finish converging even after this generator's trajectory ends.
            self._target_pose.set_position(self._target_position)
            self._target_pose.set_orientation(self._held_orientation)
            return

        elapsed_s = (self.get_clock().now() - self._state_start_time).nanoseconds / 1e9
        progress = min(max(elapsed_s / self._duration_s, 0.0), 1.0)
        smooth_progress = 0.5 * (1.0 - math.cos(math.pi * progress))
        commanded_position = self._start_position + smooth_progress * (self._target_position - self._start_position)
        self._target_pose.set_position(commanded_position)
        self._target_pose.set_orientation(self._held_orientation)

        if progress >= 1.0:
            self._state = "HOLDING_TARGET"
            self.set_predicate("is_moving", False)
            self.set_predicate("at_target", True)
            self.get_logger().info("Target trajectory completed.")
