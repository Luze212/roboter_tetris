"""Manual Cartesian target generator for robot 2 in AICA.

Connect ``target_ee_pose`` to the ``attractor_pose`` input of a Signal Point
Attractor.  The component publishes a smooth Cartesian target trajectory; the
Point Attractor and the robot controller execute the physical motion.
"""

import json
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import state_representation as sr
from clproto import MessageType
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core.encoded_state import EncodedState
from std_msgs.msg import Float64MultiArray


# Conveyor-frame definition in the robot base/world frame.  The origin is the
# middle of the conveyor start.  Its axes are parallel to world, but X and Y
# point in the negative world directions:
#   p_world = CONVEYOR_ORIGIN_WORLD_MM + AXIS_SIGNS * p_conveyor
# where * is element-wise multiplication.
CONVEYOR_ORIGIN_WORLD_MM = np.array([-880.0, 1140.0, 305.0], dtype=np.float64)
CONVEYOR_TO_WORLD_AXIS_SIGNS = np.array([-1.0, -1.0, 1.0], dtype=np.float64)


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

    Target coordinates are millimetres.  They are interpreted directly in the
    robot base ``world`` frame by default, or in the translated ``conveyor``
    frame when ``coordinate_frame`` is set to ``conveyor``.  The current EE
    orientation is retained for every move.
    """

    def __init__(self, node_name: str, *args, **kwargs) -> None:
        super().__init__(node_name, *args, **kwargs)

        self.add_parameter(
            sr.Parameter("target_x_mm", 0.0, sr.ParameterType.DOUBLE),
            "Ziel-X in mm. Bei conveyor: +X ist nach rechts in Förderrichtung und entspricht -X in world.")
        self.add_parameter(
            sr.Parameter("target_y_mm", 0.0, sr.ParameterType.DOUBLE),
            "Ziel-Y in mm. Bei conveyor: +Y ist die Förderrichtung und entspricht -Y in world.")
        self.add_parameter(
            sr.Parameter("target_z_mm", 200.0, sr.ParameterType.DOUBLE),
            "Ziel-Z in mm. Bei conveyor: +Z ist über der Förderbandebene und entspricht +Z in world.")
        self.add_parameter(sr.Parameter("coordinate_frame", "world", sr.ParameterType.STRING),
                           "'world': Roboterbasis-Koordinaten. 'conveyor': Ursprung [-880, 1140, 305] mm in world, X/Y invertiert.")
        self.add_parameter(
            sr.Parameter("workspace_limits_file_path", "", sr.ParameterType.STRING),
            "Optionale JSON-Datei mit zulässigen world-Arbeitsraumregionen. Leer verwendet die mitgelieferte workspace_limits.json.")
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
        self.add_predicate("has_valid_workspace", False)

        # The installed Modulo Python API expects (service_name, callback).
        # These services deliberately use AICA's default trigger type with no
        # payload; all target values are component parameters.
        self.add_service("move_to_target", self._on_move_to_target)
        self.add_service("stop_motion", self._on_stop_motion)

        self._state = "IDLE"
        self._state_start_time = None
        self._start_position = None
        self._target_position = None
        self._held_orientation = None
        self._duration_s = 0.0
        self._workspace_regions: List[Tuple[str, np.ndarray, np.ndarray]] = []

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self._state = "IDLE"
        self._state_start_time = None
        self._current_position_world_mm = []
        self._current_position_conveyor_mm = []
        workspace_is_valid = self._refresh_workspace()
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", False)
        self.set_predicate("has_failed", not workspace_is_valid)
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

    def _target_world_position(self) -> np.ndarray:
        target_mm = np.array([
            float(self.get_parameter("target_x_mm").get_value()),
            float(self.get_parameter("target_y_mm").get_value()),
            float(self.get_parameter("target_z_mm").get_value()),
        ], dtype=np.float64)
        if not np.all(np.isfinite(target_mm)):
            raise ValueError("Target coordinates must be finite numbers.")
        frame = str(self.get_parameter("coordinate_frame").get_value()).lower()
        if frame == "world":
            return target_mm / 1000.0
        if frame == "conveyor":
            return (
                CONVEYOR_ORIGIN_WORLD_MM
                + CONVEYOR_TO_WORLD_AXIS_SIGNS * target_mm
            ) / 1000.0
        raise ValueError("coordinate_frame must be 'world' or 'conveyor'.")

    def _workspace_limits_path(self) -> Path:
        configured_path = str(self.get_parameter("workspace_limits_file_path").get_value()).strip()
        if configured_path:
            return Path(configured_path)
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory("roboter_tetris")) / "robot_2_move" / "workspace_limits.json"
        except (ImportError, ValueError):
            # Useful for source-tree execution and unit tests outside a ROS install.
            return Path(__file__).with_name("workspace_limits.json")

    def _load_workspace_regions(self) -> List[Tuple[str, np.ndarray, np.ndarray]]:
        """Load finite inclusive world-frame AABBs from the JSON configuration."""
        limits_path = self._workspace_limits_path()
        try:
            with limits_path.open("r", encoding="utf-8") as file:
                data = json.load(file)
            if data.get("frame") != "world" or data.get("unit") != "m":
                raise ValueError("workspace limits must use frame='world' and unit='m'.")
            if data.get("inclusive_bounds") is not True:
                raise ValueError("workspace limits must declare inclusive_bounds=true.")
            regions = []
            for index, region in enumerate(data["regions"]):
                lower = np.array([region["x"][0], region["y"][0], region["z"][0]], dtype=np.float64)
                upper = np.array([region["x"][1], region["y"][1], region["z"][1]], dtype=np.float64)
                if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower > upper):
                    raise ValueError(f"region {index} has invalid bounds.")
                regions.append((str(region.get("name", f"region_{index}")), lower, upper))
            if not regions:
                raise ValueError("workspace limits contain no regions.")
            self.get_logger().info(f"Loaded {len(regions)} workspace region(s) from {limits_path}.")
            return regions
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Could not load workspace limits from {limits_path}: {error}")
            return []

    def _refresh_workspace(self) -> bool:
        """Reload the configured workspace, including runtime parameter changes."""
        self._workspace_regions = self._load_workspace_regions()
        is_valid = bool(self._workspace_regions)
        self.set_predicate("has_valid_workspace", is_valid)
        return is_valid

    def _workspace_region_for(self, position_world_m: np.ndarray) -> Optional[str]:
        """Return the region containing a world position, or ``None`` if unsafe."""
        tolerance_m = 1e-9
        for name, lower, upper in self._workspace_regions:
            if np.all(position_world_m >= lower - tolerance_m) and np.all(position_world_m <= upper + tolerance_m):
                return name
        return None

    def _validate_world_target(self, start_world_m: np.ndarray, target_world_m: np.ndarray) -> Optional[str]:
        """Check a target and the complete straight target path against the workspace.

        This is the shared validation point for manual targets and future
        coordinates received from an input signal.
        """
        if not self._workspace_regions:
            return "Keine gültige Arbeitsraum-Konfiguration geladen."
        distance_m = float(np.linalg.norm(target_world_m - start_world_m))
        sample_count = max(1, int(math.ceil(distance_m / 0.001)))
        for step in range(sample_count + 1):
            progress = step / sample_count
            position = start_world_m + progress * (target_world_m - start_world_m)
            if self._workspace_region_for(position) is None:
                return (
                    "Ziel oder gerade Zieltrajektorie liegt außerhalb des zulässigen Arbeitsraums "
                    f"(bei {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f} m)."
                )
        return None

    def _fail(self, message: str) -> None:
        self._state = "IDLE"
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", False)
        self.set_predicate("has_failed", True)
        self.get_logger().error(message)

    def _on_move_to_target(self) -> dict:
        """Start an empty AICA trigger service using the current parameters."""
        if not self._refresh_workspace():
            self._fail("Keine gültige Arbeitsraum-Konfiguration geladen.")
            return {"success": False, "message": "Keine gültige Arbeitsraum-Konfiguration geladen."}
        try:
            current_position, current_orientation = self._read_ee_pose()
        except Exception as error:
            self._fail(f"No valid robot_ee_pose received: {error}")
            return {"success": False, "message": "Keine gültige Robot EE Pose verfügbar."}

        try:
            target_position = self._target_world_position()
        except ValueError as error:
            self._fail(str(error))
            return {"success": False, "message": "Ungültige Zielkoordinaten oder Coordinate Frame."}
        workspace_error = self._validate_world_target(current_position, target_position)
        if workspace_error is not None:
            self._fail(workspace_error)
            return {"success": False, "message": "Ziel liegt außerhalb des zulässigen Arbeitsraums."}
        distance_m = float(np.linalg.norm(target_position - current_position))
        maximum_m = float(self.get_parameter("max_travel_distance_m").get_value())
        if distance_m > maximum_m:
            self._fail(f"Rejected target: {distance_m:.3f} m exceeds limit {maximum_m:.3f} m.")
            return {"success": False, "message": "Ziel liegt außerhalb der eingestellten Sicherheitsdistanz."}

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
        return {"success": True, "message": f"Fahrt gestartet: {distance_m * 1000.0:.1f} mm."}

    def _on_stop_motion(self) -> dict:
        """Stop with an empty AICA trigger service by holding the current pose."""
        try:
            position, orientation = self._read_ee_pose()
            self._target_pose.set_position(position)
            self._target_pose.set_orientation(orientation)
        except Exception as error:
            self._fail(f"Stop failed because robot pose is unavailable: {error}")
            return {"success": False, "message": "Stop nicht möglich: keine gültige Robot EE Pose."}
        self._state = "IDLE"
        self.set_predicate("is_moving", False)
        self.set_predicate("at_target", True)
        return {"success": True, "message": "Zielpose auf aktuelle Roboterposition gehalten."}

    def on_step_callback(self) -> None:
        try:
            current_position, current_orientation = self._read_ee_pose()
        except Exception:
            return

        self._current_position_world_mm = (current_position * 1000.0).tolist()
        self._current_position_conveyor_mm = (
            CONVEYOR_TO_WORLD_AXIS_SIGNS
            * (current_position * 1000.0 - CONVEYOR_ORIGIN_WORLD_MM)
        ).tolist()

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
        if self._workspace_region_for(commanded_position) is None:
            self._fail("Erzeugte Zieltrajektorie hat den zulässigen Arbeitsraum verlassen.")
            return
        self._target_pose.set_position(commanded_position)
        self._target_pose.set_orientation(self._held_orientation)

        if progress >= 1.0:
            self._state = "HOLDING_TARGET"
            self.set_predicate("is_moving", False)
            self.set_predicate("at_target", True)
            self.get_logger().info("Target trajectory completed.")
