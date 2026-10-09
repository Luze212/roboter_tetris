"""One operator-controlled calibration session with configurable stationary board poses.

Each board pose is solved independently. The robot returns to its initial pose
and waits for an explicit service call before another orbit can begin.
"""

import math
import os
import shutil

import numpy as np

from ..basecam_extrinsics import ROBOT_CAM_MULTI_ARCHIVE_DIR, archive_calibration_file
import state_representation as sr
from std_msgs.msg import Int32

from ..basecam_extrinsics import (
    DEFAULT_CALIBRATION_FILE, pose_from_quaternion, resolve_calibration_path,
)
from .calibration_component import (
    RobotCamHandEyeCalibration, build_zero_yaw_tilt, quaternion_slerp,
)
from .handeye_solver import (
    matrix_to_pose, pose_to_matrix, rotation_matrix_to_quaternion,
    save_handeye_calibration_json, solve_robot_cam_handeye,
)
from .multi_board_solver import combine_board_results, mean_transforms, rotation_angle_deg


class RobotCamHandEyeThreeBoardPositions(RobotCamHandEyeCalibration):
    """Three independent fixed-board solves followed by one guarded average."""

    DEFAULT_RESULT_FILE = "/data/robot_cam_handeye_3positions_calibration.json"
    START_SERVICE_NAME = "start_three_board_calibration"
    AUTO_START_ON_ACTIVATE = False
    DEFAULT_BOARD_COUNT = 3
    MAX_BOARD_COUNT = 6
    ROBOT_STATE_MAX_AGE_NS = 500_000_000
    BOARD_OBSERVATION_MAX_AGE_NS = 1_000_000_000

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        parameters = (
            ("board_position_count", self.DEFAULT_BOARD_COUNT, sr.ParameterType.INT,
             "Anzahl der nacheinander umgesetzten Board-Lagen in dieser Sitzung."),
            ("auto_approach_enabled", True, sr.ParameterType.BOOL,
             "Automatische Anfahrt zur neuen Board-Lage nach der ersten gelösten Lage."),
            ("auto_approach_clearance_mm", 100.0, sr.ParameterType.DOUBLE,
             "Zusätzlicher Abstand der Annäherungspose vor der späteren Sichtpose."),
            ("calibration_ws_x_min", -0.900, sr.ParameterType.DOUBLE,
             "Untergrenze des Kalibrier-Arbeitsraums in world."),
            ("calibration_ws_x_max", -0.500, sr.ParameterType.DOUBLE,
             "Obergrenze des Kalibrier-Arbeitsraums in world."),
            ("calibration_ws_y_min", 0.500, sr.ParameterType.DOUBLE,
             "Untergrenze des Kalibrier-Arbeitsraums in world."),
            ("calibration_ws_y_max", 0.760, sr.ParameterType.DOUBLE,
             "Obergrenze des Kalibrier-Arbeitsraums in world."),
            ("calibration_ws_z_min", 0.310, sr.ParameterType.DOUBLE,
             "Untergrenze des Kalibrier-Arbeitsraums in world."),
            ("calibration_ws_z_max", 0.570, sr.ParameterType.DOUBLE,
             "Obergrenze des Kalibrier-Arbeitsraums in world."),
            ("base_cam_samples_per_board_position", 10, sr.ParameterType.INT,
             "Neue Basiskamera-Beobachtungen, die an der Startpose pro Board-Lage gemittelt werden."),
            ("base_cam_sampling_timeout_s", 5.0, sr.ParameterType.DOUBLE,
             "Maximale Wartezeit auf die Basiskamera-Messreihe an der Startpose."),
            ("max_board_motion_mm", 10.0, sr.ParameterType.DOUBLE,
             "Maximale Positionsstreuung des stillliegenden Boards innerhalb einer Lage."),
            ("max_board_motion_deg", 3.0, sr.ParameterType.DOUBLE,
             "Maximale Orientierungsstreuung des stillliegenden Boards innerhalb einer Lage."),
            ("min_board_change_mm", 30.0, sr.ParameterType.DOUBLE,
             "Mindestversatz des Boards zur vorigen Lage in der statischen Basiskamera."),
            ("max_camera_spread_mm", 20.0, sr.ParameterType.DOUBLE,
             "Maximaler Abstand der drei Kamera-Ergebnisse untereinander."),
            ("max_camera_spread_deg", 1.5, sr.ParameterType.DOUBLE,
             "Maximaler Winkel der drei Kamera-Ergebnisse untereinander."),
        )
        for name, default, kind, description in parameters:
            self.add_parameter(sr.Parameter(name, default, kind), description)
        self.add_predicate("waiting_for_board", False)
        self.add_predicate("board_position_ready", False)
        self.add_predicate("base_cam_sampling_active", False)
        self.add_predicate("auto_approach_active", False)
        self.add_predicate("robot_camera_board_reacquired", False)
        for index in range(1, self.MAX_BOARD_COUNT + 1):
            self.add_predicate(f"board_position_{index}_complete", False)
        self._current_board_position = 0
        self.add_output("current_board_position", "_current_board_position", Int32)
        self.add_service("continue_after_board_move", self._continue_service)
        self._position_index = 0
        self._board_count = self.DEFAULT_BOARD_COUNT
        self._position_results = []
        self._position_details = []
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = None
        self._board_geometry_reference = None
        self._pause_robot_observation_id = -1
        self._pause_base_observation_id = -1
        self._home_settled_since_ns = None
        self._base_cam_capture_after_id = -1
        self._base_cam_capture_started_ns = None
        self._base_cam_capture_samples = []
        self._base_cam_board_pose_for_position = None
        self._base_cam_board_motion_mm = None
        self._base_cam_board_motion_deg = None
        self._provisional_world_T_base_cam = None
        self._provisional_flange_T_robot_cam = None
        self._reference_robot_cam_T_board = None
        self._auto_approach_view_target = None

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        own = {
            "board_position_count",
            "auto_approach_enabled", "auto_approach_clearance_mm",
            "calibration_ws_x_min", "calibration_ws_x_max",
            "calibration_ws_y_min", "calibration_ws_y_max",
            "calibration_ws_z_min", "calibration_ws_z_max",
            "base_cam_samples_per_board_position", "base_cam_sampling_timeout_s",
            "max_board_motion_mm",
            "max_board_motion_deg", "min_board_change_mm",
            "max_camera_spread_mm", "max_camera_spread_deg",
        }
        if name not in own:
            return True
        if parameter.is_empty():
            return False
        if name == "auto_approach_enabled":
            return isinstance(parameter.get_value(), bool)
        try:
            value = float(parameter.get_value())
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value):
            return False
        if name.startswith("calibration_ws_"):
            # World coordinates may be negative (the selected x range is).
            # Ordering of min/max is checked by _workspace_bounds at start.
            return True
        if value <= 0:
            return False
        if name == "board_position_count":
            return value.is_integer() and 1 <= value <= self.MAX_BOARD_COUNT
        if name == "base_cam_samples_per_board_position":
            return value.is_integer()
        return True

    def _reset_position_predicates(self) -> None:
        for index in range(1, self.MAX_BOARD_COUNT + 1):
            self.set_predicate(f"board_position_{index}_complete", False)

    def _reset_position_progress(self) -> None:
        self.set_predicate("board_position_ready", False)
        self.set_predicate("base_cam_sampling_active", False)
        self.set_predicate("auto_approach_active", False)
        self.set_predicate("robot_camera_board_reacquired", False)

    def _mark_position_complete(self, index: int) -> None:
        if 1 <= index <= self.MAX_BOARD_COUNT:
            self.set_predicate(f"board_position_{index}_complete", True)

    def on_activate_callback(self) -> bool:
        # A reloaded instance must not republish the target of its previous run.
        self._target_pose = sr.CartesianPose("calibration_target", "world")
        self._position_index = 0
        self._board_count = self.DEFAULT_BOARD_COUNT
        self._position_results.clear()
        self._position_details.clear()
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = None
        self._board_geometry_reference = None
        self._home_settled_since_ns = None
        self._reset_base_cam_capture()
        self._provisional_world_T_base_cam = None
        self._provisional_flange_T_robot_cam = None
        self._reference_robot_cam_T_board = None
        self._auto_approach_view_target = None
        self.set_predicate("waiting_for_board", False)
        self._reset_position_predicates()
        self._reset_position_progress()
        self._current_board_position = 0
        activated = super().on_activate_callback()
        self.get_logger().info(
            "Drei-Board-Kalibrierung bereit. Start ausschließlich über "
            "start_three_board_calibration; keine Bewegung beim Aktivieren."
        )
        return activated

    def on_deactivate_callback(self) -> bool:
        self._state = "IDLE"
        self.set_predicate("waiting_for_board", False)
        self._reset_position_progress()
        self._current_board_position = 0
        self.set_predicate("is_running", False)
        return super().on_deactivate_callback()

    def _fresh_flange(self) -> np.ndarray:
        stamp = self._last_robot_state_time_ns
        now = self.get_clock().now().nanoseconds
        if stamp is None or now - stamp > self.ROBOT_STATE_MAX_AGE_NS or now < stamp:
            raise ValueError("Flanschzustand fehlt oder ist älter als 0,5 s")
        if self._robot_ee_pose.is_empty() or self._robot_ee_pose.get_reference_frame() != "world":
            raise ValueError("Erwartet wird eine gültige Flanschpose in world")
        if self._robot_ee_pose.get_name() != "ur_tool0":
            raise ValueError("Erwartet wird der Flansch ur_tool0, keine TCP-Pose")
        try:
            pose = pose_from_quaternion(
                self._robot_ee_pose.get_position(),
                self._robot_ee_pose.get_orientation_coefficients(),
            )
        except (TypeError, ValueError, AttributeError) as exc:
            raise ValueError(f"Flanschpose ungültig: {exc}") from exc
        mean_transforms([pose])  # also checks finiteness and rigid rotation
        if np.linalg.norm(pose[:3, 3]) < 0.05 or np.linalg.norm(pose[:3, 3]) > 2.0:
            raise ValueError("Flanschposition außerhalb des plausiblen Roboterbereichs")
        return pose

    @staticmethod
    def _board_vector(message) -> list:
        values = message.data if hasattr(message, "data") else message
        try:
            pose = np.asarray(values, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise ValueError("Board-Pose ist nicht numerisch") from exc
        if len(pose) < 6 or not np.all(np.isfinite(pose[:6])) or np.linalg.norm(pose[:3]) < 0.1:
            raise ValueError("Board-Pose fehlt oder ist ungültig")
        return pose[:6].tolist()

    def _new_board_observations(self, robot_after: int, base_after: int) -> np.ndarray:
        now = self.get_clock().now().nanoseconds
        for label, stamp in (("Robot-Kamera", self._last_robot_cam_board_ns),
                             ("Basiskamera", self._last_base_cam_board_ns)):
            if stamp is None or now < stamp or now - stamp > self.BOARD_OBSERVATION_MAX_AGE_NS:
                raise ValueError(f"{label}: keine frische Board-Beobachtung")
        if (self._observation_id(self._robot_cam_observation_id) <= robot_after
                or self._observation_id(self._base_cam_observation_id) <= base_after):
            raise ValueError("Beide Kameras müssen das Board nach dem Halt neu erkannt haben")
        self._board_vector(self._robot_cam_board_pose_msg)
        base_pose = self._board_vector(self._base_cam_board_pose_msg)
        return pose_to_matrix(np.asarray(base_pose[:3]), np.asarray(base_pose[3:6]))

    def _new_base_cam_observation(self, base_after: int) -> np.ndarray:
        now = self.get_clock().now().nanoseconds
        stamp = self._last_base_cam_board_ns
        if stamp is None or now < stamp or now - stamp > self.BOARD_OBSERVATION_MAX_AGE_NS:
            raise ValueError("Basiskamera: keine frische Board-Beobachtung")
        if self._observation_id(self._base_cam_observation_id) <= base_after:
            raise ValueError("Basiskamera muss das Board nach dem Halt neu erkannt haben")
        base_pose = self._board_vector(self._base_cam_board_pose_msg)
        return pose_to_matrix(np.asarray(base_pose[:3]), np.asarray(base_pose[3:6]))

    def _workspace_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lower = np.array([
            float(self.get_parameter("calibration_ws_x_min").get_value()),
            float(self.get_parameter("calibration_ws_y_min").get_value()),
            float(self.get_parameter("calibration_ws_z_min").get_value()),
        ], dtype=np.float64)
        upper = np.array([
            float(self.get_parameter("calibration_ws_x_max").get_value()),
            float(self.get_parameter("calibration_ws_y_max").get_value()),
            float(self.get_parameter("calibration_ws_z_max").get_value()),
        ], dtype=np.float64)
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
            raise ValueError("Kalibrier-Arbeitsraum ist ungültig")
        return lower, upper

    def _require_inside_workspace(self, label: str, transform: np.ndarray) -> None:
        lower, upper = self._workspace_bounds()
        position = np.asarray(transform[:3, 3], dtype=np.float64)
        if not np.all(np.isfinite(position)) or np.any(position < lower) or np.any(position > upper):
            raise ValueError(
                f"{label} außerhalb des Kalibrier-Arbeitsraums: "
                f"X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f} m"
            )

    def _begin_transform_move(self, target: np.ndarray, state: str) -> None:
        current = self._fresh_flange()
        self._require_inside_workspace("Aktuelle Flanschpose", current)
        self._require_inside_workspace("Automatisches Ziel", target)
        self._moving_start_pos = current[:3, 3].copy()
        self._moving_start_quat = rotation_matrix_to_quaternion(current[:3, :3])
        self._moving_target_pos = np.asarray(target[:3, 3], dtype=np.float64).copy()
        self._moving_target_quat = rotation_matrix_to_quaternion(target[:3, :3])
        self._moving_duration = max(1.5, float(np.linalg.norm(
            self._moving_target_pos - self._moving_start_pos
        )) / 0.04)
        self._state_start_time = None
        self._settled_since_time = None
        self._state = state
        self._target_pose.set_position(self._moving_start_pos)
        self._target_pose.set_orientation(self._moving_start_quat)

    def _orbit_distance_m(self) -> float:
        try:
            values = (self._robot_cam_board_pose_msg.data
                      if hasattr(self._robot_cam_board_pose_msg, "data")
                      else self._robot_cam_board_pose_msg)
            if len(values) >= 6 and float(values[2]) > 0.1:
                return float(values[2])
        except (TypeError, ValueError):
            pass
        if self._reference_robot_cam_T_board is not None:
            return max(0.1, float(self._reference_robot_cam_T_board[2, 3]))
        return 0.35

    def _validate_orbit_workspace(self, home: np.ndarray, distance_m: float) -> None:
        """Reject a recognized board position before any of its orbit targets are sent."""
        count = max(4, int(self.get_parameter("num_waypoints").get_value()))
        radius = float(self.get_parameter("max_orbit_radius_mm").get_value()) / 1000.0
        if not math.isfinite(distance_m) or distance_m < 0.1 or radius <= 0.0:
            raise ValueError("Ungültiger Sichtabstand oder Orbit-Radius")
        self._require_inside_workspace("Orbit-Mittelpunkt", home)
        for index in range(count - 1):
            angle = 2.0 * math.pi * index / (count - 1)
            dz = distance_m - math.sqrt(max(distance_m * distance_m - radius * radius, 0.001))
            offset = np.array([
                radius * math.cos(angle), radius * math.sin(angle), dz,
            ], dtype=np.float64)
            target = np.eye(4, dtype=np.float64)
            target[:3, :3] = home[:3, :3] @ build_zero_yaw_tilt(radius, distance_m, angle)
            target[:3, 3] = home[:3, 3] + home[:3, :3] @ offset
            self._require_inside_workspace(f"Orbit-Wegpunkt {index + 2}/{count}", target)

    def _send_next_waypoint(self):
        """Keep every automatically enabled orbit waypoint inside the calibration box."""
        super()._send_next_waypoint()
        if not bool(self.get_parameter("auto_approach_enabled").get_value()):
            return
        try:
            target = pose_from_quaternion(self._moving_target_pos, self._moving_target_quat)
            self._require_inside_workspace("Orbit-Wegpunkt", target)
        except (AttributeError, TypeError, ValueError) as exc:
            self._fail_calibration(f"Orbit außerhalb des Kalibrier-Arbeitsraums: {exc}")

    def _begin_auto_approach(self) -> None:
        if (self._provisional_world_T_base_cam is None
                or self._reference_robot_cam_T_board is None
                or self._base_cam_board_pose_for_position is None
                or self._previous_board_in_base_cam is None
                or self._first_world_T_flange is None):
            raise ValueError("Vorläufige Kamerakalibrierung aus Board-Lage 1 fehlt")
        T_base_cam_board = pose_to_matrix(
            np.asarray(self._base_cam_board_pose_for_position[:3]),
            np.asarray(self._base_cam_board_pose_for_position[3:6]),
        )

        # The static camera observes board coordinates in its own frame.  Only
        # transfer the measured board displacement in the horizontal world
        # plane to the known-safe calibration start pose.  This is deliberately
        # a coarse approach: height and orientation stay unchanged, and the
        # robot camera must reacquire the board before the orbit begins.
        delta_base_cam = (
            T_base_cam_board[:3, 3] - self._previous_board_in_base_cam[:3, 3]
        )
        delta_world = self._provisional_world_T_base_cam[:3, :3] @ delta_base_cam
        T_world_flange_view = self._first_world_T_flange.copy()
        T_world_flange_view[:2, 3] += delta_world[:2]
        T_world_flange_view[2, 3] = self._first_world_T_flange[2, 3]
        mean_transforms([T_world_flange_view])
        self._require_inside_workspace("XY-Nachführpose", T_world_flange_view)
        self._validate_orbit_workspace(
            T_world_flange_view,
            max(0.1, float(self._reference_robot_cam_T_board[2, 3])),
        )
        self._auto_approach_view_target = T_world_flange_view
        self.set_predicate("board_position_ready", True)
        self.set_predicate("auto_approach_active", True)
        self._begin_transform_move(T_world_flange_view, "AUTO_APPROACH_VIEW")
        self.get_logger().info(
            "Neue Board-Lage erkannt; XY-Nachführung zur Sichtpose startet "
            f"(ΔX={delta_world[0] * 1000.0:.1f} mm, ΔY={delta_world[1] * 1000.0:.1f} mm, "
            f"Z unverändert {T_world_flange_view[2, 3]:.3f} m)."
        )

    def _step_auto_approach(self) -> None:
        now = self.get_clock().now()
        if self._state_start_time is None:
            self._state_start_time = now
        elapsed = (now - self._state_start_time).nanoseconds / 1e9
        progress_raw = min(elapsed / max(float(self._moving_duration), 0.1), 1.0)
        progress = 0.5 * (1.0 - math.cos(math.pi * progress_raw))
        position = self._moving_start_pos + progress * (
            self._moving_target_pos - self._moving_start_pos
        )
        orientation = quaternion_slerp(
            self._moving_start_quat, self._moving_target_quat, progress
        )
        self._target_pose.set_position(position)
        self._target_pose.set_orientation(orientation)
        if progress_raw < 1.0:
            return
        if self._state == "AUTO_APPROACH_PRE":
            self._begin_transform_move(self._auto_approach_view_target, "AUTO_APPROACH_VIEW")
            return
        self.set_predicate("auto_approach_active", False)
        self._state = "IDLE"
        super()._on_start_calibration()
        self.get_logger().info(
            "Sichtpose erreicht; Roboterkamera muss das Board jetzt für den Orbit erkennen."
        )

    def _reset_base_cam_capture(self) -> None:
        self._base_cam_capture_after_id = -1
        self._base_cam_capture_started_ns = None
        self._base_cam_capture_samples = []
        self._base_cam_board_pose_for_position = None
        self._base_cam_board_motion_mm = None
        self._base_cam_board_motion_deg = None

    def _start_base_cam_capture(self) -> None:
        """Capture a stable static-camera board pose before the robot moves."""
        self._reset_base_cam_capture()
        self._base_cam_capture_after_id = self._observation_id(self._base_cam_observation_id)
        self._base_cam_capture_started_ns = self.get_clock().now().nanoseconds
        self._state = "BASE_CAM_SAMPLING"
        self.set_predicate("is_running", True)
        self.set_predicate("is_calibrated", False)
        self.set_predicate("has_failed", False)

    def _finish_base_cam_capture(self) -> None:
        board_poses = list(self._base_cam_capture_samples)
        board_mean = mean_transforms(board_poses)
        position_motion_mm = max(
            float(np.linalg.norm(pose[:3, 3] - board_mean[:3, 3]) * 1000.0)
            for pose in board_poses
        )
        rotation_motion_deg = max(
            rotation_angle_deg(board_mean[:3, :3].T @ pose[:3, :3])
            for pose in board_poses
        )
        if (position_motion_mm > float(self.get_parameter("max_board_motion_mm").get_value())
                or rotation_motion_deg > float(self.get_parameter("max_board_motion_deg").get_value())):
            raise ValueError(
                f"Board an Startpose nicht stabil: {position_motion_mm:.2f} mm / "
                f"{rotation_motion_deg:.3f} Grad"
            )
        if self._previous_board_in_base_cam is not None:
            move_mm = float(np.linalg.norm(
                board_mean[:3, 3] - self._previous_board_in_base_cam[:3, 3]
            ) * 1000.0)
            turn_deg = rotation_angle_deg(
                self._previous_board_in_base_cam[:3, :3].T @ board_mean[:3, :3]
            )
            if move_mm < float(self.get_parameter("min_board_change_mm").get_value()):
                raise ValueError(
                    f"Board-Lage kaum verändert ({move_mm:.1f} mm / {turn_deg:.2f} Grad)"
                )
        translation, rotation = matrix_to_pose(board_mean)
        self._base_cam_board_pose_for_position = np.r_[translation, rotation].tolist()
        self._base_cam_board_motion_mm = position_motion_mm
        self._base_cam_board_motion_deg = rotation_motion_deg

    def _step_base_cam_capture(self) -> None:
        phase = "Basiskamera-Messreihe"
        try:
            self._fresh_flange()
            now = self.get_clock().now().nanoseconds
            if self._base_cam_capture_started_ns is None:
                raise ValueError("Zeitstempel der Basiskamera-Messreihe fehlt")
            timeout_ns = int(float(self.get_parameter("base_cam_sampling_timeout_s").get_value()) * 1e9)
            if now - self._base_cam_capture_started_ns > timeout_ns:
                raise ValueError(
                    f"Basiskamera lieferte nur {len(self._base_cam_capture_samples)}/"
                    f"{int(self.get_parameter('base_cam_samples_per_board_position').get_value())} "
                    "gültige Board-Beobachtungen an der Startpose"
                )
            observation_id = self._observation_id(self._base_cam_observation_id)
            if observation_id > self._base_cam_capture_after_id:
                self._base_cam_capture_after_id = observation_id
                base_pose = self._board_vector(self._base_cam_board_pose_msg)
                self._base_cam_capture_samples.append(
                    pose_to_matrix(np.asarray(base_pose[:3]), np.asarray(base_pose[3:6]))
                )
            required = int(self.get_parameter("base_cam_samples_per_board_position").get_value())
            if len(self._base_cam_capture_samples) < required:
                return
            self._finish_base_cam_capture()
            if (self._position_index > 0
                    and bool(self.get_parameter("auto_approach_enabled").get_value())):
                phase = "XY-Nachführung"
                self._begin_auto_approach()
                return
            self._state = "IDLE"
            super()._on_start_calibration()
            self.get_logger().info(
                f"Board-Lage {self._position_index + 1}/{self._board_count}: {required} Basiskamera-Aufnahmen "
                "an der Startpose gemittelt; Orbit startet."
            )
        except (TypeError, ValueError) as exc:
            self._fail_calibration(f"{phase} fehlgeschlagen: {exc}")

    def _on_start_calibration_service(self) -> dict:
        if self._state not in ("IDLE", "FINISHED", "FAILED"):
            return {"success": False, "message": f"Sitzung läuft bereits: {self._state}"}
        try:
            flange = self._fresh_flange()
            self._new_board_observations(-1, -1)
            self._check_output_path()
        except (OSError, TypeError, ValueError) as exc:
            return {"success": False, "message": f"Kein Start: {exc}"}
        self._position_index = 0
        self._board_count = int(self.get_parameter("board_position_count").get_value())
        if not 1 <= self._board_count <= self.MAX_BOARD_COUNT:
            return {"success": False,
                    "message": f"board_position_count muss zwischen 1 und {self.MAX_BOARD_COUNT} liegen"}
        self._position_results.clear()
        self._position_details.clear()
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = flange.copy()
        self._board_geometry_reference = None
        self._home_settled_since_ns = None
        self._reset_position_predicates()
        self._start_base_cam_capture()
        return {"success": True,
                "message": f"Board-Lage 1/{self._board_count}: Basiskamera-Messreihe an der Startpose läuft; "
                           "danach startet der Roboter den Orbit."}

    def _check_output_path(self) -> str:
        path = str(self.get_parameter("robot_cam_handeye_file_path").get_value() or "")
        if not os.path.isabs(path) or not path.lower().endswith(".json"):
            raise ValueError("Ergebnisdatei muss ein absoluter .json-Pfad sein")
        forbidden = (
            "/data/robot_cam_handeye_calibration.json",
            "/data/base_cam_fused_extrinsics.json",
        )
        if any(os.path.realpath(path) == os.path.realpath(other) for other in forbidden):
            raise ValueError("Ergebnisdatei darf keine Datei des Einzellaufs oder der Fusion überschreiben")
        if os.path.realpath(path) == os.path.realpath(resolve_calibration_path(DEFAULT_CALIBRATION_FILE)):
            raise ValueError("Ergebnisdatei darf die eigenständige BaseCam-Kalibrierung nicht überschreiben")
        backup = os.path.splitext(os.path.realpath(path))[0] + "_vorher.json"
        if os.path.islink(path) or os.path.islink(backup):
            raise ValueError("Ergebnisdatei und Sicherung dürfen keine symbolischen Links sein")
        return path

    def _continue_service(self) -> dict:
        if self._state != "WAIT_BOARD_REPOSITION":
            return {"success": False, "message": "Fortsetzen ist nur während 'Warte auf Board' möglich."}
        try:
            self._fresh_flange()
            if not self._is_ee_at_target():
                raise ValueError("Roboter steht nicht an der Startpose")
            if bool(self.get_parameter("auto_approach_enabled").get_value()):
                self._new_base_cam_observation(self._pause_base_observation_id)
            else:
                self._new_board_observations(
                    self._pause_robot_observation_id, self._pause_base_observation_id
                )
        except (TypeError, ValueError) as exc:
            return {"success": False, "message": f"Bleibt angehalten: {exc}"}
        self._position_index += 1
        self.set_predicate("waiting_for_board", False)
        self._home_settled_since_ns = None
        self._start_base_cam_capture()
        return {"success": True,
                "message": f"Board-Lage {self._position_index + 1}/{self._board_count}: Basiskamera-Messreihe "
                           "an der Startpose läuft; danach startet der Roboter den Orbit."}

    def _fail_calibration(self, message: str) -> None:
        # Hold the latest measured flange pose when it is trustworthy. A stale
        # hardware state cannot provide a safe new target and needs operator stop.
        try:
            flange = self._fresh_flange()
            self._target_pose.set_position(flange[:3, 3].astype(np.float64))
            self._target_pose.set_orientation(rotation_matrix_to_quaternion(flange[:3, :3]))
        except (AttributeError, TypeError, ValueError):
            pass
        self.set_predicate("waiting_for_board", False)
        super()._fail_calibration(message)

    def on_step_callback(self):
        if self._state in ("IDLE", "WAIT_BOARD_REPOSITION", "FINISHED", "FAILED"):
            return
        if self._state in ("BASE_CAM_SAMPLING", "AUTO_APPROACH_PRE", "AUTO_APPROACH_VIEW",
                           "MOVING", "SETTLING", "SAMPLING", "RETURNING_HOME", "SOLVING"):
            try:
                self._fresh_flange()
            except ValueError as exc:
                self._fail_calibration(f"Drei-Board-Lauf gestoppt: {exc}")
                return
        if self._state == "BASE_CAM_SAMPLING":
            self._step_base_cam_capture()
            return
        if self._state in ("AUTO_APPROACH_PRE", "AUTO_APPROACH_VIEW"):
            try:
                self._step_auto_approach()
            except (AttributeError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
                self._fail_calibration(f"Automatische Anfahrt fehlgeschlagen: {exc}")
            return
        if self._state == "SOLVING":
            now = self.get_clock().now()
            if self._state_start_time is None:
                self._fail_calibration("Zeitstempel der Rückfahrt fehlt")
                return
            if not self._is_ee_at_target():
                self._home_settled_since_ns = None
                elapsed = (now - self._state_start_time).nanoseconds / 1e9
                allowed = (float(self._moving_duration)
                           + float(self.get_parameter("settle_timeout_s").get_value()))
                if elapsed > allowed:
                    self._fail_calibration("Roboter erreichte die Startpose nach dem Orbit nicht")
                return
            if self._home_settled_since_ns is None:
                self._home_settled_since_ns = now.nanoseconds
                return
            if (now.nanoseconds - self._home_settled_since_ns <
                    float(self.get_parameter("settle_time_s").get_value()) * 1e9):
                return
            self._finish_position()
            return
        previous_count = len(self._collected_samples)
        super().on_step_callback()
        if len(self._collected_samples) > previous_count:
            if self._base_cam_board_pose_for_position is None:
                self._fail_calibration("Gemittelte Basiskamera-Pose der Board-Lage fehlt")
                return
            self._collected_samples[-1].base_cam_board_pose = list(
                self._base_cam_board_pose_for_position
            )

    def _finish_position(self) -> None:
        try:
            samples = list(self._collected_samples)
            if len(samples) != len(self._waypoints):
                raise ValueError("Nicht alle Wegpunkte haben gültige Samples")
            geometry_data = (self._board_geometry_msg.data
                             if hasattr(self._board_geometry_msg, "data")
                             else self._board_geometry_msg)
            geometry = np.asarray(geometry_data, dtype=np.float64).reshape(-1)
            if len(geometry) < 3 or not np.all(np.isfinite(geometry[:3])) or np.any(geometry[:3] <= 0):
                raise ValueError("Board-Geometrie fehlt oder ist ungültig")
            if self._board_geometry_reference is None:
                self._board_geometry_reference = geometry[:3].copy()
            elif not np.allclose(geometry[:3], self._board_geometry_reference, atol=1e-6, rtol=0):
                raise ValueError("Board-Geometrie hat sich zwischen den Lagen geändert")
            if self._base_cam_board_pose_for_position is None:
                raise ValueError("Gemittelte Basiskamera-Pose der Board-Lage fehlt")
            board_mean = pose_to_matrix(
                np.asarray(self._base_cam_board_pose_for_position[:3]),
                np.asarray(self._base_cam_board_pose_for_position[3:6]),
            )
            position_motion_mm = float(self._base_cam_board_motion_mm)
            rotation_motion_deg = float(self._base_cam_board_motion_deg)
            result = solve_robot_cam_handeye(samples)
            if result.T_robot_base_static_cam is None:
                raise ValueError("Statische Basiskamera-Pose konnte nicht bestimmt werden")
            max_position = float(self.get_parameter("max_position_rmse_mm").get_value())
            max_rotation = float(self.get_parameter("max_rotation_rmse_deg").get_value())
            if result.position_rmse_mm > max_position or result.rotation_rmse_deg > max_rotation:
                raise ValueError(
                    f"Board-Lage {self._position_index + 1}: RMSE "
                    f"{result.position_rmse_mm:.2f} mm / {result.rotation_rmse_deg:.3f} Grad "
                    f"(Grenzen {max_position:.2f} mm / {max_rotation:.3f} Grad)"
                )
            if self._position_index == 0:
                if self._first_world_T_flange is None or result.T_ee_robot_cam is None:
                    raise ValueError("Referenzpose für die automatische Anfahrt fehlt")
                self._provisional_world_T_base_cam = result.T_robot_base_static_cam.copy()
                self._provisional_flange_T_robot_cam = result.T_ee_robot_cam.copy()
                T_world_robot_cam = self._first_world_T_flange @ result.T_ee_robot_cam
                T_world_board = result.T_robot_base_static_cam @ board_mean
                self._reference_robot_cam_T_board = (
                    np.linalg.inv(T_world_robot_cam) @ T_world_board
                )
                mean_transforms([self._reference_robot_cam_T_board])
                self.get_logger().info(
                    "Vorläufige Kamerakalibrierung aus Board-Lage 1 für die "
                    "automatische Anfahrt der nächsten Lage gespeichert."
                )
            self._position_results.append(result)
            self._position_details.append({
                "position_index": self._position_index + 1,
                "sample_count": result.sample_count,
                "board_position_rmse_mm": round(result.position_rmse_mm, 4),
                "board_rotation_rmse_deg": round(result.rotation_rmse_deg, 5),
                "board_motion_max_mm": round(position_motion_mm, 4),
                "board_motion_max_deg": round(rotation_motion_deg, 5),
                "board_geometry": geometry[:3].tolist(),
                "T_base_cam_board": np.round(board_mean, 8).tolist(),
                "T_flange_robot_cam": np.round(result.T_ee_robot_cam, 8).tolist(),
                "T_world_base_static_cam": np.round(result.T_robot_base_static_cam, 8).tolist(),
            })
            self._previous_board_in_base_cam = board_mean
            self.get_logger().info(
                f"Board-Lage {self._position_index + 1}/{self._board_count} geprüft: "
                f"{result.sample_count} Samples, Board-RMSE "
                f"{result.position_rmse_mm:.2f} mm / {result.rotation_rmse_deg:.3f} Grad"
            )
            self._mark_position_complete(self._position_index + 1)
            if len(self._position_results) < self._board_count:
                self._pause_robot_observation_id = self._observation_id(self._robot_cam_observation_id)
                self._pause_base_observation_id = self._observation_id(self._base_cam_observation_id)
                self._state = "WAIT_BOARD_REPOSITION"
                self.set_predicate("waiting_for_board", True)
                self.get_logger().info(
                    "Roboter an Startpose; Board umsetzen. Danach "
                    "continue_after_board_move aufrufen. Bis dahin keine weitere Bewegung."
                )
                return
            combined, report = combine_board_results(
                self._position_results,
                self._first_world_T_flange,
                self.get_parameter("max_camera_spread_mm").get_value(),
                self.get_parameter("max_camera_spread_deg").get_value(),
            )
            output = self._check_output_path()
            previous = os.path.splitext(os.path.realpath(output))[0] + "_vorher.json"
            if os.path.exists(output):
                if os.path.islink(previous):
                    raise ValueError("Sicherungsdatei ist ein symbolischer Link")
                shutil.copy2(os.path.realpath(output), previous)
            save_handeye_calibration_json(
                output, combined,
                operator="robot_cam_handeye_three_board_positions",
                notes="Three independent stationary ChArUco board positions; no common conveyor pose",
                multi_board={"positions": self._position_details, "consistency": report},
            )
            archive_path = archive_calibration_file(
                output, ROBOT_CAM_MULTI_ARCHIVE_DIR, "robot_cam_handeye_multi_board"
            )
            T = combined.T_robot_base_static_cam
            self._base_cam_pose_out.set_position(T[:3, 3].astype(np.float64))
            self._base_cam_pose_out.set_orientation(rotation_matrix_to_quaternion(T[:3, :3]))
            self._calibration_matrix = combined.T_robot_base_cam.flatten().tolist()
            self.set_predicate("is_calibrated", True)
            self.set_predicate("is_running", False)
            self._state = "FINISHED"
            self.get_logger().info(
                f"DREI-BOARD-KALIBRIERUNG ERFOLGREICH: "
                f"{combined.sample_count} Samples, aktive Datei {output}; Archiv {archive_path}; "
                f"Streuung statische Basiskamera "
                f"{report['world_base_static_cam']['max_pairwise_translation_mm']:.2f} mm / "
                f"{report['world_base_static_cam']['max_pairwise_rotation_deg']:.3f} Grad"
            )
        except Exception as exc:
            self._fail_calibration(f"Drei-Board-Kalibrierung fehlgeschlagen: {exc}")
