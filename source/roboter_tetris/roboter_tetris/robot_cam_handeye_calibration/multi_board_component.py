"""One operator-controlled calibration session with three stationary board poses.

Each board pose is solved independently. The robot returns to its initial pose
and waits for an explicit service call before another orbit can begin.
"""

import math
import os
import shutil

import numpy as np
import state_representation as sr

from ..basecam_extrinsics import (
    DEFAULT_CALIBRATION_FILE, pose_from_quaternion, resolve_calibration_path,
)
from .calibration_component import RobotCamHandEyeCalibration
from .handeye_solver import (
    pose_to_matrix, rotation_matrix_to_quaternion,
    save_handeye_calibration_json, solve_robot_cam_handeye,
)
from .multi_board_solver import combine_board_results, mean_transforms, rotation_angle_deg


class RobotCamHandEyeThreeBoardPositions(RobotCamHandEyeCalibration):
    """Three independent fixed-board solves followed by one guarded average."""

    DEFAULT_RESULT_FILE = "/data/robot_cam_handeye_3positions_calibration.json"
    START_SERVICE_NAME = "start_three_board_calibration"
    AUTO_START_ON_ACTIVATE = False
    BOARD_COUNT = 3
    ROBOT_STATE_MAX_AGE_NS = 500_000_000
    BOARD_OBSERVATION_MAX_AGE_NS = 1_000_000_000

    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        parameters = (
            ("min_base_observations_per_waypoint", 3, sr.ParameterType.INT,
             "Mindestens so viele neue Basiskamera-Beobachtungen je Wegpunkt."),
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
        self.add_service("continue_after_board_move", self._continue_service)
        self._position_index = 0
        self._position_results = []
        self._position_details = []
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = None
        self._board_geometry_reference = None
        self._pause_robot_observation_id = -1
        self._pause_base_observation_id = -1
        self._home_settled_since_ns = None

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        own = {
            "min_base_observations_per_waypoint", "max_board_motion_mm",
            "max_board_motion_deg", "min_board_change_mm",
            "max_camera_spread_mm", "max_camera_spread_deg",
        }
        if name not in own:
            return True
        if parameter.is_empty():
            return False
        try:
            value = float(parameter.get_value())
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value) or value <= 0:
            return False
        if name == "min_base_observations_per_waypoint":
            return value.is_integer()
        return True

    def on_activate_callback(self) -> bool:
        # A reloaded instance must not republish the target of its previous run.
        self._target_pose = sr.CartesianPose("calibration_target", "world")
        self._position_index = 0
        self._position_results.clear()
        self._position_details.clear()
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = None
        self._board_geometry_reference = None
        self._home_settled_since_ns = None
        self.set_predicate("waiting_for_board", False)
        activated = super().on_activate_callback()
        self.get_logger().info(
            "Drei-Board-Kalibrierung bereit. Start ausschließlich über "
            "start_three_board_calibration; keine Bewegung beim Aktivieren."
        )
        return activated

    def on_deactivate_callback(self) -> bool:
        self._state = "IDLE"
        self.set_predicate("waiting_for_board", False)
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
        return pose_to_matrix(base_pose[:3], base_pose[3:6])

    def _on_start_calibration_service(self) -> dict:
        if self._state not in ("IDLE", "FINISHED", "FAILED"):
            return {"success": False, "message": f"Sitzung läuft bereits: {self._state}"}
        try:
            flange = self._fresh_flange()
            self._new_board_observations(-1, -1)
            self._check_output_path()
            if int(self.get_parameter("min_base_observations_per_waypoint").get_value()) > int(
                    self.get_parameter("samples_per_waypoint").get_value()):
                raise ValueError("min_base_observations_per_waypoint darf samples_per_waypoint nicht übersteigen")
        except (OSError, TypeError, ValueError) as exc:
            return {"success": False, "message": f"Kein Start: {exc}"}
        self._position_index = 0
        self._position_results.clear()
        self._position_details.clear()
        self._previous_board_in_base_cam = None
        self._first_world_T_flange = flange.copy()
        self._board_geometry_reference = None
        self._home_settled_since_ns = None
        super()._on_start_calibration()
        return {"success": True, "message": "Board-Lage 1/3 gestartet; der Roboter fährt jetzt den Orbit."}

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
            board = self._new_board_observations(
                self._pause_robot_observation_id, self._pause_base_observation_id
            )
            reference = self._previous_board_in_base_cam
            move_mm = float(np.linalg.norm(board[:3, 3] - reference[:3, 3]) * 1000.0)
            turn_deg = rotation_angle_deg(reference[:3, :3].T @ board[:3, :3])
            if move_mm < float(self.get_parameter("min_board_change_mm").get_value()):
                raise ValueError(
                    f"Board-Lage kaum verändert ({move_mm:.1f} mm / {turn_deg:.2f} Grad)"
                )
        except (TypeError, ValueError) as exc:
            return {"success": False, "message": f"Bleibt angehalten: {exc}"}
        self._position_index += 1
        self.set_predicate("waiting_for_board", False)
        self._home_settled_since_ns = None
        self._state = "IDLE"  # the inherited start method only accepts an idle state
        super()._on_start_calibration()
        return {"success": True,
                "message": f"Board-Lage {self._position_index + 1}/3 gestartet; Roboter bewegt sich."}

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
        if self._state in ("MOVING", "SETTLING", "SAMPLING", "RETURNING_HOME", "SOLVING"):
            try:
                self._fresh_flange()
            except ValueError as exc:
                self._fail_calibration(f"Drei-Board-Lauf gestoppt: {exc}")
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
            minimum = int(self.get_parameter("min_base_observations_per_waypoint").get_value())
            if (self._collected_samples[-1].base_cam_board_pose is None
                    or len(self._sample_buffer_base_cam) < minimum):
                self._fail_calibration(
                    f"Board-Lage {self._position_index + 1}, Wegpunkt "
                    f"{self._current_waypoint_idx}: nur {len(self._sample_buffer_base_cam)} "
                    f"Basiskamera-Beobachtungen, benötigt {minimum}"
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
            board_poses = [pose_to_matrix(sample.base_cam_board_pose[:3],
                                          sample.base_cam_board_pose[3:6])
                           for sample in samples]
            board_mean = mean_transforms(board_poses)
            if self._previous_board_in_base_cam is not None:
                difference_mm = float(np.linalg.norm(
                    board_mean[:3, 3] - self._previous_board_in_base_cam[:3, 3]
                ) * 1000.0)
                if difference_mm < float(self.get_parameter("min_board_change_mm").get_value()):
                    raise ValueError(
                        f"Board-Lage {self._position_index + 1} unterscheidet sich nur "
                        f"um {difference_mm:.1f} mm von der vorigen Lage"
                    )
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
                    f"Board während Lage {self._position_index + 1} nicht stabil: "
                    f"{position_motion_mm:.2f} mm / {rotation_motion_deg:.3f} Grad"
                )
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
                f"Board-Lage {self._position_index + 1}/3 geprüft: "
                f"{result.sample_count} Samples, Board-RMSE "
                f"{result.position_rmse_mm:.2f} mm / {result.rotation_rmse_deg:.3f} Grad"
            )
            if len(self._position_results) < self.BOARD_COUNT:
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
            T = combined.T_robot_base_static_cam
            self._base_cam_pose_out.set_position(T[:3, 3].astype(np.float64))
            self._base_cam_pose_out.set_orientation(rotation_matrix_to_quaternion(T[:3, :3]))
            self._calibration_matrix = combined.T_robot_base_cam.flatten().tolist()
            self.set_predicate("is_calibrated", True)
            self.set_predicate("is_running", False)
            self._state = "FINISHED"
            self.get_logger().info(
                f"DREI-BOARD-KALIBRIERUNG ERFOLGREICH: "
                f"{combined.sample_count} Samples, Ergebnis {output}; "
                f"Streuung statische Basiskamera "
                f"{report['world_base_static_cam']['max_pairwise_translation_mm']:.2f} mm / "
                f"{report['world_base_static_cam']['max_pairwise_rotation_deg']:.3f} Grad"
            )
        except Exception as exc:
            self._fail_calibration(f"Drei-Board-Kalibrierung fehlgeschlagen: {exc}")
