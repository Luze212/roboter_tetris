"""Robot-free checks of the new component's pause and resume contract."""

import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


def _load_component(monkeypatch):
    """Load only the subclass against a tiny lifecycle stand-in, without ROS."""
    state_representation = ModuleType("state_representation")
    state_representation.Parameter = object
    state_representation.CartesianPose = lambda *_: SimpleNamespace(is_empty=lambda: True)
    monkeypatch.setitem(sys.modules, "state_representation", state_representation)

    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Int32 = object
    std_msgs.msg = std_msgs_msg
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)

    base_name = "roboter_tetris.robot_cam_handeye_calibration.calibration_component"
    base_module = ModuleType(base_name)

    class Base:
        def _on_start_calibration(self):
            if self._state in ("IDLE", "FINISHED", "FAILED"):
                self._state = "MOVING"
                self._starts += 1

    base_module.RobotCamHandEyeCalibration = Base
    base_module.build_zero_yaw_tilt = lambda *_: np.eye(3)
    base_module.quaternion_slerp = lambda _start, target, _progress: target
    monkeypatch.setitem(sys.modules, base_name, base_module)
    path = (Path(__file__).resolve().parents[2] / "roboter_tetris" /
            "robot_cam_handeye_calibration" / "multi_board_component.py")
    spec = importlib.util.spec_from_file_location(
        "roboter_tetris.robot_cam_handeye_calibration._state_contract_under_test", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotCamHandEyeThreeBoardPositions


def test_continue_starts_base_camera_capture_before_orbit(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._state = "WAIT_BOARD_REPOSITION"
    component._position_index = 0
    component._board_count = 3
    component._starts = 0
    component._pause_robot_observation_id = 11
    component._pause_base_observation_id = 12
    component._previous_board_in_base_cam = np.eye(4)
    component._fresh_flange = lambda: np.eye(4)
    component._is_ee_at_target = lambda: True
    component.get_parameter = lambda name: SimpleNamespace(
        get_value=lambda: {"auto_approach_enabled": False}[name]
    )
    predicates = []
    component.set_predicate = lambda name, value: predicates.append((name, value))
    component._start_base_cam_capture = lambda: setattr(component, "_state", "BASE_CAM_SAMPLING")

    component._new_board_observations = lambda *_: np.eye(4)
    response = component._continue_service()
    assert response["success"] is True
    assert component._position_index == 1
    assert component._state == "BASE_CAM_SAMPLING"
    assert component._starts == 0


def test_base_camera_capture_averages_single_board_pose(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._previous_board_in_base_cam = None
    component._base_cam_capture_samples = []
    component.get_parameter = lambda name: SimpleNamespace(get_value=lambda: {
        "max_board_motion_mm": 10.0,
        "max_board_motion_deg": 3.0,
        "min_board_change_mm": 30.0,
    }[name])

    first = np.eye(4)
    first[0, 3] = 0.400
    second = np.eye(4)
    second[0, 3] = 0.404
    component._base_cam_capture_samples = [first, second]

    component._finish_base_cam_capture()

    np.testing.assert_allclose(component._base_cam_board_pose_for_position[:3], [0.402, 0.0, 0.0])
    assert component._base_cam_board_motion_mm == pytest.approx(2.0)
    assert component._base_cam_board_motion_deg == pytest.approx(0.0)


def _centering_parameters(**overrides):
    values = {
        "num_waypoints": 9,
        "max_orbit_radius_mm": 50.0,
        "calibration_ws_x_min": -0.9,
        "calibration_ws_x_max": -0.5,
        "calibration_ws_y_min": 0.5,
        "calibration_ws_y_max": 0.76,
        "calibration_ws_z_min": 0.31,
        "calibration_ws_z_max": 0.57,
    }
    values.update(overrides)
    return lambda name: SimpleNamespace(get_value=lambda: values[name])


def test_auto_approach_centers_camera_over_charuco_board(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._provisional_world_T_base_cam = np.eye(4)
    component._provisional_world_T_base_cam[:3, 3] = [-0.75, 0.56, 0.0]
    component._provisional_flange_T_robot_cam = np.eye(4)
    component._first_world_T_flange = np.eye(4)
    component._first_world_T_flange[:3, 3] = [-0.65, 0.65, 0.40]
    component._base_cam_board_pose_for_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    component._board_geometry_reference = np.array([5.0, 7.0, 35.0])
    component._center_view_height_m = None
    component._center_camera_rotation = None
    component.get_parameter = _centering_parameters()
    component.get_logger = lambda: SimpleNamespace(info=lambda *_: None)
    predicates = []
    component.set_predicate = lambda name, value: predicates.append((name, value))
    moves = []
    component._begin_transform_move = lambda target, state: moves.append((target.copy(), state))

    component._begin_auto_approach()

    assert moves[0][1] == "AUTO_APPROACH_VIEW"
    np.testing.assert_allclose(moves[0][0][:3, 3], [-0.6275, 0.6475, 0.40])
    np.testing.assert_allclose(moves[0][0][:3, :3], np.diag([1.0, -1.0, -1.0]))
    np.testing.assert_allclose(component._auto_approach_view_target[:3, 3],
                               [-0.6275, 0.6475, 0.40])
    assert ("board_position_ready", True) in predicates
    assert ("auto_approach_active", True) in predicates


def test_centering_keeps_reference_yaw_but_looks_down(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._provisional_world_T_base_cam = np.eye(4)
    component._provisional_world_T_base_cam[:3, 3] = [-0.75, 0.56, 0.0]
    component._provisional_flange_T_robot_cam = np.eye(4)
    component._first_world_T_flange = np.eye(4)
    component._first_world_T_flange[:3, :3] = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    component._first_world_T_flange[:3, 3] = [-0.65, 0.65, 0.40]
    component._base_cam_board_pose_for_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    component._board_geometry_reference = np.array([5.0, 7.0, 35.0])
    component._center_view_height_m = None
    component._center_camera_rotation = None
    component.get_parameter = _centering_parameters()
    component.get_logger = lambda: SimpleNamespace(info=lambda *_: None)
    component.set_predicate = lambda *_: None
    moves = []
    component._begin_transform_move = lambda target, state: moves.append((target.copy(), state))

    component._begin_auto_approach()

    np.testing.assert_allclose(
        moves[0][0][:3, :3],
        np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]),
    )


def test_auto_approach_rejects_view_outside_calibration_workspace(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._provisional_world_T_base_cam = np.eye(4)
    component._provisional_flange_T_robot_cam = np.eye(4)
    component._first_world_T_flange = np.eye(4)
    component._first_world_T_flange[:3, 3] = [-0.65, 0.65, 0.40]
    component._base_cam_board_pose_for_position = [0.6, 0.0, 0.0, 0.0, 0.0, 0.0]
    component._board_geometry_reference = np.array([5.0, 7.0, 35.0])
    component._center_view_height_m = None
    component._center_camera_rotation = None
    component.get_parameter = _centering_parameters()

    with pytest.raises(ValueError, match="außerhalb des Kalibrier-Arbeitsraums"):
        component._begin_auto_approach()


def test_position_complete_predicate_uses_the_matching_position(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    updates = []
    component.set_predicate = lambda name, value: updates.append((name, value))

    component._mark_position_complete(4)

    assert updates == [("board_position_4_complete", True)]


def test_workspace_parameter_accepts_negative_world_x(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    parameter = SimpleNamespace(
        get_name=lambda: "calibration_ws_x_min",
        is_empty=lambda: False,
        get_value=lambda: -0.9,
    )

    assert component.on_validate_parameter_callback(parameter) is True


def test_failed_run_writes_diagnostic_only_to_failed_runs_archive(monkeypatch, tmp_path):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._failure_diagnostic_written = False
    component._board_count = 3
    transform = np.eye(4)
    component._position_results = [SimpleNamespace(
        T_ee_robot_cam=transform.copy(),
        T_robot_base_static_cam=transform.copy(),
        sample_count=9,
    )]
    component._position_details = [{"position_index": 1, "sample_count": 9}]
    messages = []
    component.get_logger = lambda: SimpleNamespace(
        info=lambda message: messages.append(message), warn=lambda message: messages.append(message)
    )
    monkeypatch.setitem(
        component._write_failure_diagnostic.__globals__, "ROBOT_CAM_MULTI_ARCHIVE_DIR", str(tmp_path)
    )

    component._write_failure_diagnostic("Konsistenzgrenze verletzt")

    diagnostics = list((tmp_path / "failed_runs").glob("*_failed_diagnostic.json"))
    assert len(diagnostics) == 1
    payload = json.loads(diagnostics[0].read_text(encoding="utf-8"))
    assert payload["status"] == "rejected"
    assert payload["failure_message"] == "Konsistenzgrenze verletzt"
    assert payload["completed_board_positions"] == 1
    assert "aktive Kalibrierung" in payload["note"]
    assert any("Diagnose eines abgelehnten" in message for message in messages)
