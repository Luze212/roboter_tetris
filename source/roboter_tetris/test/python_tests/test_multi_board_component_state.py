"""Robot-free checks of the new component's pause and resume contract."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np


def _load_component(monkeypatch):
    """Load only the subclass against a tiny lifecycle stand-in, without ROS."""
    state_representation = ModuleType("state_representation")
    state_representation.Parameter = object
    state_representation.CartesianPose = lambda *_: SimpleNamespace(is_empty=lambda: True)
    monkeypatch.setitem(sys.modules, "state_representation", state_representation)

    base_name = "roboter_tetris.robot_cam_handeye_calibration.calibration_component"
    base_module = ModuleType(base_name)

    class Base:
        def _on_start_calibration(self):
            if self._state in ("IDLE", "FINISHED", "FAILED"):
                self._state = "MOVING"
                self._starts += 1

    base_module.RobotCamHandEyeCalibration = Base
    monkeypatch.setitem(sys.modules, base_name, base_module)
    path = (Path(__file__).resolve().parents[2] / "roboter_tetris" /
            "robot_cam_handeye_calibration" / "multi_board_component.py")
    spec = importlib.util.spec_from_file_location(
        "roboter_tetris.robot_cam_handeye_calibration._state_contract_under_test", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotCamHandEyeThreeBoardPositions


def test_continue_requires_changed_board_and_enters_moving(monkeypatch):
    component_type = _load_component(monkeypatch)
    component = object.__new__(component_type)
    component._state = "WAIT_BOARD_REPOSITION"
    component._position_index = 0
    component._starts = 0
    component._pause_robot_observation_id = 11
    component._pause_base_observation_id = 12
    component._previous_board_in_base_cam = np.eye(4)
    component._fresh_flange = lambda: np.eye(4)
    component._is_ee_at_target = lambda: True
    component.get_parameter = lambda _: SimpleNamespace(get_value=lambda: 30.0)
    component.set_predicate = lambda *_: None

    unchanged = np.eye(4)
    unchanged[0, 3] = 0.01
    component._new_board_observations = lambda *_: unchanged
    response = component._continue_service()
    assert response["success"] is False
    assert component._state == "WAIT_BOARD_REPOSITION"
    assert component._starts == 0

    changed = np.eye(4)
    changed[0, 3] = 0.05
    component._new_board_observations = lambda *_: changed
    response = component._continue_service()
    assert response["success"] is True
    assert component._position_index == 1
    assert component._state == "MOVING"
    assert component._starts == 1
