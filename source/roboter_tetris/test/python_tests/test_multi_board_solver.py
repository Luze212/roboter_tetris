"""Pure, robot-free checks for the independent three-board aggregation."""

import json
import math

import numpy as np
import pytest

from roboter_tetris.basecam_extrinsics import record_from_dict
from roboter_tetris.robot_cam_handeye_calibration.handeye_solver import (
    HandEyeCalibrationResult, save_handeye_calibration_json,
)
from roboter_tetris.robot_cam_handeye_calibration.multi_board_solver import (
    combine_board_results, mean_transforms, rotation_angle_deg,
)


def _pose(x=0.0, yaw_deg=0.0):
    angle = math.radians(yaw_deg)
    result = np.eye(4)
    result[:3, :3] = [[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
    result[0, 3] = x
    return result


def _result(flange_x, base_x, yaw=0.0):
    return HandEyeCalibrationResult(
        T_robot_base_cam=_pose(-0.6),
        T_ee_robot_cam=_pose(flange_x, yaw),
        T_robot_base_static_cam=_pose(base_x, yaw),
        sample_count=9, position_rmse_mm=2.0, rotation_rmse_deg=0.3,
        flange_rotation_span_deg=12.0,
    )


def test_equal_weight_mean_and_basecam_json(tmp_path):
    groups = [_result(0.105, -0.780, -0.5), _result(0.11, -0.775, 0),
              _result(0.115, -0.770, 0.5)]
    combined, report = combine_board_results(groups, _pose(-0.6), 20, 1.5)
    assert combined.sample_count == 27
    assert combined.T_ee_robot_cam[0, 3] == pytest.approx(0.11)
    assert combined.T_robot_base_static_cam[0, 3] == pytest.approx(-0.775)
    assert rotation_angle_deg(combined.T_ee_robot_cam[:3, :3]) == pytest.approx(0, abs=1e-7)
    assert report["world_base_static_cam"]["max_pairwise_translation_mm"] == pytest.approx(10)
    assert combined.T_robot_board is None
    assert combined.T_robot_conveyor is None

    path = tmp_path / "three_positions.json"
    save_handeye_calibration_json(str(path), combined, multi_board={"positions": [1, 2, 3]})
    data = json.loads(path.read_text())
    assert data["schema_version"] == 3
    assert data["method"] == "robot_cam_handeye_charuco"
    assert data["multi_board"]["positions"] == [1, 2, 3]
    assert "T_world_conveyor" not in data["transformations"]
    assert "board_center_conveyor_mm" not in data
    assert record_from_dict(data).world_T_cam[0, 3] == pytest.approx(-0.775)


def test_outlier_is_rejected_before_writing():
    groups = [_result(0.10, -0.780), _result(0.11, -0.775), _result(0.12, -0.730)]
    with pytest.raises(ValueError, match="widersprechen"):
        combine_board_results(groups, _pose(-0.6), 20, 1.5)


def test_accepts_configurable_group_count_and_rejects_empty_groups():
    combined, report = combine_board_results(
        [_result(.1, -.78), _result(.11, -.775)], _pose(-.6), 20, 1.5
    )
    assert combined.sample_count == 18
    assert report["group_count"] == 2
    with pytest.raises(ValueError, match="Mindestens eine"):
        combine_board_results([], _pose(-.6), 20, 1.5)
    bad = _pose(.1)
    bad[0, 0] = 2.0
    with pytest.raises(ValueError, match="Ungültige"):
        mean_transforms([bad])
