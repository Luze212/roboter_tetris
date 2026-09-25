"""Tests for the calibration file in the package and the calibration component.

The first part runs without ROS: the shipped file, how base_cam and the
component read it, and the component description against the parameter table.
The last part builds the component and needs the AICA runtime image.
"""

import importlib.util
import json
import os

import numpy as np
import pytest

from roboter_tetris.basecam_extrinsics import (
    DEFAULT_CALIBRATION_FILE, L6_CAL, ExtrinsicsRecord, cal_from_matrix,
    load_camera_calibration, matrix_from_cal, pose_from_quaternion, quaternion_from_matrix,
    resolve_calibration_path, rotation_angle_deg, save_record,
)
from roboter_tetris.calibration_run import PARAMETERS, RunParams, run_params
from roboter_tetris.vision.detection import build_cam_to_robot

_DESCRIPTIONS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "..", "component_descriptions")


def _description(name):
    with open(os.path.join(_DESCRIPTIONS, f"roboter_tetris_{name}.json"), encoding="utf-8") as f:
        return json.load(f)


# -- the shipped calibration file -------------------------------------------------

def test_shipped_file_is_the_l6_calibration():
    record, line = load_camera_calibration(DEFAULT_CALIBRATION_FILE)
    assert record is not None, line
    assert np.allclose(record.world_T_cam, build_cam_to_robot(*L6_CAL), atol=1e-8)
    cal = cal_from_matrix(record.world_T_cam)
    assert [round(cal[k], 6) for k in ("cal_x", "cal_y", "cal_z", "cal_roll", "cal_pitch", "cal_yaw")] \
        == list(L6_CAL)
    assert "uebergang_L6" in line


def test_base_cam_defaults_equal_the_shipped_file():
    """Until the first real calibration, base_cam must see the same extrinsics
    whether it reads the file or its cal_* parameters."""
    d = {p["parameter_name"]: p["default_value"] for p in _description("base_cam")["parameters"]}
    assert d["calibration_file"] == DEFAULT_CALIBRATION_FILE
    assert tuple(float(d[k]) for k in ("cal_x", "cal_y", "cal_z", "cal_roll", "cal_pitch", "cal_yaw")) \
        == L6_CAL


def test_calibration_file_sources(tmp_path):
    record, line = load_camera_calibration("")
    assert record is None and "cal_*" in line
    record, line = load_camera_calibration(str(tmp_path / "fehlt.json"))
    assert record is None and "unbrauchbar" in line
    moved = matrix_from_cal(-0.78, 0.80, 0.915, 179.5, 0.4, 179.7)
    path = str(tmp_path / "neu.json")
    save_record(path, ExtrinsicsRecord(world_T_cam=moved, method="stufe1", created="2026-09-25"))
    record, line = load_camera_calibration(path)                       # absolute path
    assert np.allclose(record.world_T_cam, moved, atol=1e-8) and "stufe1" in line
    assert resolve_calibration_path(" Extrinsics/x.json ").endswith(
        os.path.join("roboter_tetris", "Extrinsics", "x.json"))


def test_quaternions_round_trip():
    for T in (matrix_from_cal(*L6_CAL), matrix_from_cal(0.1, 0.2, 0.3, 10.0, -20.0, 30.0),
              matrix_from_cal(0, 0, 0, 0, 0, 0)):
        w, x, y, z = quaternion_from_matrix(T[:3, :3])
        assert w >= 0.0 and abs(w * w + x * x + y * y + z * z - 1.0) < 1e-12
        back = pose_from_quaternion(T[:3, 3], (w, x, y, z))
        assert rotation_angle_deg(back[:3, :3] @ T[:3, :3].T) < 1e-9
        assert np.allclose(back[:3, 3], T[:3, 3])


# -- component description ----------------------------------------------------------

def test_description_matches_the_parameter_table():
    d = _description("base_cam_calibration")
    assert d["registration"] == "roboter_tetris::BaseCamCalibration"
    described = {p["parameter_name"]: p for p in d["parameters"]}
    assert list(described) == [n for n, _, _ in PARAMETERS]
    for name, default, _ in PARAMETERS:
        p = described[name]
        kind = ("bool" if isinstance(default, bool) else "int" if isinstance(default, int)
                else "double" if isinstance(default, float) else "string")
        assert p["parameter_type"] == kind, name
        if kind == "bool":
            assert p["default_value"] == ("true" if default else "false"), name
        elif kind in ("int", "double"):
            assert float(p["default_value"]) == default, name
        else:
            assert p["default_value"] == default, name
    assert "rate" not in described                                      # K2
    signals = {s["signal_name"] for s in d["inputs"] + d["outputs"]}
    assert signals == {"color_image", "color_camera_info", "depth_image", "robot_state",
                       "target_pose", "debug_image"}


def test_description_defaults_give_the_default_run():
    d = _description("base_cam_calibration")
    values = {}
    for p in d["parameters"]:
        kind, raw = p["parameter_type"], p["default_value"]
        values[p["parameter_name"]] = (raw == "true" if kind == "bool" else int(raw) if kind == "int"
                                       else float(raw) if kind == "double" else raw)
    assert run_params(values) == RunParams()


# -- the component (AICA runtime image only) ----------------------------------------

HAVE_RUNTIME = (importlib.util.find_spec("cv_bridge") is not None
                and importlib.util.find_spec("modulo_components") is not None)
runtime = pytest.mark.skipif(not HAVE_RUNTIME, reason="cv_bridge/modulo fehlen")


@pytest.fixture()
def component(request):
    if not HAVE_RUNTIME:
        pytest.skip("cv_bridge/modulo fehlen")
    request.getfixturevalue("ros_context")
    from roboter_tetris.base_cam_calibration import BaseCamCalibration
    yield BaseCamCalibration("base_cam_calibration")


@runtime
def test_every_signal_is_created(component):
    """Nachtrag 13 / L17: each signal owns a "<signal>_topic" parameter; a
    validation that refused those once left a component without ports."""
    for signal in ("color_image", "color_camera_info", "depth_image", "robot_state",
                   "target_pose", "debug_image"):
        assert component.get_parameter(f"{signal}_topic").get_value(), signal


@runtime
def test_parameters_carry_the_table_defaults(component):
    for name, default, _ in PARAMETERS:
        assert component.get_parameter(name).get_value() == default, name
