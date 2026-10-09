"""Unit tests for the file-only calibration fusion and its immutable archive."""

import json

import numpy as np
import pytest

from roboter_tetris.basecam_extrinsics import (
    ExtrinsicsRecord, archive_calibration_file, record_to_dict, save_record,
)
from roboter_tetris.calibration_fusion import fuse_calibration_files


def _record(x, method):
    matrix = np.eye(4)
    matrix[0, 3] = x
    return ExtrinsicsRecord(world_T_cam=matrix, method=method)


def _robot_source(path, x=1.0):
    record = _record(x, "robot_cam_handeye_charuco")
    data = record_to_dict(record)
    data.update({
        "schema_version": 3,
        "status": "validated",
        "transformations": {
            "T_world_base_static_cam": {
                "source_frame": "base_camera_frame",
                "target_frame": "world",
                "homogeneous_matrix": record.world_T_cam.tolist(),
            },
        },
    })
    path.write_text(json.dumps(data), encoding="utf-8")


def test_archive_copy_has_unique_timestamped_name(tmp_path):
    source = tmp_path / "active.json"
    source.write_text('{"value": 1}\n', encoding="utf-8")
    archive = tmp_path / "archive"

    first = archive_calibration_file(str(source), str(archive), "base_cam")
    second = archive_calibration_file(str(source), str(archive), "base_cam")

    assert first != second
    assert archive.is_dir()
    assert json.loads(open(first, encoding="utf-8").read()) == {"value": 1}
    assert json.loads(open(second, encoding="utf-8").read()) == {"value": 1}


def test_fusion_writes_active_and_immutable_archive(tmp_path, monkeypatch):
    base = tmp_path / "base.json"
    robot = tmp_path / "robot.json"
    output = tmp_path / "fused.json"
    save_record(str(base), _record(0.0, "stufe1_basecam"))
    _robot_source(robot)
    archive = tmp_path / "fusion_archive"
    monkeypatch.setattr("roboter_tetris.calibration_fusion.FUSION_ARCHIVE_DIR", str(archive))

    result = fuse_calibration_files(str(base), str(robot), str(output), 50.0)

    assert output.is_file()
    assert result["archive_path"].startswith(str(archive))
    assert json.loads(output.read_text(encoding="utf-8"))["method"] == "fusion_basecam_robotcam"
    assert json.loads(open(result["archive_path"], encoding="utf-8").read())["method"] == "fusion_basecam_robotcam"


def test_fusion_rejects_bad_robot_source_without_output(tmp_path):
    base = tmp_path / "base.json"
    robot = tmp_path / "robot.json"
    output = tmp_path / "fused.json"
    save_record(str(base), _record(0.0, "stufe1_basecam"))
    robot.write_text("{}", encoding="utf-8")

    with pytest.raises(ValueError):
        fuse_calibration_files(str(base), str(robot), str(output), 50.0)
    assert not output.exists()


def test_fusion_never_overwrites_protected_main_basecam_file(tmp_path, monkeypatch):
    protected = tmp_path / "base_cam_extrinsics.json"
    robot = tmp_path / "robot.json"
    save_record(str(protected), _record(0.0, "stufe1_basecam"))
    before = protected.read_bytes()
    _robot_source(robot)
    monkeypatch.setattr(
        "roboter_tetris.calibration_fusion.DEFAULT_BASE_CAM_FILE", str(protected)
    )

    with pytest.raises(ValueError, match="Geschützte Hauptdatei"):
        fuse_calibration_files(str(protected), str(robot), str(protected), 50.0)

    assert protected.read_bytes() == before
