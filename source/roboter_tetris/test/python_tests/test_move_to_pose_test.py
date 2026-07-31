import math

import pytest

from roboter_tetris.move_to_pose_test import (
    MoveToPoseTest,
    MoveTriggerLogic,
    quaternion_from_euler_deg,
)


@pytest.fixture()
def move_to_pose_component(ros_context):
    yield MoveToPoseTest("move_to_pose_test")


def test_construction(move_to_pose_component):
    assert move_to_pose_component.get_name() == "move_to_pose_test"


# -- quaternion_from_euler_deg -------------------------------------------------

def test_quaternion_identity_at_zero_angles():
    qx, qy, qz, qw = quaternion_from_euler_deg(0.0, 0.0, 0.0)
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_roll_180_points_gripper_down():
    qx, qy, qz, qw = quaternion_from_euler_deg(180.0, 0.0, 0.0)
    assert (qx, qy, qz, qw) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_quaternion_is_unit_length():
    qx, qy, qz, qw = quaternion_from_euler_deg(37.0, -12.5, 200.0)
    assert math.sqrt(qx**2 + qy**2 + qz**2 + qw**2) == pytest.approx(1.0)


# -- MoveTriggerLogic (rising-edge detection) ----------------------------------

def test_rising_edge_fires_once():
    logic = MoveTriggerLogic()
    assert logic.update(False) is False
    assert logic.update(True) is True
    assert logic.update(True) is False  # held high: no re-trigger
    assert logic.update(False) is False
    assert logic.update(True) is True   # new rising edge


def test_reset_allows_retrigger_without_falling_edge():
    logic = MoveTriggerLogic()
    assert logic.update(True) is True
    logic.reset()
    assert logic.update(True) is True
