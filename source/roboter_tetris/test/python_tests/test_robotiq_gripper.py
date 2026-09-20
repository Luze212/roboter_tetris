import pytest

from roboter_tetris.robotiq_gripper import (
    GOBJ_AT_POSITION,
    GOBJ_IN_MOTION,
    GOBJ_OBJECT_WHILE_CLOSING,
    GOBJ_OBJECT_WHILE_OPENING,
    GripperMotionState,
    GripperTargetLogic,
    RobotiqGripperComponent,
    clamp_mm,
    percent_to_raw,
)


@pytest.fixture()
def robotiq_gripper_component(ros_context):
    yield RobotiqGripperComponent("robotiq_gripper")


def test_construction(robotiq_gripper_component):
    assert robotiq_gripper_component.get_name() == "robotiq_gripper"


# -- percent_to_raw / clamp_mm -------------------------------------------------

@pytest.mark.parametrize(
    "percent,expected",
    [(0, 0), (100, 255), (50, 128), (-10, 0), (150, 255)],
)
def test_percent_to_raw(percent, expected):
    assert percent_to_raw(percent) == expected


@pytest.mark.parametrize(
    "value,expected",
    [(-5, 0.0), (0, 0.0), (65, 65.0), (130, 130.0), (200, 130.0)],
)
def test_clamp_mm(value, expected):
    assert clamp_mm(value) == expected


# -- GripperTargetLogic (precedence + edge detection) --------------------------

def test_close_rising_edge_closes():
    logic = GripperTargetLogic()
    command = logic.update_close(True)
    assert command is not None and command.kind == "close"


def test_repeated_close_value_does_not_retrigger():
    logic = GripperTargetLogic()
    assert logic.update_close(False) is None  # same as initial state
    assert logic.update_close(True) is not None
    assert logic.update_close(True) is None


def test_close_has_priority_over_change():
    logic = GripperTargetLogic()
    logic.update_close(True)
    assert logic.update_change(50) is None  # ignored while closed


def test_change_applies_when_open():
    logic = GripperTargetLogic()
    command = logic.update_change(40)
    assert command is not None
    assert command.kind == "move_mm"
    assert command.value_mm == 40.0


def test_change_is_clamped_to_calibrated_range():
    logic = GripperTargetLogic()
    assert logic.update_change(200).value_mm == 130.0


def test_repeated_identical_change_does_not_retrigger():
    logic = GripperTargetLogic()
    assert logic.update_change(40) is not None
    assert logic.update_change(40) is None


def test_close_falling_edge_opens_and_resets_change_tracking():
    logic = GripperTargetLogic()
    logic.update_change(40)
    assert logic.update_close(True).kind == "close"
    open_command = logic.update_close(False)
    assert open_command is not None and open_command.kind == "open"
    # The same change value must re-apply after the gripper has reopened.
    reapplied = logic.update_change(40)
    assert reapplied is not None and reapplied.kind == "move_mm"


# -- GripperMotionState (S9 feedback, hardware-free) ---------------------------


def test_motion_state_starts_idle():
    state = GripperMotionState()
    assert state.motion_done is False
    assert state.has_object is False


def test_new_command_clears_motion_done():
    state = GripperMotionState()
    state.on_status(GOBJ_AT_POSITION)
    assert state.motion_done is True
    state.on_command_start()
    assert state.motion_done is False


def test_in_motion_is_not_done():
    state = GripperMotionState()
    state.on_status(GOBJ_IN_MOTION)
    assert state.motion_done is False
    assert state.has_object is False


def test_reaching_the_position_without_an_object_is_a_missed_grip():
    # motion_done without has_object is exactly the signature the follower
    # uses to detect a failed grasp.
    state = GripperMotionState()
    state.on_command_start()
    state.on_status(GOBJ_AT_POSITION)
    assert state.motion_done is True
    assert state.has_object is False


def test_stopping_on_an_object_while_closing_is_a_successful_grip():
    state = GripperMotionState()
    state.on_command_start()
    state.on_status(GOBJ_OBJECT_WHILE_CLOSING)
    assert state.motion_done is True
    assert state.has_object is True


def test_object_while_opening_also_reports_has_object():
    # Documented gOBJ quirk: the status reports an object the jaws meet while
    # opening, so has_object can flicker during LOESEN. The follower must use
    # motion_done there instead -- this test pins the behaviour it relies on.
    state = GripperMotionState()
    state.on_command_start()
    state.on_status(GOBJ_OBJECT_WHILE_OPENING)
    assert state.motion_done is True
    assert state.has_object is True


def test_reset_clears_both_signals():
    state = GripperMotionState()
    state.on_status(GOBJ_OBJECT_WHILE_CLOSING)
    state.reset()
    assert state.motion_done is False
    assert state.has_object is False


def test_has_object_follows_the_predicate_condition():
    # Abnahme 2.3: has_object must track the same gOBJ condition the component
    # uses for its is_object_grasped predicate.
    grasped_codes = (GOBJ_OBJECT_WHILE_OPENING, GOBJ_OBJECT_WHILE_CLOSING)
    for gobj in (GOBJ_IN_MOTION, GOBJ_OBJECT_WHILE_OPENING,
                 GOBJ_OBJECT_WHILE_CLOSING, GOBJ_AT_POSITION):
        state = GripperMotionState()
        state.on_status(gobj)
        assert state.has_object == (gobj in grasped_codes)
