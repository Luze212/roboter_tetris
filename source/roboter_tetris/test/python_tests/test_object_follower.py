import pytest

from roboter_tetris.object_follower import ObjectFollower


@pytest.fixture()
def object_follower_component(ros_context):
    yield ObjectFollower("object_follower")


def test_construction(object_follower_component):
    assert object_follower_component.get_name() == "object_follower"


SIGNALS = ("robot_state", "target", "object_position", "gripper_motion_done",
           "gripper_has_object", "target_pose", "gripper_close", "picked_id",
           "follower_status")


def test_every_signal_is_created(object_follower_component):
    """24.09.2026 (Nachtrag 13 / L17): the parameter check treated modulo's
    "<signal>_topic" strings as numbers and failed, so the follower loaded
    without a single input or output. Each signal owns such a parameter."""
    for signal in SIGNALS:
        topic = object_follower_component.get_parameter(f"{signal}_topic")
        assert topic.get_value(), signal
