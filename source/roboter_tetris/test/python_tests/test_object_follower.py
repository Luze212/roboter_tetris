import pytest

from roboter_tetris.object_follower import ObjectFollower


@pytest.fixture()
def object_follower_component(ros_context):
    yield ObjectFollower("object_follower")


def test_construction(object_follower_component):
    assert object_follower_component.get_name() == "object_follower"


class _TopicParameter:
    """Minimal AICA-generated transport parameter for validator coverage."""

    def get_name(self):
        return "robot_state"

    def is_empty(self):
        return False

    def get_value(self):
        return "/hardware/robot_state_broadcaster/cartesian_state"


def test_topic_parameter_is_not_validated_as_a_follower_number(object_follower_component):
    """Signals register their topic names through the parameter callback."""
    assert object_follower_component.on_validate_parameter_callback(_TopicParameter())
