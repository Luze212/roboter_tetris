import pytest

from roboter_tetris.object_follower import ObjectFollower


@pytest.fixture()
def object_follower_component(ros_context):
    yield ObjectFollower("object_follower")


def test_construction(object_follower_component):
    assert object_follower_component.get_name() == "object_follower"
