import pytest

from roboter_tetris.data_tracker import DataTracker


@pytest.fixture()
def data_tracker_component(ros_context):
    yield DataTracker("data_tracker")


def test_construction(data_tracker_component):
    assert data_tracker_component.get_name() == "data_tracker"
