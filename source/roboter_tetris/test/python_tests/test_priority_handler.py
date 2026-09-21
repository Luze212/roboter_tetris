import pytest

from roboter_tetris.priority_handler import PriorityHandler


@pytest.fixture()
def priority_handler_component(ros_context):
    yield PriorityHandler("priority_handler")


def test_construction(priority_handler_component):
    assert priority_handler_component.get_name() == "priority_handler"
