import pytest

from roboter_tetris.interface_streamer import InterfaceStreamer


@pytest.fixture()
def interface_streamer_component(ros_context):
    yield InterfaceStreamer("interface_streamer")


def test_construction(interface_streamer_component):
    assert interface_streamer_component.get_name() == "interface_streamer"
