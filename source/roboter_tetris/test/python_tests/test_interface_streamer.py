import pytest


@pytest.fixture()
def interface_streamer_component(request):
    # cv_bridge: in the AICA runtime image, not in the package-builder test
    # image -- skipped per test so the file still collects one (see
    # test_base_cam_contract.py, Nachtrag 13 / L26). Checked before ros_context.
    pytest.importorskip("cv_bridge")
    request.getfixturevalue("ros_context")
    from roboter_tetris.interface_streamer import InterfaceStreamer
    yield InterfaceStreamer("interface_streamer")


def test_construction(interface_streamer_component):
    assert interface_streamer_component.get_name() == "interface_streamer"
