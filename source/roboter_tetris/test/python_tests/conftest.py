"""ros_context for the construction tests (AICA test image).

From the init wizard template; deleted on 01.06.2026 (31c5fcc), which left the
construction tests without their fixture (Nachtrag 13 / L26). rclpy is imported
inside the fixture so the ROS-free tests still run where rclpy is missing.
"""

import pytest


@pytest.fixture
def ros_context():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()
