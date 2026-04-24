"""Unit test for coordinate frame conversion."""

import numpy as np
from transforms.coordinate import camera_to_robot, robot_to_camera


def test_camera_origin_maps_to_camera_position_in_robot_frame() -> None:
    """Tests if the camera to robot frame conversion works.

    Uses the point 0,0,0 in the camera frame as it should be equal to the 
    provided README camera position.
    """
    point_camera = np.array([0.0, 0.0, 0.0])

    point_robot = camera_to_robot(point_camera)

    assert np.allclose(point_robot, np.array([500.0, 300.0, 800.0]))


def test_robot_to_camera_inverts_camera_to_robot() -> None:
    """Tests if the conversion operations are inverses of one another."""
    point_camera = np.array([50.0, -30.0, 0.0])

    point_robot = camera_to_robot(point_camera)
    recovered_point_camera = robot_to_camera(point_robot)

    assert np.allclose(recovered_point_camera, point_camera)
