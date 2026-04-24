"""Coordinate Transformations.

=========================

Transform coordinates between camera frame and robot base frame.

Refer to the README for camera mounting specifications.
"""

import numpy as np


def build_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from Roll-Pitch-Yaw (Euler) angles.

    Args:
        roll: Rotation about X-axis in radians
        pitch: Rotation about Y-axis in radians
        yaw: Rotation about Z-axis in radians

    Returns:
        3x3 rotation matrix
    """
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ]
    )

    ry = np.array(
        [
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)],
        ]
    )

    rz = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    return rz @ ry @ rx


def camera_to_robot(point_camera: np.ndarray) -> np.ndarray:
    """Transform a point from camera frame to robot base frame.

    Args:
        point_camera: [x, y, z] coordinates in camera frame (mm)

    Returns:
        [x, y, z] coordinates in robot base frame (mm)
    """
    roll, pitch, yaw = np.deg2rad([15.0, -10.0, 45.0])

    rotation = build_rotation_matrix(roll, pitch, yaw)
    translation = np.array([500.0, 300.0, 800.0])
    transform = build_homogeneous_transform(
        rotation=rotation, translation=translation
    )

    point_camera_h = np.append(np.asarray(point_camera), 1.0)
    point_robot_h = transform @ point_camera_h

    return point_robot_h[:3]


def robot_to_camera(point_robot: np.ndarray) -> np.ndarray:
    """Transform a point from robot base frame to camera frame.

    Args:
        point_robot: [x, y, z] coordinates in robot base frame (mm)

    Returns:
        [x, y, z] coordinates in camera frame (mm)
    """
    roll, pitch, yaw = np.deg2rad([15.0, -10.0, 45.0])

    rotation = build_rotation_matrix(roll, pitch, yaw)
    translation = np.array([500.0, 300.0, 800.0])
    transform = build_homogeneous_transform(
        rotation=rotation, translation=translation
    )

    inverse_transform = np.linalg.inv(transform)

    point_robot_h = np.append(np.asarray(point_robot), 1.0)
    point_camera_h = inverse_transform @ point_robot_h

    return point_camera_h[:3]


def build_homogeneous_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Build a 4x4 homogeneous transformation matrix.

    Args:
        rotation: 3x3 rotation matrix
        translation: 3x1 or (3,) translation vector

    Returns:
        4x4 homogeneous transformation matrix
    """
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return transform
