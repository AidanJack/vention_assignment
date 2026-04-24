"""Motion Controller.

================

Implements robot motion commands for pick and place operations.
"""

import numpy as np

from .connection import RobotConnection


class MotionController:
    """Controls robot motion for palletizing operations.

    Coordinates:
    - All positions are in meters
    - All orientations are in radians (axis-angle representation for UR)
    """

    # Safety parameters
    APPROACH_HEIGHT_OFFSET = 0.100  # 100mm above pick/place position
    DEFAULT_VELOCITY = 0.5  # m/s
    DEFAULT_ACCELERATION = 0.5  # m/s²
    MAX_REACH = 0.850  # m as defined by the URe5 User Manual
    HOME_JOINTS = [
        -np.pi / 2,
        -np.pi / 2,
        -3 * np.pi / 4,
        -np.pi / 4,
        np.pi / 2,
        0,
    ]

    def __init__(self, connection: RobotConnection) -> None:
        """Initialize motion controller.

        Args:
            connection: Active robot connection instance.
        """
        self.connection = connection
        self._gripper_closed = False

    def move_to_home(self) -> bool:
        """Move robot to home/safe position.

        Returns:
            True if move completed successfully.
        """
        return self._move_joint(self.HOME_JOINTS)

    def get_pick_orientation_for_yaw(self, yaw_deg: float) -> list[float]:
        """Rotate the default downward TCP orientation about the tool Z axis."""
        base_rotation = self._axis_angle_to_rotation_matrix(
            self.get_default_orientation()
        )
        yaw_rotation = np.array(
            [
                [
                    np.cos(np.deg2rad(yaw_deg)),
                    -np.sin(np.deg2rad(yaw_deg)),
                    0.0,
                ],
                [np.sin(np.deg2rad(yaw_deg)), np.cos(np.deg2rad(yaw_deg)), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        adjusted_rotation = base_rotation @ yaw_rotation
        return self._rotation_matrix_to_axis_angle(adjusted_rotation)

    def move_to_pick(
        self,
        position: list[float],
        orientation: list[float] | None = None,
    ) -> bool:
        """Execute pick motion sequence.

        Args:
            position: [x, y, z] pick position in robot base frame (meters)
            orientation: [rx, ry, rz] tool orientation (axis-angle, radians)
                        If None, use default downward orientation.

        Returns:
            True if pick completed successfully.
        """
        tcp_pose = self.connection.get_tcp_pose()
        if len(tcp_pose) != 6:
            return False

        tool_orientation = orientation or self.get_default_orientation()
        target_x, target_y, target_z = position
        approach_z = target_z + self.APPROACH_HEIGHT_OFFSET

        lift_pose = [
            tcp_pose[0],
            tcp_pose[1],
            approach_z,
            tcp_pose[3],
            tcp_pose[4],
            tcp_pose[5],
        ]
        approach_pose = [
            target_x,
            target_y,
            approach_z,
            tool_orientation[0],
            tool_orientation[1],
            tool_orientation[2],
        ]
        pick_pose = [
            target_x,
            target_y,
            target_z,
            tool_orientation[0],
            tool_orientation[1],
            tool_orientation[2],
        ]

        if not self._move_linear(lift_pose):
            return False
        if not self._move_linear(approach_pose):
            return False
        if not self._move_linear(pick_pose):
            return False
        if not self.close_gripper():
            return False
        if not self._move_linear(approach_pose):
            return False

        return True

    def move_to_place(
        self,
        position: list[float],
        orientation: list[float] | None = None,
    ) -> bool:
        """Execute place motion sequence.

        Args:
            position: [x, y, z] place position in robot base frame (meters)
            orientation: [rx, ry, rz] tool orientation (axis-angle, radians)
                        If None, use default downward orientation.

        Returns:
            True if place completed successfully.
        """
        tcp_pose = self.connection.get_tcp_pose()
        if len(tcp_pose) != 6:
            return False

        tool_orientation = orientation or self.get_default_orientation()
        target_x, target_y, target_z = position
        approach_z = target_z + self.APPROACH_HEIGHT_OFFSET

        lift_pose = [
            tcp_pose[0],
            tcp_pose[1],
            max(tcp_pose[2], approach_z),
            tcp_pose[3],
            tcp_pose[4],
            tcp_pose[5],
        ]
        approach_pose = [
            target_x,
            target_y,
            approach_z,
            tool_orientation[0],
            tool_orientation[1],
            tool_orientation[2],
        ]
        place_pose = [
            target_x,
            target_y,
            target_z,
            tool_orientation[0],
            tool_orientation[1],
            tool_orientation[2],
        ]

        if not self._move_linear(lift_pose):
            return False
        if not self._move_linear(approach_pose):
            return False
        if not self._move_linear(place_pose):
            return False
        if not self.open_gripper():
            return False
        if not self._move_linear(approach_pose):
            return False

        return True

    def open_gripper(self) -> bool:
        """Open the gripper to release object.

        Returns:
            True if gripper opened successfully.
        """
        self._gripper_closed = False
        print("[MOCK] Gripper opened")
        return True

    def close_gripper(self) -> bool:
        """Close the gripper to grasp object.

        Returns:
            True if gripper closed successfully.
        """
        self._gripper_closed = True
        print("[MOCK] Gripper closed")
        return True

    def _move_linear(
        self,
        pose: list[float],
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
    ) -> bool:
        """Execute linear move to target pose.

        Args:
            pose: [x, y, z, rx, ry, rz] target pose
            velocity: Move velocity in m/s
            acceleration: Move acceleration in m/s²

        Returns:
            True if move completed.
        """
        if not self._is_pose_in_workspace(pose):
            return False

        if self.connection.is_mock_mode():
            print(f"[MOCK] moveL to {pose[:3]}")
            return True

        rtde_control = self.connection.control
        if rtde_control is None or not self.connection.ensure_connected():
            return False

        return rtde_control.moveL(pose, velocity, acceleration)

    def _move_joint(
        self,
        joints: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
    ) -> bool:
        """Execute joint move to target configuration.

        Args:
            joints: List of 6 joint angles in radians
            velocity: Joint velocity in rad/s
            acceleration: Joint acceleration in rad/s²

        Returns:
            True if move completed.
        """
        if self.connection.is_mock_mode():
            print(f"[MOCK] moveJ to {joints}")
            return True

        rtde_control = self.connection.control
        if rtde_control is None or not self.connection.ensure_connected():
            return False

        return rtde_control.moveJ(joints, velocity, acceleration)

    def get_default_orientation(self) -> list[float]:
        """Get default tool orientation for picking (pointing down).

        Returns:
            [rx, ry, rz] in axis-angle representation.

        Note: For a tool pointing straight down (Z toward floor),
        the rotation from base frame is typically [0, π, 0] or [π, 0, 0]
        depending on your tool frame setup.
        """
        return [0.0, np.pi, 0.0]

    def _axis_angle_to_rotation_matrix(
        self, axis_angle: list[float]
    ) -> np.ndarray:
        """Convert a UR axis-angle orientation to a 3x3 rotation matrix."""
        rotation_vector = np.asarray(axis_angle, dtype=float)
        angle = np.linalg.norm(rotation_vector)
        if angle < 1e-9:
            return np.eye(3)

        axis = rotation_vector / angle
        x, y, z = axis
        skew = np.array(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0],
            ]
        )
        identity = np.eye(3)
        return (
            identity
            + np.sin(angle) * skew
            + (1.0 - np.cos(angle)) * (skew @ skew)
        )

    def _rotation_matrix_to_axis_angle(
        self, rotation: np.ndarray
    ) -> list[float]:
        """Convert a 3x3 rotation matrix to a UR axis-angle orientation."""
        trace_value = np.trace(rotation)
        cos_angle = np.clip((trace_value - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        if angle < 1e-9:
            return [0.0, 0.0, 0.0]

        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        ) / (2.0 * np.sin(angle))
        return (axis * angle).tolist()

    def _is_pose_in_workspace(self, pose: list[float]) -> bool:
        """Reject poses that are farther from the base than the arm reach."""
        x, y, z = pose[:3]
        reach = float(np.linalg.norm([x, y, z]))

        if reach > self.MAX_REACH:
            print("[SAFETY] Rejected pose outside robot reach")
            return False

        return True
