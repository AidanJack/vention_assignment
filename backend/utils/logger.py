"""Provides a project wide python logging singleton tool."""

import logging

import numpy as np


class RobotLogger:
    """Singleton logger for robot operations.

    Logger uses f-strings instead of lazy formatting for ease of use during
    assignment. Typically would use lazy formatting to ensure no additional
    formatting is done unless it will be logged.
    """

    def __init__(
        self, logger: logging.Logger | None = None, level: int = logging.INFO
    ) -> None:
        """Initialize the internal logger."""
        self.logger = logger if logger is not None else logging.getLogger()
        self.level = level

    def _log(self, message: str, *args: object) -> None:
        """Internal log function.

        Args:
            message: Fully formatted log string.
            args: Variables that are part of the log message.
        """
        self.logger.log(self.level, message, *args)

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    def log_pick_target(
        self,
        current_idx: int,
        total: int,
        point_robot_mm: np.ndarray,
        yaw_deg: float,
    ) -> None:
        """Log pick target info."""
        msg = (
            "Pick target box "
            f"{current_idx + 1}/{total}: "
            f"robot=({point_robot_mm[0]:.1f}, "
            f"{point_robot_mm[1]:.1f}, "
            f"{point_robot_mm[2]:.1f}) mm "
            f"yaw={yaw_deg:.1f} deg"
        )
        self._log(msg)

    def log_place_target(
        self,
        current_idx: int,
        total: int,
        place_position_mm: tuple[float, float, float],
    ) -> None:
        """Log place target info."""
        msg = (
            "Place target box "
            f"{current_idx + 1}/{total}: "
            f"robot=({place_position_mm[0]:.1f}, "
            f"{place_position_mm[1]:.1f}, "
            f"{place_position_mm[2]:.1f}) mm"
        )
        self._log(msg)

    def log_point(
        self,
        label: str,
        point: np.ndarray | tuple[float, float, float],
        units: str = "mm",
    ) -> None:
        """Generic point logger.

        Args:
            label: Name of the point.
            point: 3D point.
            units: Units string.
        """
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        self._log(f"{label}=({x:.1f}, {y:.1f}, {z:.1f}) {units}")

    def log_state_change(
        self, old_state: str, new_state: str, trigger: str
    ) -> None:
        """Logs state transitions."""
        msg = (
            f"Trigger : {trigger} is set. Transitioning from {old_state} to"
            f" {new_state}"
        )
        self._log(msg)

logger = RobotLogger()
