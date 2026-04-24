"""Palletizer State Machine.

Manages the lifecycle of palletizing operations using vention-state-machine.
Documentation: https://docs.vention.io/docs/state-machine
"""

import json
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

import numpy as np
from robot.motion import MotionController
from state_machine.core import BaseTriggers, StateMachine
from state_machine.decorators import on_enter_state, on_state_change
from state_machine.defs import State, StateGroup, Trigger
from transforms.coordinate import camera_to_robot
from utils.logger import logger
from utils.units import mm_to_m

from palletizer.grid import calculate_place_positions


class PalletizerState(Enum):
    """Palletizer operation states."""

    IDLE = auto()
    HOMING = auto()
    PICKING = auto()
    PLACING = auto()
    FAULT = auto()


class Running(StateGroup):
    """Active operation states."""

    homing: State = State()
    picking: State = State()
    placing: State = State()


class States:
    """Define the running group."""

    running = Running()


class Triggers:
    """Named events that initiate transitions."""

    finished_homing = Trigger("finished_homing")
    finished_picking = Trigger("finished_picking")
    finished_placing = Trigger("finished_placing")
    cycle_complete = Trigger("cycle_complete")
    stop = Trigger("stop")


TRANSITIONS = [
    Trigger("start").transition("ready", States.running.homing),
    Triggers.finished_homing.transition(
        States.running.homing, States.running.picking
    ),
    Triggers.finished_picking.transition(
        States.running.picking, States.running.placing
    ),
    Triggers.finished_placing.transition(
        States.running.placing, States.running.picking
    ),
    Triggers.cycle_complete.transition(States.running.placing, "ready"),
    Triggers.stop.transition(States.running.homing, "ready"),
    Triggers.stop.transition(States.running.picking, "ready"),
    Triggers.stop.transition(States.running.placing, "ready"),
]


@dataclass
class PalletizerContext:
    """Shared context for state machine operations."""

    rows: int = 2
    cols: int = 2
    box_size_mm: tuple[float, float, float] = (100.0, 100.0, 50.0)
    pallet_origin_mm: tuple[float, float, float] = (400.0, -200.0, 100.0)
    pick_height_mm: float = 50.0
    current_box_index: int = 0
    total_boxes: int = 0
    pick_position: tuple[float, float, float] | None = None
    place_positions: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    error_message: str = ""


class PalletizerStateMachine(StateMachine):
    """State machine for palletizing operations.

    Usage:
        machine = PalletizerStateMachine()
        machine.trigger('start')  # Transitions to HOMING
    """

    def __init__(self, motion_controller: MotionController) -> None:
        """Initialize the state machine."""
        super().__init__(
            states=States,
            transitions=TRANSITIONS,
            enable_last_state_recovery=False,
        )
        self.motion_controller = motion_controller
        self.context = PalletizerContext()
        self.detections = self._load_detections()

    def _load_detections(self) -> list[dict]:
        """Load mocked camera detections from disk."""
        detections_path = (
            Path(__file__).resolve().parents[1]
            / "data"
            / "camera_detections.json"
        )
        with detections_path.open("r", encoding="utf-8") as detections_file:
            payload = json.load(detections_file)
        return payload.get("detections", [])

    @property
    def current_state(self) -> PalletizerState:
        """Get current state. Note: library uses format 'Running_homing' not 'running.homing'."""
        state_str = self.state
        mapping = {
            "ready": PalletizerState.IDLE,
            "fault": PalletizerState.FAULT,
            "Running_homing": PalletizerState.HOMING,
            "Running_picking": PalletizerState.PICKING,
            "Running_placing": PalletizerState.PLACING,
        }
        return mapping.get(state_str, PalletizerState.IDLE)

    @property
    def progress(self) -> dict:
        """Get current progress: state, current_box, total_boxes, error."""
        return {
            "state": self.current_state.name,
            "current_box": self.context.current_box_index,
            "total_boxes": self.context.total_boxes,
            "error": self.context.error_message
            if self.context.error_message
            else None,
        }

    def configure(
        self,
        rows: int,
        cols: int,
        box_size_mm: tuple[float, float, float],
        pallet_origin_mm: tuple[float, float, float],
    ) -> bool:
        """Configure palletizing parameters. Only valid in IDLE state."""
        if self.current_state != PalletizerState.IDLE:
            return False

        self.context.rows = rows
        self.context.cols = cols
        self.context.box_size_mm = box_size_mm
        self.context.pallet_origin_mm = pallet_origin_mm
        self.context.pick_height_mm = box_size_mm[2]
        self.context.total_boxes = rows * cols
        self.context.current_box_index = 0
        self.context.place_positions = []
        return True

    def begin(self) -> bool:
        """Start the palletizing sequence."""
        if self.current_state != PalletizerState.IDLE:
            return False
        try:
            self.trigger("start")
            return True
        except Exception:
            return False

    def stop(self) -> bool:
        """Stop the palletizing sequence and return to IDLE."""
        if self.current_state == PalletizerState.IDLE:
            return True
        try:
            self.trigger("stop")
            return True
        except Exception:
            return False

    def reset(self) -> bool:
        """Reset from FAULT state to IDLE."""
        try:
            self.trigger(BaseTriggers.RESET.value)
            self.context.error_message = ""
            return True
        except Exception:
            return False

    def fault(self, message: str) -> bool:
        """Transition to FAULT state with an error message."""
        self.context.error_message = message
        try:
            self.trigger(BaseTriggers.TO_FAULT.value)
            return True
        except Exception:
            return False

    # State Entry Callbacks - Implement your business logic here

    @on_enter_state(States.running.homing)
    def on_enter_homing(self, _) -> None:
        """Provides the motion controls for returning to the home position.

        If motion is completed successfully, set the 'finished_homing' trigger.
        If motion is unsuccessful, transition to the global fault state.
        Motion is done synchronously.
        """
        try:
            if not self.motion_controller.move_to_home():
                self.fault("Failed to move robot to home position")
                return

            self.trigger("finished_homing")
        except Exception as exc:
            self.fault(f"Homing failed: {exc}")

    @on_enter_state(States.running.picking)
    def on_enter_picking(self, _) -> None:
        """Provides the motion controls for the picking sequence.

        Picking Sequence:
        Reach Safety Height > Approach > Grip > Reach Safety Height

        All motion calls are synchronous and individually verified to ensure
        future movement commands are not sent if an earlier fails.

        Sets trigger 'finished_picking' when completed successfully.
        Sets fault state when any motion fails.
        """
        try:
            # Verify box index against the number of detections
            if self.context.current_box_index >= len(self.detections):
                self.fault("No remaining camera detections for picking")
                return

            # Get camera point and transform to robot coords
            detection = self.detections[self.context.current_box_index]
            point_camera_mm = np.array(
                [
                    detection["x_mm"],
                    detection["y_mm"],
                    detection["z_mm"],
                ]
            )
            point_robot_mm = camera_to_robot(point_camera_mm)
            point_robot_mm[2] = self.context.pick_height_mm

            self.context.pick_position = tuple(point_robot_mm)

            # Logging in mm to be consistent with sim position units.
            logger.log_pick_target(
                self.context.current_box_index,
                self.context.total_boxes,
                point_robot_mm,
                detection.get("yaw_deg", 0.0),
            )

            # Create and execute full target pose + orientation
            pick_position_m = mm_to_m(point_robot_mm)
            pick_orientation = (
                self.motion_controller.get_pick_orientation_for_yaw(
                    detection.get("yaw_deg", 0.0)
                )
            )
            if not self.motion_controller.move_to_pick(
                pick_position_m,
                orientation=pick_orientation,
            ):
                self.fault("Failed to pick box at detected position")
                return

            self.trigger("finished_picking")
        except Exception as exc:
            self.fault(f"Picking failed: {exc}")

    @on_enter_state(States.running.placing)
    def on_enter_placing(self, _) -> None:
        """Provides the motion controls for the placing sequence.

        Placing Sequence:
        Approach > Release Grip > Reach Safety Height

        All motion calls are synchronous and individually verified to ensure
        future movement commands are not sent if an earlier fails.

        Sets trigger 'finished_placing' when completed successfully.
        Sets trigger 'cycle_complete' when the final placement is successful.
        Sets fault state when any motion fails.
        """
        try:
            # Ensure place positions have been calculated for a configuration
            if not self.context.place_positions:
                self.context.place_positions = calculate_place_positions(
                    rows=self.context.rows,
                    cols=self.context.cols,
                    box_size_mm=self.context.box_size_mm,
                    pallet_origin_mm=self.context.pallet_origin_mm,
                )

            # Get place position if valid
            if self.context.current_box_index >= len(
                self.context.place_positions
            ):
                self.fault("No remaining place positions in configured grid")
                return

            place_position_mm = self.context.place_positions[
                self.context.current_box_index
            ]

            # Log in mm to remain consistent with sim units.
            logger.log_place_target(
                self.context.current_box_index,
                self.context.total_boxes,
                place_position_mm,
            )

            # Execute place sequence
            place_position_m = mm_to_m(place_position_mm)
            if not self.motion_controller.move_to_place(place_position_m):
                self.fault("Failed to place box at target position")
                return

            self.context.current_box_index += 1

            if self.context.current_box_index >= self.context.total_boxes:
                self.trigger("cycle_complete")
            else:
                self.trigger("finished_placing")
        except Exception as exc:
            self.fault(f"Placing failed: {exc}")

    @on_state_change
    def on_any_state_change(
        self, old_state: str, new_state: str, trigger: str
    ) -> None:
        """Called on every state transition. Useful for logging."""
        logger.log_state_change(old_state, new_state, trigger)
