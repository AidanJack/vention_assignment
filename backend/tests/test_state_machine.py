"""Tests for PalletizerStateMachine. Run with: pytest tests/test_state_machine.py -v"""

from palletizer.state_machine import (
    PalletizerState,
    PalletizerContext,
    States,
    TRANSITIONS,
)
from state_machine.core import StateMachine
from state_machine.decorators import on_enter_state


class _TestablePalletizerStateMachine(StateMachine):
    """Testable version with no-op entry handlers."""
    
    def __init__(self):
        super().__init__(states=States, transitions=TRANSITIONS, enable_last_state_recovery=False)
        self.context = PalletizerContext()
    
    @property
    def current_state(self) -> PalletizerState:
        mapping = {
            "ready": PalletizerState.IDLE,
            "fault": PalletizerState.FAULT,
            "Running_homing": PalletizerState.HOMING,
            "Running_picking": PalletizerState.PICKING,
            "Running_placing": PalletizerState.PLACING,
        }
        return mapping.get(self.state, PalletizerState.IDLE)
    
    @property
    def progress(self) -> dict:
        return {
            "state": self.current_state.name,
            "current_box": self.context.current_box_index,
            "total_boxes": self.context.total_boxes,
            "error": self.context.error_message if self.context.error_message else None,
        }
    
    def configure(self, rows, cols, box_size_mm, pallet_origin_mm) -> bool:
        if self.current_state != PalletizerState.IDLE:
            return False
        self.context.rows = rows
        self.context.cols = cols
        self.context.box_size_mm = box_size_mm
        self.context.pallet_origin_mm = pallet_origin_mm
        self.context.total_boxes = rows * cols
        self.context.current_box_index = 0
        return True
    
    def do_stop(self) -> bool:
        if self.current_state == PalletizerState.IDLE:
            return True
        try:
            self.trigger("stop")
            return True
        except Exception:
            return False
    
    def fault(self, message: str) -> bool:
        self.context.error_message = message
        try:
            self.trigger("to_fault")
            return True
        except Exception:
            return False
    
    def do_reset(self) -> bool:
        try:
            self.trigger("reset")
            self.context.error_message = ""
            return True
        except Exception:
            return False
    
    @on_enter_state(States.running.homing)
    def on_enter_homing(self, _):
        pass
    
    @on_enter_state(States.running.picking)
    def on_enter_picking(self, _):
        pass
    
    @on_enter_state(States.running.placing)
    def on_enter_placing(self, _):
        pass


def create_machine() -> _TestablePalletizerStateMachine:
    return _TestablePalletizerStateMachine()


class TestInitialState:
    def test_starts_in_idle(self):
        machine = create_machine()
        assert machine.current_state == PalletizerState.IDLE
    
    def test_progress_shows_idle(self):
        machine = create_machine()
        progress = machine.progress
        assert progress["state"] == "IDLE"
        assert progress["current_box"] == 0
        assert progress["error"] is None


class TestConfiguration:
    def test_configure_in_idle_succeeds(self):
        machine = create_machine()
        result = machine.configure(
            rows=3, cols=4,
            box_size_mm=(150.0, 150.0, 75.0),
            pallet_origin_mm=(500.0, -300.0, 150.0),
        )
        assert result is True
        assert machine.context.rows == 3
        assert machine.context.cols == 4
        assert machine.context.total_boxes == 12
    
    def test_configure_not_in_idle_fails(self):
        machine = create_machine()
        machine.trigger("start")
        result = machine.configure(
            rows=2, cols=2,
            box_size_mm=(100.0, 100.0, 50.0),
            pallet_origin_mm=(400.0, -200.0, 100.0),
        )
        assert result is False


class TestStateTransitions:
    def test_start_transitions_to_homing(self):
        machine = create_machine()
        machine.trigger("start")
        assert machine.current_state == PalletizerState.HOMING
    
    def test_homing_to_picking(self):
        machine = create_machine()
        machine.trigger("start")
        machine.trigger("finished_homing")
        assert machine.current_state == PalletizerState.PICKING
    
    def test_picking_to_placing(self):
        machine = create_machine()
        machine.trigger("start")
        machine.trigger("finished_homing")
        machine.trigger("finished_picking")
        assert machine.current_state == PalletizerState.PLACING
    
    def test_placing_to_picking_continues_cycle(self):
        machine = create_machine()
        machine.trigger("start")
        machine.trigger("finished_homing")
        machine.trigger("finished_picking")
        machine.trigger("finished_placing")
        assert machine.current_state == PalletizerState.PICKING
    
    def test_cycle_complete_returns_to_idle(self):
        machine = create_machine()
        machine.trigger("start")
        machine.trigger("finished_homing")
        machine.trigger("finished_picking")
        machine.trigger("cycle_complete")
        assert machine.current_state == PalletizerState.IDLE


class TestStopAndReset:
    def test_stop_from_homing(self):
        machine = create_machine()
        machine.trigger("start")
        assert machine.do_stop() is True
        assert machine.current_state == PalletizerState.IDLE
    
    def test_stop_from_picking(self):
        machine = create_machine()
        machine.trigger("start")
        machine.trigger("finished_homing")
        assert machine.do_stop() is True
        assert machine.current_state == PalletizerState.IDLE
    
    def test_fault_and_reset(self):
        machine = create_machine()
        machine.fault("Test error")
        assert machine.current_state == PalletizerState.FAULT
        assert machine.context.error_message == "Test error"
        
        machine.do_reset()
        assert machine.current_state == PalletizerState.IDLE
        assert machine.context.error_message == ""
