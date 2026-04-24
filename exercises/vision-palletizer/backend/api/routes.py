"""Palletizer API Routes.

====================

FastAPI routes for palletizer control.
"""

from typing import Any

import numpy as np
from fastapi import APIRouter, HTTPException, Request
from palletizer.grid import calculate_place_positions
from pydantic import BaseModel, Field
from transforms.coordinate import camera_to_robot, robot_to_camera

router = APIRouter()


# ============================================================================
# Request/Response Models
# ============================================================================


class PalletConfig(BaseModel):
    """Configuration for palletizing operation."""

    rows: int = Field(
        ..., ge=1, le=10, description="Number of rows in the grid"
    )
    cols: int = Field(
        ..., ge=1, le=10, description="Number of columns in the grid"
    )
    box_width_mm: float = Field(
        ..., gt=0, description="Box width in mm (X direction)"
    )
    box_depth_mm: float = Field(
        ..., gt=0, description="Box depth in mm (Y direction)"
    )
    box_height_mm: float = Field(
        ..., gt=0, description="Box height in mm (Z direction)"
    )
    pallet_origin_x_mm: float = Field(..., description="Pallet origin X in mm")
    pallet_origin_y_mm: float = Field(..., description="Pallet origin Y in mm")
    pallet_origin_z_mm: float = Field(..., description="Pallet origin Z in mm")

    class Config:
        """Provide OpenAPI example data for the pallet configuration model."""

        json_schema_extra = {
            "example": {
                "rows": 2,
                "cols": 2,
                "box_width_mm": 100.0,
                "box_depth_mm": 100.0,
                "box_height_mm": 50.0,
                "pallet_origin_x_mm": 400.0,
                "pallet_origin_y_mm": -200.0,
                "pallet_origin_z_mm": 100.0,
            }
        }


class VisionDetection(BaseModel):
    """Simulated vision detection of a box."""

    x_mm: float = Field(..., description="Box X position in camera frame (mm)")
    y_mm: float = Field(..., description="Box Y position in camera frame (mm)")
    z_mm: float = Field(..., description="Box Z position in camera frame (mm)")
    yaw_deg: float | None = Field(
        0.0, description="Box rotation about Z (degrees)"
    )

    class Config:
        """Provide OpenAPI example data for the vision detection model."""

        json_schema_extra = {
            "example": {
                "x_mm": 50.0,
                "y_mm": -30.0,
                "z_mm": 0.0,
                "yaw_deg": 15.0,
            }
        }


class StatusResponse(BaseModel):
    """Palletizer status response."""

    state: str = Field(..., description="Current state machine state")
    current_box: int = Field(..., description="Current box index (0-based)")
    total_boxes: int = Field(..., description="Total boxes to palletize")
    error: str | None = Field(
        None, description="Error message if in FAULT state"
    )


class ConfigResponse(BaseModel):
    """Configuration response."""

    success: bool
    message: str
    grid_size: str | None = None


class CommandResponse(BaseModel):
    """Generic command response."""

    success: bool
    message: str


# ============================================================================
# API Endpoints
# ============================================================================


@router.post("/configure", response_model=ConfigResponse)
async def configure_palletizer(
    config: PalletConfig, request: Request
) -> ConfigResponse:
    """Configure the palletizing operation.

    Sets up the grid dimensions, box size, and pallet origin.
    Can only be called when the palletizer is in IDLE state.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    configured = machine.configure(
        rows=config.rows,
        cols=config.cols,
        box_size_mm=(
            config.box_width_mm,
            config.box_depth_mm,
            config.box_height_mm,
        ),
        pallet_origin_mm=(
            config.pallet_origin_x_mm,
            config.pallet_origin_y_mm,
            config.pallet_origin_z_mm,
        ),
    )

    if not configured:
        raise HTTPException(
            status_code=409,
            detail="Palletizer can only be configured while in IDLE state",
        )

    return ConfigResponse(
        success=True,
        message="Palletizer configured successfully",
        grid_size=f"{config.rows}x{config.cols}",
    )


@router.post("/start", response_model=CommandResponse)
async def start_palletizer(request: Request) -> CommandResponse:
    """Start the palletizing sequence.

    Begins the pick-and-place cycle. The palletizer must be configured first.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    if not machine.begin():
        return CommandResponse(
            success=False, message="Palletizer was unable to begin."
        )

    return CommandResponse(
        success=True, message="Palletizer started successfully."
    )


@router.post("/stop", response_model=CommandResponse)
async def stop_palletizer(request: Request) -> CommandResponse:
    """Stop the palletizing sequence.

    Gracefully stops the operation and returns to IDLE state.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    if not machine.stop():
        return CommandResponse(
            success=False, message="Palletizer was unable to stop."
        )

    return CommandResponse(
        success=True, message="Palletizer stopped successfully."
    )


@router.post("/reset", response_model=CommandResponse)
async def reset_palletizer(request: Request) -> CommandResponse:
    """Reset from FAULT state.

    Clears the fault and returns to IDLE state.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    if not machine.reset():
        return CommandResponse(
            success=False, message="Palletizer was unable to reset."
        )

    return CommandResponse(
        success=True, message="Palletizer has reset successfully."
    )


@router.get("/status", response_model=StatusResponse)
async def get_status(request: Request) -> StatusResponse:
    """Get current palletizer status.

    Returns the current state, progress, and any error messages.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    return StatusResponse(
        state=machine.current_state.name,
        current_box=machine.context.current_box_index,
        total_boxes=machine.context.total_boxes,
        error=machine.context.error_message,
    )


@router.post("/vision/detect", response_model=CommandResponse)
async def simulate_vision_detection(
    detection: VisionDetection, request: Request
) -> CommandResponse:
    """Simulate a vision detection event.

    In a real system, this would come from the vision system.
    For this exercise, use this endpoint to simulate box detections.

    The coordinates are stored in the camera frame and transformed
    later when the pick step consumes the detection queue.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    if len(machine.detections) >= machine.context.total_boxes:
        return CommandResponse(success=False, message="Cannot add to grid.")

    machine.detections.append(
        {
            "x_mm": detection.x_mm,
            "y_mm": detection.y_mm,
            "z_mm": detection.z_mm,
            "yaw_deg": detection.yaw_deg or 0.0,
        }
    )

    return CommandResponse(
        success=True, message="Vision detection queued successfully."
    )


# ============================================================================
# Helper/Debug Endpoints (Optional)
# ============================================================================


@router.get("/debug/positions")
async def get_calculated_positions(request: Request) -> dict[str, Any]:
    """Debug endpoint: Get all calculated place positions.

    Useful for verifying grid calculations without running the full sequence.
    """
    machine = getattr(request.app.state, "palletizer_machine", None)
    if machine is None:
        raise HTTPException(
            status_code=503, detail="Palletizer state machine is not available"
        )

    positions = machine.context.place_positions
    if not positions and machine.context.total_boxes > 0:
        positions = calculate_place_positions(
            rows=machine.context.rows,
            cols=machine.context.cols,
            box_size_mm=machine.context.box_size_mm,
            pallet_origin_mm=machine.context.pallet_origin_mm,
        )

    return {
        "grid": {
            "rows": machine.context.rows,
            "cols": machine.context.cols,
            "total_boxes": machine.context.total_boxes,
        },
        "place_positions_mm": [
            {"x_mm": pos[0], "y_mm": pos[1], "z_mm": pos[2]}
            for pos in positions
        ],
    }


@router.post("/debug/transform")
async def test_transform(detection: VisionDetection) -> dict[str, Any]:
    """Debug endpoint: Test coordinate transformation.

    Transforms the input coordinates and returns both camera and robot frame
    values. Useful for verifying transformation math.
    """
    point_camera_mm = np.array(
        [detection.x_mm, detection.y_mm, detection.z_mm], dtype=float
    )
    point_robot_mm = camera_to_robot(point_camera_mm)
    recovered_camera_mm = robot_to_camera(point_robot_mm)

    return {
        "camera_frame_mm": {
            "x_mm": detection.x_mm,
            "y_mm": detection.y_mm,
            "z_mm": detection.z_mm,
            "yaw_deg": detection.yaw_deg or 0.0,
        },
        "robot_frame_mm": {
            "x_mm": point_robot_mm[0],
            "y_mm": point_robot_mm[1],
            "z_mm": point_robot_mm[2],
        },
        "camera_frame_roundtrip_mm": {
            "x_mm": recovered_camera_mm[0],
            "y_mm": recovered_camera_mm[1],
            "z_mm": recovered_camera_mm[2],
        },
    }
