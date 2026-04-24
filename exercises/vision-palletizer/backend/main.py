"""Vision Palletizer Backend.

========================

FastAPI application for coordinating UR5e robot palletizing operations.
"""

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from api.routes import router as palletizer_router
from fastapi import FastAPI
from palletizer.state_machine import PalletizerStateMachine
from robot.connection import RobotConnection
from robot.motion import MotionController

# Global robot connection instance
robot_connection: RobotConnection | None = None
motion_controller: MotionController | None = None
palletizer_machine: PalletizerStateMachine | None = None


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Manage application lifecycle - connect/disconnect robot."""
    global robot_connection, motion_controller, palletizer_machine

    # Startup: Initialize robot connection (will auto-reconnect later if needed)
    robot_connection = RobotConnection()
    motion_controller = MotionController(connection=robot_connection)
    palletizer_machine = PalletizerStateMachine(
        motion_controller=motion_controller
    )
    app.state.robot_connection = robot_connection
    app.state.motion_controller = motion_controller
    app.state.palletizer_machine = palletizer_machine
    connected = robot_connection.connect()

    if connected:
        print("✓ Robot connection established")
    else:
        print(
            "⚠ Robot not available yet - will auto-reconnect when robot "
            "is powered on"
        )
        print(
            "  → Power on the robot in PolyScope, then call /health to connect"
        )

    yield

    # Shutdown: Clean up robot connection
    if robot_connection:
        robot_connection.disconnect()
        print("✓ Robot connection closed")


app = FastAPI(
    title="Vision Palletizer API",
    description=(
        "API for controlling UR5e palletizing operations with "
        "vision-based picking"
    ),
    version="1.0.0",
    lifespan=lifespan,
)


# Include palletizer routes
app.include_router(palletizer_router, prefix="/palletizer", tags=["Palletizer"])


@app.get("/health")
async def health_check() -> dict[str, str]:
    """Check API and robot connection health.

    This endpoint also attempts to reconnect to the robot if disconnected,
    so you can use it to verify when the robot becomes available after
    powering it on in PolyScope.
    """
    robot_status = "disconnected"

    if robot_connection:
        # This will attempt to reconnect if not connected
        if robot_connection.check_and_reconnect():
            robot_status = "connected"

    return {
        "status": "healthy",
        "robot": robot_status,
    }


@app.get("/")
async def root() -> dict[str, str]:
    """API root - redirect to docs."""
    return {
        "message": "Vision Palletizer API",
        "docs": "/docs",
        "health": "/health",
    }


def get_robot_connection() -> RobotConnection | None:
    """Get the global robot connection instance."""
    return robot_connection


def get_motion_controller() -> MotionController | None:
    """Get the global motion controller instance."""
    return motion_controller


def get_palletizer_machine() -> PalletizerStateMachine | None:
    """Get the global palletizer state machine instance."""
    return palletizer_machine
