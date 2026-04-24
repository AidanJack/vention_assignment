# Vision-Palletizer

### Aidan Jackson


## Overview

This project is a **FastAPI-based backend** for controlling a robot using vision-based detections. It provides endpoints for transforming coordinates between camera and robot frames, as well as executing motion commands in a structured and safe way.

The system is designed to sit between:

* a **vision system** (providing detections in camera space)
* and a **rtde_control** (executing movements in robot space)

## Assumptions

* Robot frame origin is equivalent to world frame origin.
* Camera specification are given in base frame.
* Picking Positions always have a 'z' coordinate of '0.5'.


## API Endpoints

# Palletizer Control API Documentation

## Overview

This project exposes a **FastAPI-based control system** for a robotic palletizing machine.
It provides endpoints for:

* Configuring pallet layout
* Starting / stopping / resetting the state machine
* Simulating vision detections
* Inspecting runtime status
* Debugging coordinate transformations

The system is driven by a **state machine architecture**, ensuring safe transitions between IDLE, RUNNING, and FAULT states.

---

## API Endpoints

# Core Control Endpoints

---

## `POST /configure`

Configure the palletizing grid, box dimensions, and pallet origin.

### Requirements

* Must be in **IDLE** state

### Request

```json id="c1q9aa"
{
  "rows": 3,
  "cols": 4,
  "box_width_mm": 100,
  "box_depth_mm": 100,
  "box_height_mm": 120,
  "pallet_origin_x_mm": 0,
  "pallet_origin_y_mm": 0,
  "pallet_origin_z_mm": 0
}
```

### Response

```json id="xk2p9d"
{
  "success": true,
  "message": "Palletizer configured successfully",
  "grid_size": "3x4"
}
```

---

## `POST /start`

Start the palletizing sequence.

### Behavior

* Begins pick-and-place cycle
* Requires prior configuration

### Response

```json id="v9k3ld"
{
  "success": true,
  "message": "Palletizer started successfully."
}
```

---

## `POST /stop`

Gracefully stops the palletizing process and returns to IDLE.

### Response

```json id="p0w8ad"
{
  "success": true,
  "message": "Palletizer stopped successfully."
}
```

---

## `POST /reset`

Reset the system from FAULT state.

### Behavior

* Clears fault condition
* Returns system to IDLE

### Response

```json id="q8n2sd"
{
  "success": true,
  "message": "Palletizer has reset successfully."
}
```

---

## Status & Monitoring

---

## `GET /status`

Returns current state and progress of the palletizer.

### Response

```json id="s8d1aa"
{
  "state": "RUNNING",
  "current_box": 2,
  "total_boxes": 12,
  "error": null
}
```

---

# Vision Simulation

---

## `POST /vision/detect`

Simulates a vision detection event (used for testing without real camera input).

### Behavior

* Adds detection to internal queue
* Stored in **camera frame**
* Transformed later during execution

### Request

```json id="d8k1qp"
{
  "x_mm": 120.0,
  "y_mm": -45.0,
  "z_mm": 300.0,
  "yaw_deg": 15.0
}
```

### Response

```json id="m2x9zz"
{
  "success": true,
  "message": "Vision detection queued successfully."
}
```

---

# Debug Endpoints

---

## `GET /debug/positions`

Returns computed pallet placement positions.

### Response

```json id="z1k9aa"
{
  "grid": {
    "rows": 3,
    "cols": 4,
    "total_boxes": 12
  },
  "place_positions_mm": [
    { "x_mm": 0, "y_mm": 0, "z_mm": 0 },
    { "x_mm": 100, "y_mm": 0, "z_mm": 0 }
  ]
}
```

---

## `POST /debug/transform`

Tests coordinate transformation between camera and robot frames.

### Request

```json id="t9k2aa"
{
  "x_mm": 100.0,
  "y_mm": 50.0,
  "z_mm": 200.0,
  "yaw_deg": 0.0
}
```

### Response

```json id="r8m1bb"
{
  "camera_frame_mm": {
    "x_mm": 100.0,
    "y_mm": 50.0,
    "z_mm": 200.0,
    "yaw_deg": 0.0
  },
  "robot_frame_mm": {
    "x_mm": 120.0,
    "y_mm": -30.0,
    "z_mm": 180.0
  },
  "camera_frame_roundtrip_mm": {
    "x_mm": 100.0,
    "y_mm": 50.0,
    "z_mm": 200.0
  }
}
```

---

# Notes

## State Machine Rules

* `configure()` → only allowed in **IDLE**
* `start()` → requires configured system
* `stop()` → safe interruption at any time
* `reset()` → clears FAULT state only

---

## Vision Pipeline

1. Detection is received in **camera frame**
2. Converted to **robot frame during execution**
3. Used for pick-and-place cycle

## Tooling & Standards

### Code Quality

* **Ruff** for linting and formatting
* Enforced **80-character line limit**
* Google docstring format

### Editor Tools

* VSCode **Spell Checker extension**
* VSCode **charliermarsh.ruff**

## Usage

### 1. Run with Docker

Running for the first time:
```bash
docker-compose up -d
```

Once the container has been constructed the first time, it can be controlled using:
```bash
docker-compose start
docker-compose stop
```

### 2. Access

Simulator
```
http://localhost:6080/vnc.html
```

When attempting to run with the simulator, use the following steps to ensure proper configuration:
1. Enable remote operation using the button located to the upper right side of the screen.
2. Use the button in the lower left corner to open the power menu > Click "On" and "Start".

Docs

```
http://localhost:8000/docs
```

## Future Improvements

* Allow for 3D pallet grid configuration
* Connect unit test suite to Github Actions or equivalent