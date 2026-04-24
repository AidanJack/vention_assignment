# Vision Palletizer: Coordinate Transformation & Robot Control

Build a backend service that coordinates a **Universal Robots (UR5e)** to pick items from a vision-defined location and place them into a **configurable grid pattern**.

**Core focus areas:** Coordinate frame transformations, state machine design, and clean API implementation.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Technical Specifications](#technical-specifications)
- [Requirements](#requirements)
- [Acceptance Criteria](#acceptance-criteria)
- [Project Structure](#project-structure)
- [Resources](#resources)
- [Submission](#submission)

---

## Quick Start

### Prerequisites

- Docker Desktop running
- ~4GB free disk space (for URSim image)

> **Apple Silicon (M1/M2/M3) Note:** URSim is an x86 image that runs via emulation. You may see a platform warning — this is normal. The simulator will work but may be slower than on Intel machines.

### 1. Launch the Environment

```bash
docker-compose up -d
```

This starts:

| Service | Access |
|---------|--------|
| URSim (PolyScope UI) | http://localhost:6080/vnc.html |
| Backend API | http://localhost:8000/docs |

### 2. Power On the Robot

1. Open PolyScope at http://localhost:6080/vnc.html
2. Click the red button in the bottom-left corner
3. Click **ON** → **START** to enable the robot

### 3. Verify Connection

```bash
curl http://localhost:8000/health
```

---

## Technical Specifications

### Camera Mounting

| Parameter | Value |
|-----------|-------|
| Position (X, Y, Z) | 500mm, 300mm, 800mm |
| Orientation (Roll, Pitch, Yaw) | 15°, -10°, 45° |
| Rotation convention | Intrinsic rotations: Z → Y → X |
| Optical axis | Camera Z points toward the scene |

### Vision System

The "camera" in this exercise represents an upstream vision system that detects boxes and reports their positions in the **camera coordinate frame**.

A mock vision output is provided in `backend/data/camera_detections.json`:

```json
{
  "detections": [
    {"x_mm": 50.0, "y_mm": -30.0, "z_mm": 0.0, "yaw_deg": 0.0},
    {"x_mm": 120.0, "y_mm": 45.0, "z_mm": 0.0, "yaw_deg": 15.0},
    ...
  ]
}
```

**Your task:** Load detections from this file and transform each position from camera frame to robot base frame before commanding the robot to pick.

> The JSON structure can be replaced with any arbitrary data following the same format — your implementation should handle different detection sets.

### URSim Ports

| Port | Purpose |
|------|---------|
| 6080 | PolyScope UI (VNC) |
| 29999 | Dashboard |
| 30004 | RTDE |
| 30001 | Primary Interface |

---

## Requirements

### Mandatory

#### 1. Coordinate Transformation

Transform coordinates from the camera frame to the robot base frame using the camera mounting specifications above.

#### 2. Palletizing Grid

Calculate TCP target positions for placing boxes in an N×M grid pattern on a pallet.

#### 3. Motion Control

Execute safe pick-and-place movements on the UR5e:

- **Pick sequence:** approach from above → descend → grip → retract
- **Place sequence:** approach from above → descend → release → retract
- Use approach heights >= 50mm above pick/place targets

#### 4. State Machine

Manage the palletizing operation lifecycle using [`vention-state-machine`](https://pypi.org/project/vention-state-machine/).

**States:** `IDLE` → `HOMING` → `PICKING` → `PLACING` → `IDLE` (with `FAULT` for error handling)

A bootstrapped state machine implementation is provided in `backend/palletizer/state_machine.py`. Extend it with your business logic.

### Bonus

#### 5. REST API

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/palletizer/configure` | Set grid dimensions, box size, pallet origin |
| POST | `/palletizer/start` | Begin the palletizing sequence |
| POST | `/palletizer/stop` | Stop operation and return to IDLE |
| GET | `/palletizer/status` | Return current state and progress |
| POST | `/palletizer/vision/detect` | Simulate vision detection with camera-frame coordinates |

---

## Acceptance Criteria

### Mandatory (must complete)

| # | Criteria |
|---|----------|
| 1 | Camera-to-robot coordinate transformation correctly applies rotation and translation |
| 2 | Grid positions calculated from configurable rows, columns, box dimensions, pallet origin, and spacing |
| 3 | Grid fills row-by-row starting from the origin, with positions accounting for box size |
| 4 | Robot moves to home position on startup/reset |
| 5 | Pick sequence: approach → descend → grip → retract |
| 6 | Place sequence: approach → descend → release → retract |
| 7 | Motions use approach heights >= 50mm above pick/place targets |
| 8 | State machine implements: `IDLE → HOMING → PICKING → PLACING → IDLE` |
| 9 | `FAULT` state with recovery and graceful stop |

### Bonus (optional enhancements)

| # | Criteria |
|---|----------|
| 10 | REST API endpoints for configure, start, stop, status, vision detect |
| 11 | Gripper rotation adjusts for rotated boxes detected by vision |
| 12 | Target positions validated against robot workspace before moving |
| 13 | Unit tests verify transformation math with known input/output pairs |
| 14 | Calculated robot positions logged or visualized for debugging |

---

## Project Structure

```
vision-palletizer/
├── docker-compose.yml
├── README.md
└── backend/
    ├── main.py
    ├── requirements.txt
    ├── pytest.ini
    ├── data/
    │   └── camera_detections.json  # Mocked vision system output
    ├── api/
    │   └── routes.py          # REST API endpoints
    ├── robot/
    │   ├── connection.py      # UR5e connection management
    │   └── motion.py          # Motion control logic
    ├── transforms/
    │   └── coordinate.py      # Camera-to-robot transformations
    ├── palletizer/
    │   ├── grid.py            # Grid position calculations
    │   └── state_machine.py   # Operation lifecycle management (bootstrapped)
    └── tests/
        └── test_state_machine.py  # State machine unit tests
```

---

## Resources

### Libraries

| Library | Purpose |
|---------|---------|
| [`ur_rtde`](https://sdurobotics.gitlab.io/ur_rtde/) | Real-time data exchange with UR robots |
| [`vention-state-machine`](https://pypi.org/project/vention-state-machine/) | State machine implementation |

---

## Submission

1. Push your solution to a **Git repository** (GitHub/GitLab)
2. Document your approach and any assumptions made
3. **Required:** Screen recording or GIF showing a 2×2 palletizing sequence in URSim

### Evaluation Weights

| Criteria | Weight |
|----------|--------|
| Correct coordinate transformation math | 35% |
| Working palletizing sequence in URSim | 35% |
| Code quality and architecture | 15% |
| Bonus features | 15% |

---

## Questions?

If you have any questions regarding the hardware specs or the Vention ecosystem, please reach out to the MachineApps team.
