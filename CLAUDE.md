# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

- ROS 2 Humble + Gazebo 11 (Classic) in **WSL2 on Windows**
- `ament_python` build type — no C++ compilation, pure Python package
- Xvfb provides a virtual X11 display (`:1`) so `gzserver` camera sensors work via software GLX; `gzclient` uses WSLg display (`:0`) for the visible window

## Build & Run

```bash
# Source ROS (required in every terminal)
source /opt/ros/humble/setup.bash

# Build (from workspace root)
cd ~/turtlebot3_lane_ws
colcon build --packages-select lane_detection --symlink-install
source install/setup.bash

# Launch full simulation
export TURTLEBOT3_MODEL=burger
ros2 launch lane_detection simulation.launch.py
```

`--symlink-install` means edits to Python source files take effect immediately without rebuilding.

### Verify in a second terminal

```bash
source /opt/ros/humble/setup.bash && source ~/turtlebot3_lane_ws/install/setup.bash
ros2 topic hz /camera/image_raw          # expect ~1.7 Hz (software rendering)
ros2 topic echo /cmd_vel                 # non-zero values = robot steering
ros2 run rqt_image_view rqt_image_view   # view /lane_detection/debug_img
```

### Diagnose HSV thresholds (run while simulation is live)

```bash
ros2 run lane_detection diagnose_camera
# Saves /tmp/diag_raw.png, diag_white_mask.png, diag_yellow_mask.png, diag_overlay.png
# Prints per-channel HSV stats and pixel-count reports, then exits
```

## Architecture

### Data Flow

```
/camera/image_raw  (~1.7 Hz)
    │
    ▼
LaneDetectorNode.image_callback()
    ├─ Crop ROI: rows 35%–70% from top  (horizon band where markings appear)
    ├─ BGR → HSV
    ├─ white_mask  (H:0-180, S:0-80,   V:160-255)  → left_x, right_x
    ├─ yellow_mask (H:10-40, S:80-255, V:80-255)   → yellow_x
    ├─ _compute_error() → pixel error
    ├─ PD controller  → /cmd_vel
    └─ _publish_debug() → /lane_detection/debug_img
```

### Launch Sequence (`simulation.launch.py`)

| Delay | Action |
|-------|--------|
| t=0s | Xvfb on `:1`, gzserver on Xvfb (needs GLX for camera), gzclient on `:0` (WSLg GUI), robot_state_publisher |
| t=6s | Spawn robot at random position on road ring (radius 2.95–3.95 m, tangential yaw) |
| t=9s | Start `lane_detector_node` |

`pkg_path` in the launch file is **hardcoded** as `~/turtlebot3_lane_ws/src/lane_detection` (not from the install space) to reference the `worlds/` directory.

### Road Geometry (`worlds/two_lane_circle.world`)

```
4.2m  ── white outer boundary
3.45m ── yellow dashed center line
2.85m ── white inner boundary
2.75m ── green inner island
```

Robot targets the **inner (left) lane** — Indian LHT. Lane center ≈ 3.16 m radius.

### Error Computation — 6 Priority Cases (`_compute_error`)

`error = target_x − image_center_x`. Positive → target is right of center → turn right (−ω).

| Case | Visible markings | How target is computed |
|------|-----------------|----------------------|
| 1 | Both white edges | `target = (left_x + road_mid) / 2` |
| 2 | Yellow + left white | `target = (left_x + yellow_x) / 2` |
| 3 | Yellow + right white | Mirror yellow to infer inner white, then case 2 |
| 4 | Yellow only | Steer yellow to 75% of image width |
| 5 | Left white only | Steer left edge to 25% of image width |
| 6 | Nothing | Not valid → SEARCHING state |

### State Machine

- **FOLLOWING**: `angular_z = -(KP * error + KD * d_error/dt)`. Linear speed scales from `LINEAR_MAX` (0.12 m/s) at zero angular demand to `LINEAR_MIN` (0.04 m/s) at max angular demand.
- **SEARCHING**: Zero linear, rotate at `SEARCH_OMEGA` (0.25 rad/s) until a lane is detected. Triggers after `LANE_LOST_TTL` (2) consecutive invalid frames (~1.2 s).

### Contour Filtering

Blobs are filtered by area: `MIN_CONTOUR_AREA (20 px²) < area < MAX_CONTOUR_AREA (2500 px²)`. This rejects noise (< 20) and sky/background blobs (> 10,000 px²). Morphological OPEN (3×3 ellipse) removes speckle — deliberately no CLOSE, which was merging sky blobs with road markings.

## Tuning Constants (all in `lane_detector_node.py`, module-level globals)

All tuning is done by editing source directly — none of these are exposed as ROS parameters.

| Constant | Default | Effect |
|----------|---------|--------|
| `KP` | 0.006 | Proportional gain; reduce if robot oscillates |
| `KD` | 0.002 | Derivative gain; smooths oscillations |
| `LINEAR_MAX` / `LINEAR_MIN` | 0.12 / 0.04 m/s | Forward speed range |
| `ROI_TOP_FRAC` / `ROI_BOT_FRAC` | 0.35 / 0.70 | Horizon band scanned for markings |
| `MAX_CONTOUR_AREA` | 2500 px² | Upper bound filters sky blobs |
| `LANE_LOST_TTL` | 2 frames | Misses before SEARCHING (~1.2 s at 1.7 Hz) |
| `SEARCH_OMEGA` | 0.25 rad/s | Rotation speed when searching |
