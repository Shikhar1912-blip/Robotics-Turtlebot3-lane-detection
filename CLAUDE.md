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
ros2 topic hz /camera/image_raw          # expect ~15 Hz (OV2710 USB camera)
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
/camera/image_raw  (~15 Hz, OV2710 2MP USB camera)
    │
    ▼
LaneDetectorNode.image_callback()
    ├─ Crop ROI: rows 35%–70% from top  (horizon band where markings appear)
    ├─ BGR → HSV
    ├─ white_mask  (H:0-180, S:0-80,   V:160-255)  → left_x, right_x
    ├─ yellow_mask (H:10-40, S:80-255, V:80-255)   → yellow_x
    ├─ _compute_error() → pixel error
    ├─ PID controller  → /cmd_vel
    └─ _publish_debug() → /lane_detection/debug_img
```

### Launch Sequence (`simulation.launch.py`)

| Delay | Action |
|-------|--------|
| t=0s | Xvfb on `:1`, gzserver on Xvfb (needs GLX for camera), gzclient on `:0` (WSLg GUI), robot_state_publisher |
| t=6s | Spawn robot on top straight of capsule track: x ∈ [−1.5, 1.5], y = 1.35, yaw = 0 |
| t=9s | Start `lane_detector_node` |

`pkg_path` is resolved via `get_package_share_directory('lane_detection')` — `setup.py` installs `worlds/` into the share directory, so this works correctly from the install space.

**Hardware (physical robot):** `ros2 launch lane_detection hardware.launch.py` — starts `usb_cam_node` on `/dev/video0` at 15 Hz + `lane_detector_node`. Accepts `device`, `width`, `height`, `framerate` arguments.

### Road Geometry (`worlds/two_lane_circle.world`) — Capsule (Stadium) Track

```
Shape: two 4.0 m straight sections + two 1.0 m-radius semicircular ends
Semicircle centres: x = ±2.0 m

Radii from each semicircle centre:
  1.70 m  ── outer white boundary (outer edge)
  1.64 m  ── road surface outer edge
  1.35 m  ── yellow dashed centre line  /  lane centre
  1.06 m  ── road surface inner edge
  1.00 m  ── inner white boundary / green island edge

Total track width: 0.70 m   (inner to outer edge)
Lane centre on straights: y = ±1.35 m
```

Robot targets the **inner (left) lane** — Indian LHT.

Design constraints:
- Lane width / robot width ≈ 0.70 / 0.178 ≈ 3.9 (spec ≥ 4:1, marginal)
- Curve radius / robot width ≈ 1.00 / 0.178 ≈ 5.6 (spec ≥ 5:1 ✓)

### Error Computation — 6 Priority Cases (`_compute_error`)

`error = target_x − image_center_x`. Positive → target is right of center → turn right (−ω).

| Priority | Visible markings | How target is computed |
|----------|-----------------|----------------------|
| 1 | Yellow + left white | `(left_x + yellow_x) / 2 + FISHEYE_OUTWARD_BIAS_PX` |
| 2 | Both white edges | `left_x + LANE_CENTER_OFFSET_FRAC × road_width + FISHEYE_OUTWARD_BIAS_PX` |
| 3 | Left white only | Steer inner edge to `LEFT_EDGE_TARGET_FRAC` (0.30) of image width |
| 4 | Yellow + right white | Mirror yellow → infer inner white → midpoint + bias |
| 5 | Yellow only | Steer yellow to `YELLOW_TARGET_FRAC` (0.70) of image width |
| 6 | Nothing | Not valid → SEARCHING state |

`FISHEYE_OUTWARD_BIAS_PX = 10 px` is added in Cases 1, 2, and 4 to compensate for
the 182° fisheye lens compressing objects near the image edges.

### State Machine

- **FOLLOWING**: Full PID controller. Linear speed scales from `LINEAR_MAX` (0.10 m/s)
  at zero angular demand to `LINEAR_MIN` (0.04 m/s) at max angular demand.
- **SEARCHING**: Zero linear velocity, rotate at `SEARCH_OMEGA` (0.25 rad/s) until a
  lane marking is detected. Triggers after `LANE_LOST_TTL` (2) consecutive invalid
  frames (~0.13 s at 15 Hz). Integral term is reset on entry to prevent windup.
- **Grace period**: When the lane is first lost (counter < TTL), the robot coasts
  forward with `error = 0` (no correction) for up to ~0.067 s per frame before switching to SEARCHING.

### Contour Filtering

Blobs are filtered by area: `MIN_CONTOUR_AREA (20 px²) < area < MAX_CONTOUR_AREA (2500 px²)`.
Morphological OPEN (3×3 ellipse) removes speckle — deliberately **no CLOSE**, which was
merging sky blobs with road markings into one giant contour.

## Tuning Constants (all in `lane_detector_node.py`, module-level globals)

All tuning is done by editing source directly — none are exposed as ROS parameters.

### Speed & limits

| Constant | Value | Effect |
|----------|-------|--------|
| `LINEAR_MAX` | 0.10 m/s | Forward speed at zero angular demand |
| `LINEAR_MIN` | 0.04 m/s | Forward speed at max angular demand |
| `ANGULAR_MAX` | 0.6 rad/s | Hard clamp on angular output |
| `SEARCH_OMEGA` | 0.25 rad/s | Rotation speed when searching |

### PID gains

| Constant | Value | Effect |
|----------|-------|--------|
| `KP` | 0.008 | Proportional; increase for tighter curve tracking, decrease if oscillating |
| `KI` | 0.0003 | Integral; eliminates steady-state offset on curves; anti-windup at `INTEGRAL_MAX = 150 px·s` |
| `KD` | 0.001 | Derivative; kept low — 1.7 Hz makes raw derivative very noisy |
| `DERIVATIVE_ALPHA` | 0.3 | Low-pass filter on derivative (`0 = off`, `1 = raw`) |

### Detection & ROI

| Constant | Value | Effect |
|----------|-------|--------|
| `ROI_TOP_FRAC` / `ROI_BOT_FRAC` | 0.35 / 0.70 | Horizon band scanned for markings |
| `MIN_CONTOUR_AREA` | 20 px² | Rejects speckle |
| `MAX_CONTOUR_AREA` | 2500 px² | Rejects sky/background blobs |
| `LANE_LOST_TTL` | 2 frames | Consecutive misses before SEARCHING (~0.13 s at 15 Hz) |

### Fisheye compensation

| Constant | Value | Effect |
|----------|-------|--------|
| `FISHEYE_OUTWARD_BIAS_PX` | 10 px | Pushes lane-target outward to counter 182° lens compression |
| `LANE_CENTER_OFFSET_FRAC` | 0.35 | Fraction of road width from inner edge to target (Case 2) |
| `LEFT_EDGE_TARGET_FRAC` | 0.30 | Target image-fraction for inner-white edge (Case 5) |
| `YELLOW_TARGET_FRAC` | 0.70 | Target image-fraction for yellow centre (Case 4) |

## Subdirectory CLAUDE.md files

| Path | Covers |
|------|--------|
| `lane_detection/CLAUDE.md` | All node files: detector, diagnose, respawn — full constant reference, case tables, debug guide |
| `launch/CLAUDE.md` | Launch sequence, env vars, spawn logic, failure modes |

## .claude/ reference files (auto-loaded by Claude Code)

| Path | Covers |
|------|--------|
| `.claude/rules/code-style.md` | Python conventions, naming, comment style, forbidden patterns |
| `.claude/rules/control-logic.md` | Full perception → error → PID pipeline with formulas |
| `.claude/rules/development-rules.md` | Build rules, forbidden changes, git conventions, test checklist |
| `.claude/rules/tuning-guide.md` | Step-by-step HSV, ROI, contour, PID, speed, fisheye tuning |
| `.claude/specs/track-geometry.md` | Full capsule track dimensions, dash layout, spawn formula |
| `.claude/specs/hardware.md` | Robot specs, camera properties, track constraints |
| `.claude/specs/ros-interface.md` | Topics, services, plugins, TF tree, CLI commands |
| `.claude/specs/tech-stack.md` | OS, ROS, Gazebo, Python libraries, workspace layout |
