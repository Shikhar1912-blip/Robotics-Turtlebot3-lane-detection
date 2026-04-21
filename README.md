# TurtleBot3 Lane Detection — Indian LHT Simulation

A ROS 2 lane-following system for TurtleBot3 Burger running in Gazebo Classic on WSL2. The robot navigates a capsule-shaped two-lane track using a PID controller driven by HSV colour detection on a 182° fisheye camera feed.

---

## Environment

| Layer | Detail |
|-------|--------|
| Host OS | Windows 11 + WSL2 (Ubuntu 22.04 LTS) |
| ROS 2 | Humble Hawksbill |
| Simulator | Gazebo Classic 11 |
| Build type | `ament_python` (pure Python, no C++) |
| Display | WSLg (`:0`) for GUI window · Xvfb (`:1`) for camera rendering |

---

## Package Layout

```
~/turtlebot3_lane_ws/src/lane_detection/
├── lane_detection/
│   ├── lane_detector_node.py   # main perception + PID control loop
│   ├── diagnose_camera.py      # one-shot HSV diagnostic tool
│   └── respawn_robot.py        # delete + respawn robot at new pose
├── launch/
│   └── simulation.launch.py    # full simulation launch
├── worlds/
│   └── two_lane_circle.world   # capsule track SDF
├── package.xml
└── setup.py
```

---

## Quick Start

```bash
# Terminal 1 — build and launch
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_lane_ws
colcon build --packages-select lane_detection --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch lane_detection simulation.launch.py
```

```bash
# Terminal 2 — verify everything is running
source /opt/ros/humble/setup.bash && source ~/turtlebot3_lane_ws/install/setup.bash
ros2 topic hz /camera/image_raw          # expect ~15 Hz
ros2 topic echo /cmd_vel                 # non-zero angular.z = robot is steering
ros2 run rqt_image_view rqt_image_view   # subscribe to /lane_detection/debug_img
```

`--symlink-install` means Python source edits take effect immediately — no rebuild needed.

---

## Architecture

### Data Flow

```
/camera/image_raw  (~15 Hz, OV2710 2MP USB camera)
    │
    ▼
LaneDetectorNode.image_callback()
    ├─ Crop ROI: rows 35%–70% from top  (horizon band)
    ├─ BGR → HSV
    ├─ white_mask  (H:0-180, S:0-80,   V:160-255)
    ├─ yellow_mask (H:10-40, S:80-255, V:80-255)
    ├─ _compute_error() → pixel error (6 priority cases)
    ├─ PID controller  → /cmd_vel
    └─ _publish_debug() → /lane_detection/debug_img
```

### Launch Sequence

| Delay | Action |
|-------|--------|
| t = 0 s | Xvfb on `:1` (software GLX for camera sensors) |
| t = 0 s | `gzserver` on Xvfb · `gzclient` on WSLg `:0` · `robot_state_publisher` |
| t = 6 s | Spawn robot: `x ∈ [−1.5, 1.5]`, `y = 1.35`, `yaw = 0` (top straight) |
| t = 9 s | `lane_detector_node` |

### Error Computation — 6 Priority Cases

`error = target_x − image_center_x`. Positive → target right of centre → turn right (−ω).

| Priority | Visible markings | Target formula |
|----------|-----------------|----------------|
| 1 | Yellow + left white | `(left_x + yellow_x) / 2 + FISHEYE_OUTWARD_BIAS_PX` |
| 2 | Both white edges | `left_x + LANE_CENTER_OFFSET_FRAC × road_width + bias` |
| 3 | Left white only | `left_x − int(w × LEFT_EDGE_TARGET_FRAC)` |
| 4 | Yellow + right white | Mirror yellow → infer inner white → midpoint + bias |
| 5 | Yellow only | `yellow_x − int(w × YELLOW_TARGET_FRAC)` |
| 6 | Nothing | `valid = False` → SEARCHING state |

### State Machine

```
┌──────────────┐  valid detection     ┌─────────────┐
│  SEARCHING   │ ────────────────────▶ │  FOLLOWING  │
│ ω=0.25 rad/s │                      │ PID + speed │
│ linear = 0   │ ◀──────────────────── │   scaling   │
└──────────────┘  2 consecutive misses └─────────────┘
                  (~0.13 s at 15 Hz)
```

---

## Track Geometry

```
Shape: capsule (two 4.0 m straights + two 1.0 m-radius semicircular ends)
Semicircle centres: x = ±2.0 m

Radii:
  1.70 m  — outer white boundary
  1.35 m  — yellow dashed centre line / lane centre
  1.00 m  — inner white boundary / green island edge

Total track width: 0.70 m
Lane centre on straights: y = ±1.35 m
```

---

## ROS Interface

| Topic | Type | Direction | Hz |
|-------|------|-----------|-----|
| `/camera/image_raw` | `sensor_msgs/Image` | subscribed | ~15 |
| `/cmd_vel` | `geometry_msgs/Twist` | published | ~15 |
| `/lane_detection/debug_img` | `sensor_msgs/Image` | published | ~15 |

---

## Tuning Constants

All constants are module-level globals in `lane_detection/lane_detector_node.py`. Edit source directly — no rebuild needed with `--symlink-install`.

### PID Gains

| Constant | Value | Effect |
|----------|-------|--------|
| `KP` | 0.008 | Proportional — increase for tighter curve tracking |
| `KI` | 0.0003 | Integral — eliminates steady-state offset; anti-windup at 150 px·s |
| `KD` | 0.001 | Derivative — kept low due to sensor jitter at 15 Hz |
| `DERIVATIVE_ALPHA` | 0.3 | Low-pass filter on derivative (`0` = off, `1` = raw) |

### Speed

| Constant | Value | Effect |
|----------|-------|--------|
| `LINEAR_MAX` | 0.10 m/s | Forward speed at zero angular demand |
| `LINEAR_MIN` | 0.04 m/s | Forward speed at max angular demand |
| `ANGULAR_MAX` | 0.6 rad/s | Hard clamp on angular output |

### Detection & ROI

| Constant | Value | Effect |
|----------|-------|--------|
| `ROI_TOP_FRAC` | 0.35 | Scan starts at 35% from top (cuts sky) |
| `ROI_BOT_FRAC` | 0.70 | Scan ends at 70% from top (cuts green island) |
| `MIN_CONTOUR_AREA` | 20 px² | Rejects speckle |
| `MAX_CONTOUR_AREA` | 2500 px² | Rejects sky/background blobs |
| `LANE_LOST_TTL` | 2 frames | Consecutive misses before SEARCHING |

### Fisheye Compensation

| Constant | Value | Effect |
|----------|-------|--------|
| `FISHEYE_OUTWARD_BIAS_PX` | 10 px | Counters 182° lens compression at image edges (Cases 1–3) |
| `LANE_CENTER_OFFSET_FRAC` | 0.35 | Fraction of road width from inner edge to target (Case 2) |
| `LEFT_EDGE_TARGET_FRAC` | 0.30 | Target image-fraction for inner-white edge (Case 3) |
| `YELLOW_TARGET_FRAC` | 0.70 | Target image-fraction for yellow centre (Case 5) |

---

## HSV Diagnostic Tool

Run while simulation is live to inspect colour masks:

```bash
ros2 run lane_detection diagnose_camera
```

Saves to `/tmp/`: `diag_raw.png`, `diag_white_mask.png`, `diag_yellow_mask.png`, `diag_overlay.png`. Prints per-channel HSV stats and pixel counts.

---

## Respawn Robot

Respawn without restarting the simulation:

```bash
ros2 run lane_detection respawn_robot
```

---

## Pre-commit Checklist

```
□ ros2 topic hz /camera/image_raw  → ~15 Hz
□ ros2 topic echo /cmd_vel         → non-zero angular.z on curves
□ rqt_image_view /lane_detection/debug_img  → coloured overlays visible
□ Robot completes at least one full lap without entering SEARCHING
□ No ERROR or WARN messages in lane_detector terminal output
```

---

## Dependencies

```
rclpy · sensor_msgs · geometry_msgs · cv_bridge
gazebo_ros · robot_state_publisher · turtlebot3_gazebo · usb_cam
OpenCV 4.x · NumPy
```
