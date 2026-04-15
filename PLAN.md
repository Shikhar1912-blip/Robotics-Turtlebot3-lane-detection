# TurtleBot3 Lane Detection — Project Plan

## Objective
Simulate a TurtleBot3 Burger robot that spawns **randomly** on a two-lane circular road
and autonomously navigates to the **left lane** (Indian Left-Hand Traffic rules), then
follows it continuously. Runs on ROS 2 Humble + Gazebo 11 (Classic) in WSL2 on Windows.

---

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                         │
│                                                              │
│   two_lane_circle.world  ←  custom circular 2-lane road      │
│   turtlebot3_burger_cam  ←  burger with fisheye camera       │
│   (spawns at random pos on road, tangential orientation)     │
└──────────────────────────┬──────────────────────────────────┘
                           │ /camera/image_raw
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                  lane_detector_node.py                       │
│                                                              │
│  1. HSV thresholding → white mask + yellow mask              │
│  2. ROI = bottom 40% of camera frame                         │
│  3. Find left white edge, right white edge, yellow centroid  │
│  4. Compute error = left_lane_center_x − image_center_x      │
│  5. P-controller: angular_z = −Kp × error                    │
│  6. State: SEARCHING (rotate) or FOLLOWING (drive+steer)     │
└──────────────────────────┬──────────────────────────────────┘
                           │ /cmd_vel
                           ▼
                    TurtleBot3 drives
                    in the LEFT lane
```

---

## Road Layout (two_lane_circle.world)

```
Radius:  4.2m  ── white outer boundary (10cm visible strip)
         4.1m  ── dark road surface outer edge
         3.45m ── yellow dashed center line (8 dashes × 45cm)
         2.85m ── white inner boundary (10cm visible strip)
         2.75m ── green inner island starts

Lane widths (~1.25m total road):
  LEFT  lane (inner) : 2.85m → 3.475m  center ≈ 3.16m  ← robot targets this
  RIGHT lane (outer) : 3.475m → 4.1m   center ≈ 3.79m
```

---

## Lane Detection Algorithm

| Priority | Visible markings | How to find left-lane center |
|---|---|---|
| 1 | Both white edges | `road_mid = (L+R)/2`, `target = (L+road_mid)/2` |
| 2 | Yellow + left white | `target = (L + yellow_x) / 2` |
| 3 | Yellow + right white | Mirror to infer outer white, then Case 2 |
| 4 | Yellow only | Steer yellow to 75% of image width |
| 5 | Left white only | Steer left edge to 25% of image width |
| 6 | Nothing | Enter SEARCHING state |

**Error signal:** `error = target_x − image_center_x`
**Control law:** `angular_z = −Kp × error` (Kp = 0.005)

---

## State Machine

```
         ┌────────────────┐
         │   SEARCHING    │  No lanes visible for 10+ frames
         │  rotate 0.4    │  ──────────────────────────────→
         │  rad/s CCW     │
         └────────┬───────┘
                  │ lanes detected
                  ▼
         ┌────────────────┐
         │   FOLLOWING    │  linear_x = 0.15 m/s
         │  P-controller  │  angular_z = −Kp × error
         └────────────────┘
```

---

## Files Changed

| File | Action | Description |
|---|---|---|
| `lane_detection/lane_detector_node.py` | **CREATED** | Main detection + control node |
| `launch/simulation.launch.py` | **REPLACED** | burger_cam SDF, delays, lane_detector |
| `setup.py` | **MODIFIED** | Added `lane_detector_node` entry point |
| `package.xml` | **MODIFIED** | Added exec_depends |
| `worlds/two_lane_circle.world` | **MODIFIED** | Yellow dashes widened + 16 total |

---

## How to Run

### Prerequisites (Windows)
1. Start **VcXsrv** (or another X11 server) with "Disable access control" checked
2. Open WSL2 terminal

### Build
```bash
cd ~/turtlebot3_lane_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lane_detection --symlink-install
source install/setup.bash
```

### Launch
```bash
ros2 launch lane_detection simulation.launch.py
```

Gazebo opens → robot spawns at random position → lane_detector starts → robot finds
left lane and follows it.

### Verify (second terminal)
```bash
source /opt/ros/humble/setup.bash && source ~/turtlebot3_lane_ws/install/setup.bash
ros2 topic hz /camera/image_raw    # expect ~30 Hz
ros2 topic echo /cmd_vel           # non-zero values = robot steering
ros2 run rqt_image_view rqt_image_view  # view camera feed
```

---

## Tuning Guide

| Parameter | Default | Effect |
|---|---|---|
| `KP` | 0.005 | Higher = sharper turns; if robot oscillates, reduce |
| `LINEAR_SPEED` | 0.15 m/s | Reduce to 0.10 if robot can't track curves |
| `ROI_TOP_FRAC` | 0.60 | Move toward 0.70 to look closer to robot feet |
| `SEARCH_OMEGA` | 0.4 rad/s | How fast robot spins when lost |
| `LANE_LOST_TTL` | 10 frames | Increase for smoother state transitions |

---

## Next Steps (after simulation verified)
1. 
2. Add speed scaling in curves (reduce speed when |angular_z| is high)
3. Transfer to physical TurtleBot3 (swap Gazebo camera topic for real camera)
4. Calibrate HSV thresholds for real-world lighting conditions
