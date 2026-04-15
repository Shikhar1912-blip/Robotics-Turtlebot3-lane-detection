# launch/ — CLAUDE.md

This directory contains the single launch file that starts the complete lane-detection simulation.

## Files

| File | Purpose |
|------|---------|
| `simulation.launch.py` | Starts Xvfb, Gazebo (server + client), robot_state_publisher, spawns robot, starts lane detector |

---

## simulation.launch.py

### What it does

Brings up the full simulation stack in a fixed sequence with timed delays to let each
component finish initialising before the next depends on it.

### Launch sequence

| Delay | Action | Why delayed |
|-------|--------|-------------|
| t = 0 s | Start **Xvfb** on `:1` (virtual X11 display) | Must be ready before gzserver |
| t = 0 s | Start **gzserver** (physics + camera rendering) | Loads world file, brings up ROS services |
| t = 5 s | Start **gzclient** (3-D GUI window) | Needs gzserver fully up to connect |
| t = 0 s | Start **robot_state_publisher** | Publishes TF from URDF; safe to start immediately |
| t = 6 s | **Spawn robot** via `spawn_entity.py` | Needs gzserver + `/spawn_entity` service ready |
| t = 9 s | Start **lane_detector_node** | Needs robot spawned + `/camera/image_raw` publishing |

### Robot spawn (capsule track)

```python
spawn_x   = random.uniform(-1.5, 1.5)   # random along top straight (4 m long, clear of curves)
spawn_y   = 1.35                          # top-straight lane centre
spawn_yaw = 0.0                           # facing +x (direction of travel on top straight)
```

The robot always starts on the **top straight** of the capsule track, facing right (+x).

**Gazebo Reset World does NOT re-randomise the spawn position** — the spawn coordinates are
computed once at Python import time and frozen. To get a new random position without
restarting the launch, use:

```bash
ros2 run lane_detection respawn_robot
```

> Note: `respawn_robot.py` still contains old circular-track geometry — update it to match
> the capsule track before using it.

### Environment variables

#### gzserver (runs on Xvfb `:1`)

| Variable | Value | Purpose |
|----------|-------|---------|
| `DISPLAY` | `:1` (Xvfb) | Forces OGRE to use the virtual display with software GLX |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Mesa CPU rasteriser — no GPU needed (WSL2 has no OpenGL ICD) |
| `OGRE_RTT_MODE` | `Copy` | Prevents OGRE render-to-texture crash under Mesa |
| `GAZEBO_MODEL_PATH` | turtlebot3_gazebo models dir | Lets Gazebo find `turtlebot3_burger_cam` |
| `GAZEBO_IP` | `127.0.0.1` | Locks Gazebo transport to localhost (avoids WSL2 network issues) |

#### gzclient (runs on WSLg `:0`)

| Variable | Value | Purpose |
|----------|-------|---------|
| `DISPLAY` | `:0` (WSLg/XWayland) | Renders the 3-D window on the Windows desktop |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Same Mesa fallback |
| `OGRE_RTT_MODE` | `Copy` | Same crash fix |
| `GAZEBO_IP` | `127.0.0.1` | Matches gzserver |

### Xvfb (`_start_xvfb`)

- Launches `Xvfb :1` with software GLX and RANDR extensions enabled.
- Polls with `xdpyinfo` every 0.5 s (up to 5 s) to confirm the server started.
- If `:1` is already running (e.g. from a previous launch), it skips re-launching.
- gzserver camera sensors **require** a GLX-capable display. WSLg/XWayland (`:0`) lacks
  hardware GLX, so Xvfb + Mesa provides the required software GLX path.

### File paths

```python
pkg_path   = ~/turtlebot3_lane_ws/src/lane_detection   # hardcoded — references worlds/ dir
world_file = pkg_path/worlds/two_lane_circle.world
robot_sdf  = turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf
robot_urdf = turtlebot3_gazebo/urdf/turtlebot3_burger_cam.urdf
```

`pkg_path` is hardcoded (not from the install space) because the world file must be
resolved at launch time, before `colcon` install-space symlinks are evaluated.

### Verify the simulation is working

```bash
# In a second terminal (source both setup files first):
ros2 topic hz /camera/image_raw          # expect ~1.7 Hz (software rendering)
ros2 topic echo /cmd_vel                 # non-zero angular.z = robot is steering
ros2 run rqt_image_view rqt_image_view   # subscribe to /lane_detection/debug_img
```

### Common failure modes

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `/camera/image_raw` never published | Xvfb didn't start or gzserver crashed | Check `DISPLAY=:1 xdpyinfo` |
| gzclient shows blank window | gzserver not ready when gzclient connected | Increase gzclient delay (currently 5 s) |
| Robot spawns outside track | Wrong spawn coordinates | Check `spawn_x/y/yaw` in launch file |
| `spawn_entity` service timeout | gzserver still loading world | Increase spawn delay (currently 6 s) |
| Camera renders green blob only | ROI cuts wrong region | Tune `ROI_TOP_FRAC`/`ROI_BOT_FRAC` in `lane_detector_node.py` |
