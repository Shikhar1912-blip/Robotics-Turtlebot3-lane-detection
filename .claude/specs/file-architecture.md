# Robotics Turtlebot3 Lane Detection - Directory Architecture

## Project Structure

```
Robotics-Turtlebot3-lane-detection/      ← ROS 2 ament_python package root
│                                           (also the colcon workspace source dir)
│
├── lane_detection/                       # Python package — all node source files
│   ├── __init__.py
│   ├── lane_detector_node.py             # Main lane-following node (PID, state machine, HSV)
│   ├── diagnose_camera.py                # One-shot HSV diagnostic tool
│   └── respawn_robot.py                  # Robot respawn / recovery node
│
├── launch/
│   └── simulation.launch.py             # Full sim launch: Xvfb, gzserver, gzclient,
│                                        #   robot_state_publisher, spawn, lane_detector
│
├── worlds/
│   └── two_lane_circle.world            # Gazebo world — capsule track with lane markings
│
├── resource/
│   └── lane_detection                   # ament index marker file (do not edit)
│
├── .claude/
│   ├── rules/
│   │   ├── code-style.md                # Python conventions, naming, forbidden patterns
│   │   ├── control-logic.md             # Perception → error → PID pipeline with formulas
│   │   ├── development-rules.md         # Build rules, git conventions, test checklist
│   │   └── tuning-guide.md              # Step-by-step HSV, ROI, PID, speed tuning
│   │
│   └── specs/
│       ├── file-architecture.md         # This file — directory structure documentation
│       ├── hardware.md                  # Robot specs, camera properties, track constraints
│       ├── ros-interface.md             # Topics, services, plugins, TF tree, CLI commands
│       ├── tech-stack.md                # OS, ROS, Gazebo, Python libraries, workspace layout
│       └── track-geometry.md            # Full capsule track dimensions, dash layout, spawn formula
│
├── CLAUDE.md                            # Top-level Claude Code instructions (auto-loaded)
├── lane_detection/CLAUDE.md             # Node-level Claude instructions (detector, diagnose, respawn)
├── launch/CLAUDE.md                     # Launch sequence, env vars, spawn logic, failure modes
├── PLAN.md                              # Project plan / design notes
├── plan2.md                             # Secondary plan / iteration notes
├── package.xml                          # ROS 2 package manifest (ament_python, dependencies)
├── setup.py                             # Python package setup + console_scripts entry points
├── setup.cfg                            # ament/colcon build config
└── .gitignore                           # Ignores build/, install/, log/, __pycache__, .DS_Store, etc.
```

## Directory Descriptions

### `lane_detection/`

The Python package containing all ROS 2 node source files.

- **`lane_detector_node.py`** — Core node. Subscribes to `/camera/image_raw`, runs
  HSV-based white/yellow detection, 6-priority error computation, PID controller,
  and publishes `/cmd_vel` + `/lane_detection/debug_img`.
- **`diagnose_camera.py`** — Standalone diagnostic. Saves `/tmp/diag_*.png` and prints
  HSV stats. Run while sim is live to verify mask coverage.
- **`respawn_robot.py`** — Respawn / recovery node. Monitors robot state and teleports
  back to a random road pose when the robot leaves the track.

### `launch/`

ROS 2 launch files. `simulation.launch.py` orchestrates the full simulation:
Xvfb virtual display → gzserver (on `:1`) → gzclient (on `:0`) →
robot_state_publisher → spawn robot → lane_detector_node.

### `worlds/`

Gazebo world files. `two_lane_circle.world` defines the capsule-shaped two-lane track
with white boundary lines and yellow dashed centre line.

### `resource/`

ament index marker — required by the ROS 2 package discovery system. Do not edit.

### `.claude/rules/`

Claude Code rules loaded automatically. Cover code style, control logic, development
constraints, and tuning procedures.

### `.claude/specs/`

Claude Code specs loaded automatically. Cover file architecture, hardware, ROS
interface, tech stack, and track geometry.

## Entry Points (from `setup.py`)

| Command | Module | Description |
|---------|--------|-------------|
| `ros2 run lane_detection lane_detector_node` | `lane_detection.lane_detector_node:main` | Main lane-following node |
| `ros2 run lane_detection diagnose_camera`    | `lane_detection.diagnose_camera:main`    | HSV diagnostic tool |
| `ros2 run lane_detection respawn_robot`      | `lane_detection.respawn_robot:main`      | Robot respawn node |
```
