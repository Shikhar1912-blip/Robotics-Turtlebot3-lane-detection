# Tech Stack

## Operating System & Virtualisation

| Layer | Technology | Version / Detail |
|-------|-----------|-----------------|
| Host OS | Windows 11 | WSL2 backend |
| Linux distro | Ubuntu 22.04 LTS (Jammy) | Inside WSL2 |
| Display server | WSLg / XWayland | Host display `:0` — for gzclient GUI window |
| Virtual display | Xvfb (X Virtual Framebuffer) | Display `:1` — for gzserver software-GLX camera rendering |
| GL renderer | Mesa LLVMpipe | Software rasteriser (`LIBGL_ALWAYS_SOFTWARE=1`) |

---

## Robotics Middleware

| Component | Package / Version | Role |
|-----------|------------------|------|
| ROS 2 | **Humble Hawksbill** (LTS, May 2022) | Middleware, pub/sub, services, TF |
| Build tool | `colcon` | Workspace build (`colcon build`) |
| Build type | `ament_python` | Pure Python package — no C++ compilation |
| Simulator | **Gazebo Classic 11** (`gazebo11`) | Physics + camera rendering |
| Gazebo-ROS bridge | `gazebo_ros_pkgs` (Humble) | `/spawn_entity`, `/delete_entity`, sensor plugins |
| Robot description | `turtlebot3_gazebo` (Humble) | SDF, URDF, and camera model files |

### Key ROS packages used at runtime

| Package | Purpose |
|---------|---------|
| `robot_state_publisher` | Broadcasts TF from URDF |
| `gazebo_ros` | `spawn_entity.py` executable |
| `rqt_image_view` | Debug image viewer |
| `cv_bridge` | ROS Image ↔ OpenCV ndarray conversion |

---

## Programming Language & Libraries

| Library | Version | Usage |
|---------|---------|-------|
| **Python** | 3.10 (Ubuntu 22.04 default) | All node logic |
| **OpenCV** (`cv2`) | 4.x (shipped with ROS Humble) | HSV masking, morphology, contours, debug overlay |
| **NumPy** | 1.x | Array ops, HSV threshold arrays, `np.clip` |
| `rclpy` | Humble | ROS 2 Python client library |
| `cv_bridge` | Humble | Image message ↔ `cv2` BGR conversion |
| `sensor_msgs` | Humble | `Image` message type |
| `geometry_msgs` | Humble | `Twist`, `Pose`, `Point`, `Quaternion` |
| `gazebo_msgs` | Humble | `DeleteEntity`, `SpawnEntity` service types |
| `ament_index_python` | Humble | `get_package_share_directory()` |

---

## File Formats

| Format | File(s) | Purpose |
|--------|---------|---------|
| SDF 1.6 | `worlds/two_lane_circle.world` | Gazebo world — capsule track geometry |
| SDF | `turtlebot3_burger_cam/model.sdf` | Robot model with fisheye camera |
| URDF | `turtlebot3_burger_cam.urdf` | Robot URDF for `robot_state_publisher` |
| Python | `lane_detection/*.py` | ROS nodes |
| Python | `launch/simulation.launch.py` | Launch file |

---

## Workspace Layout

```
~/turtlebot3_lane_ws/
├── src/
│   └── lane_detection/          ← this repo
│       ├── lane_detection/      ← Python package (nodes)
│       ├── launch/              ← launch files
│       ├── worlds/              ← Gazebo world files
│       ├── setup.py             ← ament_python entry points
│       └── package.xml
├── install/                     ← colcon install space (auto-generated)
└── build/                       ← colcon build space (auto-generated)
```

`--symlink-install` means Python source edits take effect without rebuilding.
