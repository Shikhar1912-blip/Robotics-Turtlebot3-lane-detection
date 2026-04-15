# ROS Interface

## Topics

### Subscribed

| Topic | Message type | Hz | Producer | Consumer |
|-------|-----------|----|----------|----------|
| `/camera/image_raw` | `sensor_msgs/Image` | ~1.7 | Gazebo camera plugin | `lane_detector_node` |

### Published

| Topic | Message type | Hz | Producer | Consumer |
|-------|-----------|----|----------|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | ~1.7 | `lane_detector_node` | Gazebo diff-drive plugin |
| `/lane_detection/debug_img` | `sensor_msgs/Image` | ~1.7 | `lane_detector_node` | `rqt_image_view` (manual) |
| `/robot_description` | `std_msgs/String` | latched | `robot_state_publisher` | TF consumers |

### `/cmd_vel` field usage

| Field | Used? | Range | Meaning |
|-------|-------|-------|---------|
| `linear.x` | Yes | 0.04–0.10 m/s | Forward speed (scaled by turn demand) |
| `angular.z` | Yes | −0.6–+0.6 rad/s | Steering: positive = turn left, negative = turn right |
| All other fields | No | 0.0 | Unused (differential drive ignores them) |

---

## Services

### Used by `respawn_robot.py`

| Service | Type | Direction | Purpose |
|---------|------|-----------|---------|
| `/delete_entity` | `gazebo_msgs/DeleteEntity` | client | Remove robot from Gazebo world |
| `/spawn_entity` | `gazebo_msgs/SpawnEntity` | client | Respawn robot at new pose |

Both services are provided by `libgazebo_ros_factory.so` (loaded via `-s` flag in gzserver).

---

## Gazebo Plugins

Loaded by `gzserver` at startup:

| Plugin `.so` | Purpose |
|-------------|---------|
| `libgazebo_ros_init.so` | Initialises ROS 2 ↔ Gazebo bridge |
| `libgazebo_ros_factory.so` | Provides `/spawn_entity` and `/delete_entity` services |

Camera sensor plugin is embedded in `turtlebot3_burger_cam/model.sdf` — publishes to `/camera/image_raw`.

---

## TF Tree

```
world
  └── base_footprint          (robot_state_publisher, from URDF)
        └── base_link
              └── camera_link
```

The lane detector does not use TF — all processing is in image space.

---

## Node Graph

```
[gzserver]
  ├─ /camera/image_raw ──────────────────────▶ [lane_detector_node]
  │                                                  │
  │                                                  ├─▶ /cmd_vel ──▶ [gzserver diff-drive]
  │                                                  └─▶ /lane_detection/debug_img
  │
  └─ /spawn_entity  ◀────── [respawn_robot]
  └─ /delete_entity ◀────── [respawn_robot]

[robot_state_publisher]
  └─▶ /tf (base_footprint → base_link → camera_link)
```

---

## Useful CLI commands

```bash
# Check topics
ros2 topic list
ros2 topic hz /camera/image_raw        # should be ~1.7 Hz
ros2 topic echo /cmd_vel               # shows live steering output

# Inspect image
ros2 run rqt_image_view rqt_image_view
# → Subscribe to /lane_detection/debug_img

# Check node
ros2 node list
ros2 node info /lane_detector

# Respawn robot
ros2 run lane_detection respawn_robot

# Run HSV diagnostic (simulation must be running)
ros2 run lane_detection diagnose_camera
```
