# Hardware Specifications

> All values below describe the **simulated** TurtleBot3 Burger in Gazebo.
> Physical hardware is not used — this is a pure simulation project.

---

## Robot Platform — TurtleBot3 Burger

| Property | Value |
|----------|-------|
| Model | TurtleBot3 Burger (`turtlebot3_burger_cam` variant) |
| Drive type | Differential drive (two powered wheels + caster) |
| Body width | ~0.178 m (17.8 cm) — used for lane-width ratio checks |
| Max linear velocity | 0.22 m/s (hardware limit; we run at 0.10 m/s) |
| Max angular velocity | 2.84 rad/s (hardware limit; clamped to 0.6 rad/s in software) |
| Wheel separation | ~0.160 m |
| Wheel radius | ~0.033 m |
| SDF source | `turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf` |
| URDF source | `turtlebot3_gazebo/urdf/turtlebot3_burger_cam.urdf` |

---

## Camera

| Property | Value |
|----------|-------|
| Model | OV2710 2MP USB camera |
| Sensor | OmniVision OV2710, 1/2.7" CMOS |
| Resolution | 2MP (1920×1080 or configurable) |
| Frame rate | 15 Hz (configured capture frequency) |
| Mount height | ~9.3 cm above ground |
| Orientation | Forward-facing, slight downward tilt |
| Interface | USB (V4L2) |
| Topic | `/camera/image_raw` (`sensor_msgs/Image`, encoding `bgr8`) |
| Debug topic | `/lane_detection/debug_img` (`sensor_msgs/Image`) |

### Frame rate and controller timing

The OV2710 is configured to capture at **15 Hz** (`dt ≈ 0.067 s` per frame).

**Controller design consequence:** All timing constants (derivative filter,
integral accumulator, `dt` estimate) assume a ~0.067 s frame interval.
This is significantly faster than the previous software-rendered simulation rate
(~1.7 Hz), so PID gains and integral limits may need retuning if ported back
to a purely simulated environment.

---

## Simulated Track (`worlds/two_lane_circle.world`)

| Property | Value |
|----------|-------|
| Shape | Capsule (stadium) — two straights + two semicircular ends |
| Straight length | 4.0 m (semicircle centres at x = ±2.0 m) |
| Semicircle inner radius | 1.0 m |
| Track width | 0.70 m (inner to outer boundary) |
| Outer white boundary | r = 1.70 m, 0.06 m wide |
| Lane centre | r = 1.35 m from each semicircle centre; y = ±1.35 m on straights |
| Inner white boundary | r = 1.0–1.06 m, 0.06 m wide |
| Green inner island | r = 0–1.0 m |
| Yellow centre dashes | r = 1.35 m; 0.4 m long × 0.06 m wide; period 0.65 m |
| Total track length | ~16.5 m (2 × 4.0 m straights + 2 × π × 1.35 m semicircles) |

### Design-constraint verification

| Constraint | Spec | Actual | Pass? |
|-----------|------|--------|-------|
| Lane width / robot width | ≥ 4:1 | 0.70 / 0.178 ≈ 3.93 | ≈ pass |
| Curve radius / robot width | ≥ 5:1 | 1.00 / 0.178 ≈ 5.6 | ✓ |
| Dash length > gap | dash > gap | 0.40 > 0.25 | ✓ |

---

## Compute (Host Machine)

| Component | Detail |
|-----------|--------|
| Environment | Windows 11 + WSL2 |
| Display | WSLg (XWayland) on `:0` for GUI; Xvfb on `:1` for camera rendering |
| GPU | Not used for simulation rendering (Mesa CPU fallback) |
| ROS install | `/opt/ros/humble/` (Debian packages, apt) |
| Workspace | `~/turtlebot3_lane_ws/` |
