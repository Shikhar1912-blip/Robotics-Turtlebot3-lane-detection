# Development Rules

Rules and conventions for making changes to this repository.
Follow these to avoid breaking the simulation or introducing subtle bugs.

---

## Build rules

- Always use `--symlink-install` with colcon:
  ```bash
  colcon build --packages-select lane_detection --symlink-install
  ```
  Without this flag, Python source edits require a rebuild before they take effect.

- Source both setup files in every new terminal:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/turtlebot3_lane_ws/install/setup.bash
  ```

- `export TURTLEBOT3_MODEL=burger` must be set before launching.

---

## Editing constants

- All tuneable values live as **module-level constants** in `lane_detection/lane_detector_node.py`.
- Do **not** move them into class attributes or ROS parameters unless explicitly asked.
- After editing constants, the `--symlink-install` link means the change is live immediately —
  no rebuild needed. Just restart the node or re-launch.

---

## Do not change these without careful testing

| Item | Risk if changed carelessly |
|------|---------------------------|
| `cv2.MORPH_OPEN` → `MORPH_CLOSE` | Sky/background blobs merge with road markings — detection breaks |
| `MAX_CONTOUR_AREA` raised above ~5000 | Sky leak-through detected as lane edge |
| Removing `FISHEYE_OUTWARD_BIAS_PX` | Robot hugs inner island edge on every curve |
| `INTEGRAL_MAX` removed | Integral windup after SEARCHING → violent overshoot |
| `gzserver DISPLAY=:0` instead of `:1` | Camera sensors fail silently — no `/camera/image_raw` |
| Adding `cv2.MORPH_CLOSE` to yellow mask | Yellow mask grows to engulf road surface |

---

## World file rules

- The file `worlds/two_lane_circle.world` is the **single source of truth** for track geometry.
- All constants that reference track dimensions (spawn positions, radius values) must be updated
  whenever the world file changes.
- Files to keep in sync when the track changes:
  1. `worlds/two_lane_circle.world`
  2. `launch/simulation.launch.py` — spawn x/y/yaw
  3. `lane_detection/respawn_robot.py` — `SPAWN_RADIUS_MIN/MAX` or `_random_road_pose()`
  4. `CLAUDE.md` — Road Geometry section
  5. `.claude/specs/track-geometry.md`
  6. `.claude/specs/hardware.md` — simulated track table

---

## Gazebo environment rules

- `gzserver` **must** run on `DISPLAY=:1` (Xvfb) — not `:0` (WSLg).
  Xvfb provides the software GLX that camera sensors require.
- `gzclient` **must** run on `DISPLAY=:0` (WSLg) for the window to appear on screen.
- Always set `LIBGL_ALWAYS_SOFTWARE=1` and `OGRE_RTT_MODE=Copy` for both processes.
- Never change `GAZEBO_IP` away from `127.0.0.1` — it prevents Gazebo from binding to
  WSL2 network interfaces that can disconnect.

---

## Git rules

- Commit only source files — never commit `build/`, `install/`, `__pycache__/`, or `/tmp/*`.
- The `.world` file counts as source — commit it when the track changes.
- Commit message convention: `<type>: <short description>`
  - Types: `fix`, `feat`, `tune`, `refactor`, `docs`, `world`
  - Example: `tune: increase KP to 0.008 for better curve tracking`

---

## Testing checklist before committing

```
□ ros2 topic hz /camera/image_raw  → ~15 Hz  (OV2710 USB camera is working)
□ ros2 topic echo /cmd_vel         → non-zero angular.z on curves
□ rqt_image_view /lane_detection/debug_img  → coloured overlays visible over lane markings
□ Robot completes at least one full lap without entering SEARCHING
□ No ERROR or WARN messages in lane_detector terminal output
```

---

## Forbidden patterns

```python
# ✗ Do not use print() in ROS nodes
print("debug")

# ✓ Use the ROS logger
self.get_logger().info("debug")

# ✗ Do not sleep inside callbacks
time.sleep(1.0)

# ✗ Do not add MORPH_CLOSE to lane masks
cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# ✗ Do not hardcode image dimensions
width = 640

# ✓ Always derive from the actual frame
h, w = frame.shape[:2]

# ✗ Do not read pkg_path from the install space for worlds/
pkg_path = get_package_share_directory('lane_detection')  # won't find worlds/ correctly

# ✓ Hardcoded source path as documented
pkg_path = os.path.join(os.path.expanduser('~'), 'turtlebot3_lane_ws/src/lane_detection')
```
