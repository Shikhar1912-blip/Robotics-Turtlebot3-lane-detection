# lane_detection/ — CLAUDE.md

This directory contains all executable Python nodes for the lane-detection package.

## Files

| File | Entry point | Purpose |
|------|-------------|---------|
| `lane_detector_node.py` | `lane_detector_node` | Main perception + control loop |
| `diagnose_camera.py` | `diagnose_camera` | One-shot HSV diagnostic, saves PNG snapshots |
| `respawn_robot.py` | `respawn_robot` | Delete + respawn robot at new random pose |
| `__init__.py` | — | Empty package marker |

---

## lane_detector_node.py

### Data flow

```
/camera/image_raw  (~1.7 Hz, software-rendered via Xvfb)
    │
    ▼
image_callback()
    ├─ Crop ROI: rows [ROI_TOP_FRAC=0.35 … ROI_BOT_FRAC=0.70] from top
    ├─ BGR → HSV
    ├─ white_mask   (H:0-180, S:0-80,   V:160-255)
    ├─ yellow_mask  (H:10-40, S:80-255, V:80-255)
    ├─ MORPH_OPEN  (3×3 ellipse) on each mask — removes speckle, NO CLOSE
    ├─ _edge_centroid(white, 'left')   → left_x
    ├─ _edge_centroid(white, 'right')  → right_x
    ├─ _largest_centroid(yellow)       → yellow_x
    ├─ _compute_error()                → pixel error, valid flag, case number
    ├─ PID controller                  → /cmd_vel (Twist)
    └─ _publish_debug()                → /lane_detection/debug_img
```

### Tuning constants (all module-level, edit source directly — no ROS params)

#### Speed & angular limits

| Constant | Value | Effect |
|----------|-------|--------|
| `LINEAR_MAX` | 0.10 m/s | Forward speed at zero angular demand |
| `LINEAR_MIN` | 0.04 m/s | Forward speed at max angular demand |
| `ANGULAR_MAX` | 0.6 rad/s | Hard clamp on angular output |
| `SEARCH_OMEGA` | 0.25 rad/s | Rotation speed in SEARCHING state |

#### PID gains

| Constant | Value | Notes |
|----------|-------|-------|
| `KP` | 0.008 | Proportional; increase if robot doesn't track curves tightly |
| `KI` | 0.0003 | Integral; eliminates steady-state offset on constant-curvature sections |
| `INTEGRAL_MAX` | 150.0 px·s | Anti-windup clamp; prevents integral runaway |
| `KD` | 0.001 | Derivative; kept low — 1.7 Hz makes raw derivative very noisy |
| `DERIVATIVE_ALPHA` | 0.3 | Low-pass weight on derivative: `0.3*raw + 0.7*prev`. Set to 0 to disable D term |

#### Detection thresholds

| Constant | Value | Effect |
|----------|-------|--------|
| `ROI_TOP_FRAC` | 0.35 | ROI starts 35% from top (cuts sky) |
| `ROI_BOT_FRAC` | 0.70 | ROI ends 70% from top (cuts green island) |
| `MIN_CONTOUR_AREA` | 20 px² | Rejects speckle noise |
| `MAX_CONTOUR_AREA` | 2500 px² | Rejects sky/background blobs (>10 000 px²) |
| `LANE_LOST_TTL` | 2 frames | Consecutive misses before → SEARCHING (~1.2 s at 1.7 Hz) |
| `DEBUG_EVERY_N` | 1 | Publish debug image every N frames (1 = every frame) |

#### Lane-centering & fisheye compensation

| Constant | Value | Effect |
|----------|-------|--------|
| `LANE_CENTER_OFFSET_FRAC` | 0.35 | Fraction of road width from inner edge to target (Case 2) |
| `LEFT_EDGE_TARGET_FRAC` | 0.30 | Target image-fraction for inner-white edge (Case 5) |
| `YELLOW_TARGET_FRAC` | 0.70 | Target image-fraction for yellow centre (Case 4) |
| `FISHEYE_OUTWARD_BIAS_PX` | 10 px | Added to Cases 1–3 to counter 182° fisheye compression near image edges |

### Error computation — `_compute_error()` (6 priority cases)

`error = target_x − img_cx`.  Positive → target right of centre → turn right (−ω).

| Priority | Visible | How target is computed |
|----------|---------|----------------------|
| 1 | Yellow + left white | `(left_x + yellow_x) / 2 + FISHEYE_OUTWARD_BIAS_PX` |
| 2 | Both white edges | `left_x + LANE_CENTER_OFFSET_FRAC * road_width + FISHEYE_OUTWARD_BIAS_PX` |
| 3 | Yellow + right white | Mirror yellow to infer inner white → midpoint + bias |
| 4 | Yellow only | Steer yellow to `YELLOW_TARGET_FRAC` × image width |
| 5 | Left white only | Steer inner edge to `LEFT_EDGE_TARGET_FRAC` × image width |
| 6 | Nothing | `valid = False` → state machine decides |

### State machine

```
┌──────────────┐  valid detection     ┌─────────────┐
│  SEARCHING   │ ────────────────────▶ │  FOLLOWING  │
│ ω=SEARCH_OMEGA│                      │ PID + speed │
│ linear=0     │ ◀──────────────────── │   scaling   │
└──────────────┘  LANE_LOST_TTL misses └─────────────┘
                  (integral reset)

Grace period: first lost frame(s) within TTL → publish error=0 (coast forward, no correction).
```

**Speed scaling (FOLLOWING):**
```
turn_ratio = |angular_z| / ANGULAR_MAX        # 0 … 1
linear_x   = LINEAR_MAX − (LINEAR_MAX − LINEAR_MIN) × turn_ratio
```

### Contour filtering

`_edge_centroid` — splits white mask into left/right blobs:
- Picks the leftmost bounding-rect origin for `side='left'`
- Picks the rightmost bounding-rect right-edge for `side='right'`
- Returns x-centroid of that blob (from `cv2.moments`)

`_largest_centroid` — picks biggest qualifying blob by area.

Both filter: `MIN_CONTOUR_AREA < area < MAX_CONTOUR_AREA`.

### Debug image (`/lane_detection/debug_img`)

Overlay colours:
- **Blue tint** over ROI — white mask hits
- **Cyan tint** over ROI — yellow mask hits
- **Blue vertical line** — `left_x`
- **Red vertical line** — `right_x`
- **Cyan vertical line** — `yellow_x`
- **Gray vertical line** — image centre
- **Green horizontal lines** — ROI top/bottom boundaries
- **Text label** — current state (FOLLOWING / SEARCHING)

View with: `ros2 run rqt_image_view rqt_image_view`

---

## diagnose_camera.py

Run while simulation is live to inspect raw HSV values hitting the thresholds.

```bash
ros2 run lane_detection diagnose_camera
```

Captures **one** frame from `/camera/image_raw`, then:

| Output | Path |
|--------|------|
| Raw BGR frame | `/tmp/diag_raw.png` |
| ROI BGR crop | `/tmp/diag_roi_bgr.png` |
| ROI HSV image | `/tmp/diag_roi_hsv.png` |
| White mask | `/tmp/diag_white_mask.png` |
| Yellow mask | `/tmp/diag_yellow_mask.png` |
| Colour overlay | `/tmp/diag_overlay.png` |

Also prints:
- Per-channel HSV stats (min/max/mean/non-zero%) for the ROI
- Pixel counts for white and yellow masks
- Broad-threshold fallback test if both masks return zero hits
- Brightpixel analysis if image appears blank

`ROI_TOP_FRAC` in this file is set to **0.50** (more conservative than the node's 0.35) to focus on the road surface, not sky.

---

## respawn_robot.py

Respawns the robot without restarting the simulation.

```bash
ros2 run lane_detection respawn_robot
```

**Sequence:**
1. Calls `/delete_entity` to remove `turtlebot3_burger_cam` from Gazebo
2. Picks a random pose on the track (see constants below)
3. Calls `/spawn_entity` to place the robot at the new pose

**Constants that must match the world file:**

| Constant | Current value | Notes |
|----------|---------------|-------|
| `SPAWN_RADIUS_MIN` | 2.95 m | **STALE — circular track geometry** |
| `SPAWN_RADIUS_MAX` | 3.95 m | **STALE — circular track geometry** |
| `SPAWN_Z` | 0.01 m | Slight lift to avoid ground penetration |

> **Action needed:** `respawn_robot.py` still uses the old circular-track radius logic.
> Update `_random_road_pose()` to match the capsule track:
> spawn on top straight at `y = 1.35`, `x ∈ [−1.5, 1.5]`, `yaw = 0.0`.
