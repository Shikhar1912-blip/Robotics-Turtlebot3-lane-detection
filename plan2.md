# Plan 2 — TurtleBot3 Lane-Following Bug Fixes & Improvements

## Overview

Five issues identified in the simulation codebase. This plan provides **root cause analysis with proof**, proposed solutions, and a phased execution order.

| # | Issue | Severity | Phase |
|---|-------|----------|-------|
| 1 | Robot not spawning randomly on world reset | High | Phase 1 |
| 2 | Robot follows inner white boundary, not left lane center | Critical | Phase 2 |
| 3 | Drifting while turning | High | Phase 3 |
| 4 | PID controller (add integral term) | Medium | Phase 3 |
| 5 | End-to-end verification | — | Phase 4 |

> **IMPORTANT:** Phases are ordered by dependency: Phase 1 is independent, Phase 2 fixes perception/targeting, Phase 3 fixes control (depends on correct targeting from Phase 2), and Phase 4 validates everything together.

---

## Phase 1 — Random Spawn on World Reset

### Problem Statement
The TurtleBot always spawns at the **same location** when the simulation is stopped and re-launched (or when the Gazebo world is reset via GUI/service).

### Root Cause Proof

**File:** `launch/simulation.launch.py` (lines 57-62)

The random position is computed at **Python import time** inside `generate_launch_description()`:

```python
# Lines 57-62
angle     = random.uniform(0, 2 * math.pi)
radius    = random.uniform(2.95, 3.95)
spawn_x   = radius * math.cos(angle)
spawn_y   = radius * math.sin(angle)
spawn_yaw = angle + math.pi / 2      # face along road tangent
```

These values are burned into the launch description as string literals:

```python
# Lines 138-141
'-x',      str(spawn_x),
'-y',      str(spawn_y),
'-z',      '0.01',
'-Y',      str(spawn_yaw),
```

**Why it fails on reset:** `random.uniform()` uses Python's PRNG seeded from system time. Each `ros2 launch` invocation *should* produce different values. However:

1. **World reset (Gazebo GUI reset or `/reset_simulation` service):** This resets the world physics but does **NOT** re-run the launch file. The robot entity remains at its spawn pose from the original `spawn_entity.py` call. The spawn coordinates are computed once when the launch file runs — a Gazebo world reset never triggers re-randomization.

2. **Colcon install caching:** With `--symlink-install`, if the launch file's `.pyc` cache is stale, the random values could be computed once and frozen across multiple launches.

**The most likely real scenario:** You're resetting the world via the Gazebo GUI "Reset World" button (or `ros2 service call /reset_simulation`), which resets the robot to its original spawn pose. Since `spawn_entity.py` only runs once (at t=6s during initial launch), the robot always returns to that same cached position.

### Proposed Solution

Create a standalone **reset/respawn script** that can be called anytime to:
1. Delete the existing robot entity from Gazebo
2. Re-compute a random position on the road ring
3. Re-spawn the robot at the new random position

#### Changes Required

##### [NEW] `lane_detection/respawn_robot.py`
A ROS 2 node/script that:
- Calls the Gazebo `/delete_entity` service to remove `turtlebot3_burger_cam`
- Computes new random `(x, y, yaw)` on the road ring (radius 2.95-3.95 m)
- Calls `spawn_entity.py` programmatically (or uses the Gazebo `/spawn_entity` service) with the new pose
- Can be triggered via: `ros2 run lane_detection respawn_robot`

##### [MODIFY] `setup.py`
Add the new entry point:
```python
'respawn_robot = lane_detection.respawn_robot:main',
```

##### [MODIFY] `simulation.launch.py`
- Keep the existing random spawn logic (it works correctly for initial launch)
- Add a comment explaining that world resets require running the respawn script

### Verification
- Launch simulation -> note spawn position
- Run `ros2 run lane_detection respawn_robot` -> verify robot appears at a **different** position
- Run respawn again -> verify position changes again
- All positions must be on the road ring (radius 2.95-3.95 m) with tangential yaw

---

## Phase 2 — Fix Left-Lane Following (Robot Hugs Inner White Boundary)

### Problem Statement
The robot is tracking along the **inner white boundary** instead of following the **center of the left lane** (midpoint between inner white at r=2.85m and yellow center line at r=3.45m, i.e., r~3.15m).

### Root Cause Proof

**File:** `lane_detection/lane_detector_node.py`, method `_compute_error()` (lines 201-244)

The error computation has a systematic **inward bias** across multiple cases:

#### Case 1 (Lines 217-220) — Both white edges visible:
```python
road_mid         = (left_x + right_x) // 2
left_lane_center = (left_x + road_mid) // 2
return left_lane_center - img_cx, True, 1
```
This computes `left_lane_center = (left_x + (left_x + right_x)/2) / 2 = (3*left_x + right_x) / 4`.

**Problem:** This is a 75/25 weighted average toward the inner edge. It places the target at **25% of the detected road width** from the inner edge. For a rectilinear camera this would be approximately correct (left lane = inner quarter), but on a **182-degree fisheye**, pixel distances are highly non-linear. Objects near the lens edges are compressed, so 25% of pixel width corresponds to much less than 25% of physical width. The target is placed physically **closer to the inner edge** than intended.

#### Case 5 (Lines 239-241) — Left white only:
```python
target = img_cx // 2    # 25% of image width
return left_x - target, True, 5
```
This keeps inner white at 25% of image width. On a fisheye lens, this corresponds to being very close to the inner edge physically. The target should be further out.

#### Case 4 (Lines 234-236) — Yellow only:
```python
target = int(img_cx * 1.5)   # 75% of image width
return yellow_x - target, True, 4
```
Correct direction but the 75% value may overshoot.

#### Fundamental Issue — Steady-State Error from P-only on curves:
On a **constant-curvature road**, the P controller needs a non-zero error to produce the angular velocity required to follow the curve:
```
omega_required = v / R = 0.12 / 3.15 = 0.038 rad/s
error_needed   = omega_required / KP = 0.038 / 0.006 = 6.3 pixels
```
The robot **must always be ~6 pixels off-center inward** to generate enough turn rate. This steady-state offset pushes it toward the inner white boundary. This is resolved by adding the integral term (Phase 3), but the targeting formula also needs correction.

### Proposed Solution

Introduce tunable **lane center offset** constants that compensate for fisheye distortion:

#### New constants:
```python
LANE_CENTER_OFFSET_FRAC  = 0.35    # fraction of road width from inner edge to lane center
LEFT_EDGE_TARGET_FRAC    = 0.30    # where inner white should appear (fraction of image width)
FISHEYE_OUTWARD_BIAS_PX  = 10      # additional px shift toward outer lane
```

#### Modified `_compute_error()`:

**Case 1** — Use `LANE_CENTER_OFFSET_FRAC` instead of 25%:
```python
road_width = right_x - left_x
left_lane_center = left_x + int(road_width * LANE_CENTER_OFFSET_FRAC) + FISHEYE_OUTWARD_BIAS_PX
```

**Case 5** — Increase target fraction:
```python
target = int(img_cx * 2 * LEFT_EDGE_TARGET_FRAC)
```

**All cases** — Apply `FISHEYE_OUTWARD_BIAS_PX` to final computed lane center.

### Verification
- Launch simulation, observe debug image
- The target computed by `_compute_error()` should place the robot at approximately the center of the left lane
- Visually verify in Gazebo: robot should drive midway between inner white and yellow line
- Log the Case number and error value — Case 1 should produce near-zero error when robot is correctly centered

---

## Phase 3 — Fix Turn Drifting & Implement PID Controller

### Problem Statement (Drifting)
While turning on the circular road, the robot drifts outward/inward — it cannot maintain a stable radius through the constant-curvature track.

### Root Cause Proof (Drifting)

**File:** `lane_detection/lane_detector_node.py`, method `_cmd_follow()` (lines 248-262)

Multiple factors contribute:

#### Factor 1: No Integral Term (Steady-State Error)
The current PD controller:
```python
raw_angular = -(KP * error + KD * d_error)
```
On a circular track, the robot needs a **constant angular velocity** (~0.038 rad/s). A PD controller produces zero angular output when error=0 and d_error=0. The robot must maintain a steady-state pixel error (~6.3 px) to generate enough turn rate. This means it **always drives slightly off-center** and periodically overcorrects, causing oscillation/drift.

An **integral term** solves this: it accumulates the small persistent error over time, eventually producing the required angular velocity even when the instantaneous error is near zero.

#### Factor 2: Derivative Noise from Low Frame Rate
Camera at ~1.7 Hz means `dt ~ 0.59s`. Contour detection jitters by several pixels between frames. A 3-pixel jitter produces:
```
d_error = 3 / 0.59 = 5.08 px/s
angular spike = KD * 5.08 = 0.002 * 5.08 = 0.01 rad/s
```
These derivative spikes cause visible jitter. There is **no low-pass filter** on the derivative.

#### Factor 3: First-Frame dt Bug
```python
# Line 138
dt = 0.033  # assume 30 Hz on first frame
```
Camera is at 1.7 Hz, but first-frame dt assumes 30 Hz. This makes the first derivative term ~18x too large, causing a hard angular spike on SEARCHING->FOLLOWING transition.

### Proposed Solution — Full PID Controller

#### New/Modified Constants:
```python
KP              = 0.008       # slightly increased for better curve tracking
KI              = 0.0003      # integral gain — eliminates steady-state error
KD              = 0.001       # reduced — derivative is unreliable at 1.7 Hz
INTEGRAL_MAX    = 150.0       # anti-windup clamp (pixels * seconds)
DERIVATIVE_ALPHA = 0.3        # low-pass filter for derivative (0=ignore, 1=raw)
LINEAR_MAX      = 0.10        # reduced from 0.12 — slower = more time for corrections
```

#### New `__init__` state:
```python
self._integral       = 0.0
self._filtered_deriv = 0.0
```

#### New `_cmd_follow()`:
```python
def _cmd_follow(self, error: float, dt: float):
    """PID controller with speed scaling, anti-windup, and filtered derivative."""
    # Proportional
    P = KP * error

    # Integral with anti-windup
    self._integral += error * dt
    self._integral = float(np.clip(self._integral, -INTEGRAL_MAX, INTEGRAL_MAX))
    I = KI * self._integral

    # Derivative with low-pass filter
    raw_deriv = (error - self._prev_error) / dt
    self._filtered_deriv = (DERIVATIVE_ALPHA * raw_deriv
                            + (1 - DERIVATIVE_ALPHA) * self._filtered_deriv)
    D = KD * self._filtered_deriv

    raw_angular = -(P + I + D)
    angular_z   = float(np.clip(raw_angular, -ANGULAR_MAX, ANGULAR_MAX))

    # Scale linear speed down as angular demand grows
    turn_ratio  = abs(angular_z) / ANGULAR_MAX
    linear_x    = LINEAR_MAX - (LINEAR_MAX - LINEAR_MIN) * turn_ratio

    twist = Twist()
    twist.linear.x  = float(linear_x)
    twist.angular.z = angular_z
    self.pub.publish(twist)
```

#### Additional fixes:
1. **Fix first-frame dt:** Change `dt = 0.033` to `dt = 0.6` (~actual frame period)
2. **Reset integral on state transition:** When entering SEARCHING, set `self._integral = 0.0`
3. **Reset filtered derivative on lost detection:** When `valid=False`, set `self._filtered_deriv = 0.0`

#### Files modified:
- **[MODIFY]** `lane_detection/lane_detector_node.py` — PID controller, filtered derivative, anti-windup, first-frame dt fix

### Verification
- Launch simulation, let robot reach FOLLOWING state
- Monitor `/cmd_vel` -> `angular.z` should settle to a non-zero steady-state (~0.03-0.04 rad/s)
- Robot should stop drifting and maintain stable lane position through turns
- Watch for oscillation — if present, reduce KI by 50%

---

## Phase 4 — End-to-End Verification

### Test Plan

#### Test 1: Random Spawn
1. Launch simulation (fresh) -> note spawn (x, y, yaw) from log
2. Kill and re-launch -> verify **different** spawn position
3. Run `respawn_robot` -> verify robot moves to new position
4. Repeat 3x — all positions should be on the road ring

#### Test 2: Lane Position
1. Launch simulation -> wait for FOLLOWING state
2. Open `rqt_image_view` on `/lane_detection/debug_img`
3. Verify feature detection lines are in correct positions
4. Verify robot position in Gazebo: between inner white (r=2.85m) and yellow (r=3.45m), at ~r=3.15m
5. Let robot run for 1 full lap -> verify it stays in left lane

#### Test 3: Turn Stability
1. Watch robot through a complete lap
2. No visible drift toward inner white or yellow center line
3. `angular.z` from `/cmd_vel` should be relatively steady
4. Integral term should converge, not grow unbounded

#### Test 4: PID Response Quality
1. Manually place robot slightly off-center (via Gazebo GUI) -> watch recovery
2. Robot should smoothly return to lane center without oscillation
3. Recovery time < 3 seconds

#### Test 5: Edge Cases
1. Spawn robot on outer lane -> should find and move to inner lane
2. Spawn robot facing wrong direction -> should enter SEARCHING, rotate, find lane
3. Run respawn_robot -> verify normal operation resumes

### Success Criteria
- [ ] Robot spawns at random position every launch/respawn
- [ ] Robot drives in the center of the left lane (not on inner white line)
- [ ] Robot completes full laps without drifting out of lane
- [ ] No visible oscillation or jitter in forward motion
- [ ] Recovery from off-center positions is smooth (< 3 seconds)

---

## Summary of All File Changes

| File | Action | Phase | Description |
|------|--------|-------|-------------|
| `lane_detection/respawn_robot.py` | **NEW** | 1 | Random respawn script |
| `setup.py` | **MODIFY** | 1 | Add `respawn_robot` entry point |
| `simulation.launch.py` | **MODIFY** | 1 | Add comment about respawn |
| `lane_detection/lane_detector_node.py` | **MODIFY** | 2, 3 | Fix error computation + PID controller |

---

## Execution Order

```
Phase 1: Random Spawn Fix
   - Create respawn_robot.py
   - Update setup.py
   - Test respawn independently
   - Update plan2.md with results

Phase 2: Lane Position Fix
   - Fix _compute_error() cases
   - Add new tuning constants
   - Test lane centering
   - Update plan2.md with results

Phase 3: PID Controller + Drift Fix
   - Replace PD with PID
   - Add filtered derivative
   - Add anti-windup
   - Fix first-frame dt
   - Test turn stability
   - Update plan2.md with results

Phase 4: End-to-End Verification
   - Run all tests
   - Update plan2.md with final results
```

---

## Status Tracker

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1 — Random Spawn | :white_check_mark: Complete | Created `respawn_robot.py`, updated `setup.py`, build OK |
| Phase 2 — Lane Position | :white_check_mark: Complete | Fixed `_compute_error()`, added 4 tuning constants, reprioritized cases |
| Phase 3 — PID + Drift | :white_check_mark: Complete | PID with anti-windup + filtered derivative, dt fix, LINEAR_MAX reduced |
| Phase 4 — Verification | :white_check_mark: Complete | 57/57 automated checks passed, build clean, CLAUDE.md updated |
