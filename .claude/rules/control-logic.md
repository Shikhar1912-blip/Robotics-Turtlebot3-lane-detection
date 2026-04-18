# Control Logic

Documents the perception → error → control pipeline in `lane_detector_node.py`.
Read this before modifying any detection, error-computation, or control code.

---

## Pipeline overview

```
Camera frame (BGR)
    │
    ▼
Crop ROI  [ROI_TOP_FRAC … ROI_BOT_FRAC] rows
    │
    ▼
BGR → HSV
    │
    ├──▶ white_mask  → MORPH_OPEN → _edge_centroid(left)  → left_x
    │                              → _edge_centroid(right) → right_x
    │
    └──▶ yellow_mask → MORPH_OPEN → _largest_centroid()   → yellow_x
                                               │
    ┌──────────────────────────────────────────┘
    ▼
_compute_error(left_x, right_x, yellow_x, img_cx)
    → (error_px, is_valid, case_number)
                    │
    ┌───────────────┘
    ▼
State machine → PID controller or search rotation
    → /cmd_vel (Twist)
```

---

## ROI rationale

The camera is a 182° fisheye at 9.3 cm height. The image rows contain:

| Rows (approx.) | Content |
|----------------|---------|
| 0–35% | Sky / upper background (no road markings) |
| 35–70% | **Horizon band — lane markings appear here** ← ROI |
| 70–100% | Green inner island / immediate foreground |

Scanning outside the ROI introduces false positives (sky blobs, island colour bleed).
`ROI_TOP_FRAC = 0.35`, `ROI_BOT_FRAC = 0.70`.

---

## HSV colour thresholds

### White lane markings

```python
WHITE_LO  = [  0,   0, 160]   # H: any hue, S: low saturation, V: bright
WHITE_HI  = [180,  80, 255]
```

Broad hue/saturation range because white has low saturation regardless of hue.
Value floor of 160 rejects dim surfaces.

### Yellow centre dashes

```python
YELLOW_LO = [ 10,  80,  80]   # H: orange-yellow range, moderate sat+val
YELLOW_HI = [ 40, 255, 255]
```

---

## Morphological processing

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
```

- **OPEN only** — erodes then dilates. Removes isolated speckle without growing blobs.
- **No CLOSE** — closing dilates then erodes, which was merging the sky blob with road
  markings into one giant contour that broke the centroid calculation. Never add CLOSE.

---

## Contour filtering

Both `_edge_centroid` and `_largest_centroid` reject contours outside the area band:

```
MIN_CONTOUR_AREA (20 px²)  <  area  <  MAX_CONTOUR_AREA (2500 px²)
```

- Below 20 px²: sensor noise, speckle not cleaned by morphology
- Above 2500 px²: sky/background leak-through (sky blobs are typically > 10 000 px²)

### `_edge_centroid(mask, side)`

Finds the **leftmost** or **rightmost** qualifying blob:
- `'left'`  → blob with smallest `bounding_rect.x` (inner white boundary)
- `'right'` → blob with largest  `bounding_rect.x + width` (outer white boundary)

Returns x-centroid via `cv2.moments`.

### `_largest_centroid(mask)`

Returns x-centroid of the **largest-area** qualifying blob (used for yellow dashes).

---

## Error computation — `_compute_error`

```
error = target_x − img_cx
```

- **Positive error**: target is right of image centre → steer right → `angular_z < 0`
- **Negative error**: target is left of image centre → steer left → `angular_z > 0`

### Priority order (highest to lowest)

| # | Condition | Target formula |
|---|-----------|----------------|
| 1 | yellow AND left white | `(left_x + yellow_x) / 2 + FISHEYE_OUTWARD_BIAS_PX` |
| 2 | both white edges | `left_x + LANE_CENTER_OFFSET_FRAC × (right_x − left_x) + FISHEYE_OUTWARD_BIAS_PX` |
| 3 | yellow AND right white | `inferred_left = 2·yellow_x − right_x`; then `(inferred_left + yellow_x)/2 + bias` |
| 4 | yellow only | `error = yellow_x − int(w × YELLOW_TARGET_FRAC)` |
| 5 | left white only | `error = left_x − int(w × LEFT_EDGE_TARGET_FRAC)` |
| 6 | nothing | `valid = False` |

### Fisheye correction

`FISHEYE_OUTWARD_BIAS_PX = 10` is added to Cases 1–3 only (not 4/5).

The 182° fisheye compresses objects near the image edges — the inner white boundary
appears closer to image centre than it physically is. Adding 10 px pushes the computed
lane centre outward, countering this distortion.

---

## PID controller — `_cmd_follow`

```
angular_z = −(KP·error  +  KI·integral  +  KD·filtered_derivative)
angular_z = clip(angular_z, −ANGULAR_MAX, +ANGULAR_MAX)
```

### Proportional

```python
P = KP * error      # KP = 0.008
```

### Integral (anti-windup)

```python
self._integral += error * dt
self._integral  = clip(self._integral, -INTEGRAL_MAX, +INTEGRAL_MAX)
I = KI * self._integral     # KI = 0.0003, INTEGRAL_MAX = 150.0
```

Integral is **reset to 0** when entering SEARCHING state (prevents windup carry-over).

### Derivative (low-pass filtered)

```python
raw_deriv            = (error − prev_error) / dt
self._filtered_deriv = DERIVATIVE_ALPHA * raw_deriv + (1 − DERIVATIVE_ALPHA) * self._filtered_deriv
D = KD * self._filtered_deriv   # KD = 0.001, DERIVATIVE_ALPHA = 0.3
```

At 15 Hz, a 3-pixel jitter produces `d_error ≈ 45 px/s`. The filter coefficient `0.3`
means 70% of the previous smoothed value is retained each frame, damping noise.

### Speed scaling

```python
turn_ratio = |angular_z| / ANGULAR_MAX        # 0 … 1
linear_x   = LINEAR_MAX − (LINEAR_MAX − LINEAR_MIN) × turn_ratio
```

Robot slows to `LINEAR_MIN` (0.04 m/s) at maximum angular demand, and runs at
`LINEAR_MAX` (0.10 m/s) when driving straight.

---

## State machine

```
          ┌────────────────────────────────────────────────────┐
          │                   FOLLOWING                        │
          │  • Run PID controller                              │
          │  • Publish Twist with linear.x and angular.z      │
          │  • lost_counter = 0 on every valid frame          │
          └───────────────────┬────────────────────────────────┘
                              │ invalid frame
                              ▼
                     lost_counter += 1
                              │
               ┌──────────────┴───────────────┐
               │ counter < LANE_LOST_TTL (2)  │ counter ≥ LANE_LOST_TTL
               │                              │
               ▼                              ▼
          Grace period                   SEARCHING
          (coast: error=0,           • angular_z = +SEARCH_OMEGA
           no correction)            • linear.x = 0
                                     • integral reset
                                              │ valid frame detected
                                              ▼
                                          FOLLOWING
```

**LANE_LOST_TTL = 2 frames ≈ 0.13 s** at 15 Hz.

---

## `dt` handling

```python
if self._last_time is None:
    dt = 0.067        # first frame: assume ~15 Hz (OV2710 USB camera)
else:
    dt = max(now - self._last_time, 0.005)   # clamp avoids div-by-zero
self._last_time = now
```

`dt` is used for both integral accumulation and derivative calculation.
The 0.005 s floor prevents division-by-zero if callbacks arrive faster than expected.

---

## Common failure modes and fixes

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| Robot oscillates side-to-side | KP too high | Reduce `KP` (e.g. 0.006) |
| Robot drifts to inner edge on curves | Integral windup or fisheye bias too small | Increase `FISHEYE_OUTWARD_BIAS_PX` or `LANE_CENTER_OFFSET_FRAC` |
| Robot immediately enters SEARCHING | ROI misses all markings | Adjust `ROI_TOP_FRAC`/`ROI_BOT_FRAC`; run `diagnose_camera` |
| Yellow detected but robot still oscillates | Derivative noise | Lower `KD` or reduce `DERIVATIVE_ALPHA` |
| Robot drives too slowly | `LINEAR_MAX` too conservative | Increase `LINEAR_MAX` (max 0.22 m/s hardware limit) |
| Contour area filter rejects real markings | `MAX_CONTOUR_AREA` too small | Increase to 4000 or check actual blob sizes via `diagnose_camera` |
