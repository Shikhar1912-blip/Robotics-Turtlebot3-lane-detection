# Tuning Guide

Step-by-step reference for adjusting the lane-detection system.
All constants are in `lane_detection/lane_detector_node.py` at the module level.

---

## Before tuning: verify the camera works

```bash
# Terminal 1: start simulation
export TURTLEBOT3_MODEL=burger && ros2 launch lane_detection simulation.launch.py

# Terminal 2: verify camera frames are arriving
ros2 topic hz /camera/image_raw        # expect ~15 Hz (OV2710 USB camera)

# Terminal 3: one-shot HSV diagnostic
ros2 run lane_detection diagnose_camera
# Opens /tmp/diag_overlay.png — check that white/yellow masks are hitting real markings
```

If the masks show zero hits, fix HSV thresholds first before touching PID.

---

## HSV threshold tuning

Run `diagnose_camera` and read the printed HSV stats, then adjust `WHITE_LO/HI` and `YELLOW_LO/HI`.

```python
WHITE_LO  = np.array([  0,   0, 160], dtype=np.uint8)
WHITE_HI  = np.array([180,  80, 255], dtype=np.uint8)
YELLOW_LO = np.array([ 10,  80,  80], dtype=np.uint8)
YELLOW_HI = np.array([ 40, 255, 255], dtype=np.uint8)
```

| Symptom | Adjustment |
|---------|-----------|
| White mask misses the lines | Lower `WHITE_LO[2]` (Value floor), e.g. 140 |
| White mask hits road surface | Raise `WHITE_LO[2]`, e.g. 180, or lower `WHITE_HI[1]` (Sat cap) |
| Yellow mask misses dashes | Widen hue range: lower `YELLOW_LO[0]` or raise `YELLOW_HI[0]` |
| Yellow mask hits white lines | Raise `YELLOW_LO[1]` (Sat floor, e.g. 100) |

---

## ROI tuning

If the correct lane markings are outside the scanned region:

```python
ROI_TOP_FRAC = 0.35   # scan starts here (fraction from top of image)
ROI_BOT_FRAC = 0.70   # scan ends here
```

| Symptom | Adjustment |
|---------|-----------|
| Robot misses far-ahead markings | Raise `ROI_BOT_FRAC` slightly (e.g. 0.75) |
| Sky noise leaks into white mask | Raise `ROI_TOP_FRAC` (e.g. 0.40) |
| Green island bleed into ROI | Lower `ROI_BOT_FRAC` (e.g. 0.65) |

Use `diagnose_camera` — it saves `diag_roi_bgr.png` so you can see exactly what the ROI captures.

---

## Contour filter tuning

```python
MIN_CONTOUR_AREA = 20     # px² — rejects speckle
MAX_CONTOUR_AREA = 2500   # px² — rejects sky/background
```

| Symptom | Adjustment |
|---------|-----------|
| Real markings not detected (tiny blobs) | Lower `MIN_CONTOUR_AREA` (e.g. 10) |
| Sky blob detected as white edge | Lower `MAX_CONTOUR_AREA` (e.g. 1500) |
| Thick contiguous marking rejected | Raise `MAX_CONTOUR_AREA` (e.g. 4000) |

To see actual blob sizes, print `cv2.contourArea(c)` temporarily in `_edge_centroid`.

---

## PID gain tuning

Always tune in order: **P → I → D**. Change one gain at a time.

### Proportional (KP = 0.008)

Controls how aggressively the robot steers in response to lateral error.

| Symptom | Action |
|---------|--------|
| Robot oscillates / weaves side-to-side | Reduce KP (e.g. 0.006) |
| Robot understeer / takes wide curves | Increase KP (e.g. 0.010) |
| Robot is stable on straights but overshoots curves | Reduce KP slightly |

Start from 0.004 and double until oscillation appears; then halve back.

### Integral (KI = 0.0003, INTEGRAL_MAX = 150)

Eliminates steady-state offset — the persistent bias where the robot hugs one edge
even when the PD controller should have centred it.

| Symptom | Action |
|---------|--------|
| Robot consistently off-centre on curves | Increase KI (e.g. 0.0005) |
| Robot overshoots after a straight | Reduce KI or lower `INTEGRAL_MAX` |
| Robot oscillates with a slow drift | Integral windup — reduce `INTEGRAL_MAX` |

Set `KI = 0` first to tune P+D, then add integral last.

### Derivative (KD = 0.001, DERIVATIVE_ALPHA = 0.3)

Damps oscillations. At 15 Hz the derivative is less noisy than at slow frame rates, but sensor jitter still warrants a low-pass filter — keep KD conservative.

| Symptom | Action |
|---------|--------|
| Oscillations persist after reducing KP | Increase KD slightly (e.g. 0.002) |
| Robot jerks erratically each frame | KD too high or filter too transparent — reduce `DERIVATIVE_ALPHA` (e.g. 0.1) |
| Derivative has no visible effect | Increase `DERIVATIVE_ALPHA` (e.g. 0.5) to reduce over-filtering |

### Anti-windup (INTEGRAL_MAX = 150 px·s)

Cap on the raw integral accumulator. Limits how much the integrator can "charge up"
during long periods of off-centre driving (e.g. the SEARCHING state).

Reduce if the robot overshoots badly after exiting a curve.

---

## Speed tuning

```python
LINEAR_MAX  = 0.10   # m/s (at zero angular demand)
LINEAR_MIN  = 0.04   # m/s (at max angular demand)
ANGULAR_MAX = 0.6    # rad/s (hard clamp)
```

| Symptom | Action |
|---------|--------|
| Robot misses curves (too fast) | Reduce `LINEAR_MAX` |
| Robot moves too slowly overall | Increase `LINEAR_MAX` (hardware limit: 0.22 m/s) |
| Robot stalls in sharp curves | Increase `LINEAR_MIN` |
| Angular output saturates on straight | Lower `ANGULAR_MAX` |

Speed is automatically scaled: `linear = LINEAR_MAX − (LINEAR_MAX − LINEAR_MIN) × |ω|/ANGULAR_MAX`.
Increasing `ANGULAR_MAX` gives the PID more headroom before saturation.

---

## Fisheye compensation tuning

The 182° fisheye compresses objects near the image edges. The inner white boundary
appears closer to image centre than its true position. Three constants compensate:

```python
FISHEYE_OUTWARD_BIAS_PX = 10   # added to Cases 1–3 target x
LANE_CENTER_OFFSET_FRAC = 0.35  # Case 2: fraction of road width from inner edge
LEFT_EDGE_TARGET_FRAC   = 0.30  # Case 5: where inner white should sit in image
YELLOW_TARGET_FRAC      = 0.70  # Case 4: where yellow dash should sit in image
```

| Symptom | Action |
|---------|--------|
| Robot hugs inner (green island) edge | Increase `FISHEYE_OUTWARD_BIAS_PX` or `LANE_CENTER_OFFSET_FRAC` |
| Robot hugs outer boundary | Decrease `FISHEYE_OUTWARD_BIAS_PX` or `LANE_CENTER_OFFSET_FRAC` |
| Robot misaligned when only yellow visible | Adjust `YELLOW_TARGET_FRAC` (0.5 = centre, 1.0 = far right) |
| Robot off-centre when only inner white visible | Adjust `LEFT_EDGE_TARGET_FRAC` |

---

## Search behaviour tuning

```python
SEARCH_OMEGA  = 0.25   # rad/s (rotation speed when searching)
LANE_LOST_TTL = 2      # frames before switching to SEARCHING
```

| Symptom | Action |
|---------|--------|
| Robot enters SEARCHING too quickly | Increase `LANE_LOST_TTL` (e.g. 3) |
| Robot coasts for too long after losing lane | Decrease `LANE_LOST_TTL` (e.g. 1) |
| Robot rotates past the lane while searching | Reduce `SEARCH_OMEGA` (e.g. 0.15) |
| Robot can't rotate fast enough to find lane | Increase `SEARCH_OMEGA` (e.g. 0.35) |

---

## Tuning workflow summary

```
1. Run diagnose_camera → verify masks hit real markings
2. Fix HSV thresholds if needed
3. Adjust ROI if markings appear outside the scanned band
4. Adjust contour area limits if real blobs are filtered out
5. Set KI=0, KD=0 → tune KP until robot tracks but oscillates slightly
6. Reduce KP by 20% → stable but may drift
7. Add KD slowly to damp residual oscillation
8. Add KI slowly to eliminate steady-state offset
9. Tune fisheye constants (FISHEYE_OUTWARD_BIAS_PX, etc.) for lateral centering
10. Adjust LINEAR_MAX / LINEAR_MIN to taste
```
