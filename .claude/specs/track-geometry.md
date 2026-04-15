# Track Geometry — Capsule (Stadium) Track

Source: `worlds/two_lane_circle.world`

---

## Shape Overview

```
         ←────── 4.0 m (straight) ──────→
    ┌────────────────────────────────────────┐  ─┐
   ╱                                          ╲   │
  │   ══════════════════════════════════════   │  │ outer white (1.70 m)
  │  ┌──────────────────────────────────────┐  │  │
  │  │ - - - - - - - - - - - - - - - - - -  │  │  │ road surface + dashes
  │  └──────────────────────────────────────┘  │  │ inner white (1.00–1.06 m)
  │             (green island)                 │  │
   ╲                                          ╱   │
    └────────────────────────────────────────┘  ─┘
```

---

## Coordinate System

| Axis | Direction |
|------|-----------|
| X | Along the straight sections (left = −x, right = +x) |
| Y | Across the track width (top straight at +y, bottom straight at −y) |
| Z | Vertical (upward) |

- **Left semicircle centre**: (−2.0, 0, 0)
- **Right semicircle centre**: (+2.0, 0, 0)
- **Track origin**: world origin (0, 0, 0)

---

## Radii (from each semicircle centre)

| Boundary | Radius | Visual colour |
|----------|--------|---------------|
| Outer edge of outer white line | 1.70 m | White |
| Inner edge of outer white line / road outer edge | 1.64 m | Dark gray |
| **Yellow centre dashes / lane centre** | **1.35 m** | Yellow |
| Inner edge of road / outer edge of inner white | 1.06 m | White |
| Inner edge of inner white / island edge | 1.00 m | White → Green |
| Green island | 0–1.00 m | Green |

**Lane centre on straight sections**: y = ±1.35 m

---

## Layer Stack (z-ordering)

Each layer is a thin disc (cylinder) for the curved ends + a box for the straight middle.

| Layer | z offset | Radius (disc) | Box size (L × W) | Colour |
|-------|----------|--------------|-----------------|--------|
| Outer white boundary | 0.001 | 1.70 m | 4.0 × 3.40 m | White (0.95, 0.95, 0.95) |
| Road surface | 0.002 | 1.64 m | 4.0 × 3.28 m | Dark gray (0.18, 0.18, 0.18) |
| Inner white boundary | 0.003 | 1.06 m | 4.0 × 2.12 m | White (0.95, 0.95, 0.95) |
| Green inner island | 0.004 | 1.00 m | 4.0 × 2.00 m | Green (0.18, 0.42, 0.18) |
| Yellow dashes | 0.005 | — (boxes only) | 0.4 × 0.06 m each | Yellow (0.9, 0.75, 0.0) |

All layer heights are 0.002 m. The z-stacking ensures higher layers visually occlude lower ones.

---

## Yellow Dash Layout

| Parameter | Value |
|-----------|-------|
| Dash length | 0.40 m |
| Dash width | 0.06 m |
| Gap between dashes | 0.25 m |
| Period (dash + gap) | 0.65 m |
| Dashes per straight section | 6 |
| Dashes per semicircle | 6 |
| **Total dashes** | **24** |

### Straight section dash positions

Margin on each end: `(4.0 − 6×0.4 − 5×0.25) / 2 = 0.175 m`

| Dash | x centre | y (top) | y (bottom) | Yaw |
|------|----------|---------|------------|-----|
| 1 | −1.625 | +1.35 | −1.35 | 0 |
| 2 | −0.975 | +1.35 | −1.35 | 0 |
| 3 | −0.325 | +1.35 | −1.35 | 0 |
| 4 | +0.325 | +1.35 | −1.35 | 0 |
| 5 | +0.975 | +1.35 | −1.35 | 0 |
| 6 | +1.625 | +1.35 | −1.35 | 0 |

### Curved section formula

```
r           = 1.35 m  (lane centre radius)
arc_length  = π × 1.35 = 4.241 m
margin_arc  = (4.241 − 3.65) / 2 = 0.296 m
θ_start     = ±π/2 + margin_arc/r = ±1.571 + 0.219 = start angle
θ_step      = 0.65 / 1.35 = 0.4815 rad  (≈ 27.6°)

x   = cx + 1.35·cos(θ)     (cx = ±2.0 for right/left curve)
y   =      1.35·sin(θ)
yaw = θ + π/2               (tangential alignment)
```

---

## Robot Spawn Position

| Parameter | Value |
|-----------|-------|
| Location | Top straight, lane centre |
| x | `random.uniform(−1.5, 1.5)` |
| y | +1.35 m |
| yaw | 0.0 rad (facing +x, direction of travel) |
| z | 0.01 m (slight lift above ground) |

---

## Traffic Direction

**Indian Left-Hand Traffic (LHT):**
- Top straight → robot travels in **+x** direction
- Right semicircle → robot curves **downward** (through positive-x)
- Bottom straight → robot travels in **−x** direction
- Left semicircle → robot curves **upward** (through negative-x)
- Overall direction: **clockwise** when viewed from above

The lane detector keeps the robot in the **inner (left) lane** relative to the direction of travel.
