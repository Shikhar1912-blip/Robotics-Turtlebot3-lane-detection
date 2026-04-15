# Code Style

All Python code in this repository follows these conventions.
Edits should match the existing style — do not reformat surrounding unchanged code.

---

## General

- **Language**: Python 3.10+, ROS 2 Humble idioms
- **Line length**: no hard limit enforced, but keep lines readable (≤ 100 chars where possible)
- **Encoding**: UTF-8, Unix line endings
- **Shebang**: `#!/usr/bin/env python3` on all executable scripts
- **Docstrings**: module-level triple-quote docstring in every file; method docstrings where logic is non-obvious

---

## Naming conventions

| Construct | Convention | Example |
|-----------|-----------|---------|
| Module-level constants | `UPPER_SNAKE_CASE` | `LINEAR_MAX`, `ROI_TOP_FRAC` |
| Classes | `PascalCase` | `LaneDetectorNode`, `State` |
| Methods | `_leading_underscore` for private, `lowercase` for public | `_compute_error`, `image_callback` |
| Local variables | `snake_case` | `left_x`, `roi_y0`, `spawn_yaw` |
| ROS node names | `snake_case` string | `'lane_detector'`, `'respawn_robot'` |
| Model / entity names in SDF | `snake_case` | `outer_white_left`, `dash_ts_1` |

---

## Constants style

All tuneable constants are **module-level globals** (not class attributes, not ROS parameters).
Group them at the top of the file with aligned comments:

```python
# ── Group heading ─────────────────────────────────
LINEAR_MAX       = 0.10     # m/s — brief explanation
LINEAR_MIN       = 0.04     # m/s — brief explanation
ANGULAR_MAX      = 0.6      # rad/s clamp on output
```

- Use `=` alignment within a group (pad with spaces so `=` signs line up)
- Every constant gets an inline comment with units and a brief effect description
- Units always go first: `# m/s`, `# px²`, `# rad/s`, `# frames`

---

## Comment style

```python
# ── Section heading ─────────────────────────────── (use ── … ──)
# Regular inline comment — starts with capital letter

# Multi-line explanation uses standard # blocks, not block strings,
# so they don't interfere with docstrings.
```

- Section headers use `# ── text ─────` pattern (Unicode em-dash `─`, ASCII hyphen `-` for padding)
- Don't add docstrings or comments to code you didn't change
- Explain *why*, not *what* — the code already shows what

---

## Imports

Order (PEP 8 with ROS convention):
1. Standard library (`os`, `sys`, `math`, `random`)
2. Third-party (`cv2`, `numpy`)
3. ROS packages (`rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`)

One blank line between groups. No wildcard imports (`from x import *`).

```python
import os
import math
import random

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
```

---

## ROS node boilerplate

Every node follows this exact pattern:

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # publishers / subscribers
        # initial state

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Error handling

- Use `try/except` only around actual I/O boundaries (cv_bridge, file open, service calls)
- Log exceptions with `self.get_logger().error(f'context: {exc}')` — not `print()`
- Use `self.get_logger().warn(...)` (not `.warning(...)`) — ROS 2 Humble uses `warn`
- Throttle repeated diagnostic logs: `throttle_duration_sec=0.5`

---

## SDF / world file style

- One model per `<model>` element (no shared links)
- Visual-only models omit `<collision>` entirely
- Inline single-link models on one line for dash elements (24 dashes — brevity matters):
  ```xml
  <model name="dash_ts_1"><static>true</static><pose>...</pose><link name="link"><visual name="v">...</visual></link></model>
  ```
- Multi-element models (ground plane, island) use multi-line indented format
- Section comments use `<!-- ══ LAYER N — description ══ -->` pattern
- All z-offsets are multiples of 0.001 m; visual height is always 0.002 m

---

## What NOT to do

- Do not add type annotations to existing functions unless the PR is specifically about typing
- Do not add `logging.getLogger` — use `self.get_logger()` throughout
- Do not expose tuning constants as ROS parameters — edit source directly
- Do not use `print()` inside ROS nodes — use the ROS logger
- Do not use `time.sleep()` inside callbacks — use timers or `TimerAction` in launch files
- Do not use `cv2.MORPH_CLOSE` on lane masks — it merges sky blobs with road markings
