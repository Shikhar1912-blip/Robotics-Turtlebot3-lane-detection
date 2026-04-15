#!/usr/bin/env python3
"""
Lane detector for TurtleBot3 - Indian Left-Hand Traffic.

Subscribes : /camera/image_raw         (sensor_msgs/Image)
Publishes  : /cmd_vel                  (geometry_msgs/Twist)
             /lane_detection/debug_img (sensor_msgs/Image)   ← HSV mask overlay

State machine:
  SEARCHING  → no lane edges visible for LANE_LOST_TTL frames → rotate in place
  FOLLOWING  → at least one usable detection → PID-controller, drive forward

Controller:
  angular_z = -(KP * error + KI * integral(error) + KD * d_error/dt)
  Integral has anti-windup clamp; derivative uses low-pass filter.
  linear_x  scales from LINEAR_MAX down to LINEAR_MIN as |angular_z| grows
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# ── Tuneable constants ───────────────────────────────────────────────────────────────────────
LINEAR_MAX       = 0.10     # m/s forward speed at zero angular command
                             # reduced from 0.12: at 1.7 Hz the robot travels ~6 cm/frame;
                             # slower speed gives the controller more frames to correct
LINEAR_MIN       = 0.04     # m/s forward speed at max angular command
ANGULAR_MAX      = 0.6      # rad/s clamp on angular output

# ── PID gains ───────────────────────────────────────────────────────────────────────
# Proportional: maps pixel error to angular velocity
KP               = 0.008    # slightly higher than old 0.006 for better curve tracking
# Integral: eliminates steady-state error on constant-curvature roads
# On a circle of radius ~3.15 m at 0.10 m/s the robot needs omega ~ 0.032 rad/s.
# Without KI the P-term alone requires a ~4 px steady-state offset to produce this.
KI               = 0.0003   # integral gain  (pixels*s -> rad/s)
INTEGRAL_MAX     = 150.0    # anti-windup clamp on integral accumulator (pixels*seconds)
# Derivative: damps oscillations, but unreliable at 1.7 Hz -- use with low-pass filter
KD               = 0.001    # reduced from 0.002: at 1.7 Hz derivative is noisy
DERIVATIVE_ALPHA = 0.3      # low-pass filter coefficient for derivative
                             # 0 = ignore derivative completely, 1 = use raw (noisy)
                             # 0.3 = 70% previous + 30% new -- smooths out jitter

SEARCH_OMEGA     = 0.25     # rad/s rotation speed when searching for lanes
# ── ROI: the horizon band where lane markings actually appear ─────────────────
# Camera is at 9.3 cm height, forward-facing 182° fisheye.
# Lane markings appear as a thin strip at the horizon (~40–60% from top).
# Bottom of image = green inner island; top = gray sky.  Scan the middle.
ROI_TOP_FRAC     = 0.35     # ROI top    = 35% from top
ROI_BOT_FRAC     = 0.70     # ROI bottom = 70% from top  (35-px margin above/below markings)
MIN_CONTOUR_AREA = 20       # px² — ignore speckle
MAX_CONTOUR_AREA = 2500     # px² — ignore sky/background (sky blob is >10 000 px²; lane strips are <2500)
# Camera runs at ~1.7 Hz under software rendering → 1 frame ≈ 0.6 s
LANE_LOST_TTL    = 2        # 2 consecutive misses (~1.2 s) before SEARCHING
DEBUG_EVERY_N    = 1        # publish debug image every frame (1.7 Hz is already low)

# ── Lane-centering constants (fisheye compensation) ───────────────────────────
# On a 182° fisheye, pixel distances are nonlinear — objects near image edges
# are compressed.  These constants shift the lane-center target outward to
# compensate for the lens distortion that biases the robot toward the inner edge.
LANE_CENTER_OFFSET_FRAC  = 0.35   # fraction of detected road width from inner edge to target
                                   # 0.25 = quarter (old default), 0.35 = slightly outward
LEFT_EDGE_TARGET_FRAC    = 0.30   # where inner-white should sit in the image (fraction of width)
                                   # 0.25 = old default, 0.30 = push robot further from inner edge
YELLOW_TARGET_FRAC       = 0.70   # where yellow center should sit (fraction of image width)
                                   # 0.75 = old default, 0.70 = slightly less aggressive
FISHEYE_OUTWARD_BIAS_PX  = 10     # additional px added to lane-center x to push target outward
                                   # compensates for fisheye compression at image periphery

# ── HSV colour thresholds ──────────────────────────────────────────────────────
# White lane boundaries — broader thresholds for Gazebo rendering variability
WHITE_LO  = np.array([  0,   0, 160], dtype=np.uint8)
WHITE_HI  = np.array([180,  80, 255], dtype=np.uint8)
# Yellow center dashes — wider hue/sat range
YELLOW_LO = np.array([ 10,  80,  80], dtype=np.uint8)
YELLOW_HI = np.array([ 40, 255, 255], dtype=np.uint8)


class State:
    SEARCHING = 'SEARCHING'
    FOLLOWING = 'FOLLOWING'


class LaneDetectorNode(Node):

    def __init__(self):
        super().__init__('lane_detector')

        self.sub       = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub       = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(
            Image, '/lane_detection/debug_img', 10)
        self.bridge    = CvBridge()

        self.state           = State.SEARCHING
        self.lost_counter    = 0
        self._prev_error     = 0.0
        self._integral       = 0.0       # PID integral accumulator (pixels*seconds)
        self._filtered_deriv = 0.0       # low-pass filtered derivative (pixels/s)
        self._last_time      = None      # seconds (float), set on first callback
        self._frame_count    = 0

        self.get_logger().info('LaneDetectorNode started — Indian LHT mode (PID controller)')

    # ──────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        self._frame_count += 1
        now = self.get_clock().now().nanoseconds * 1e-9

        h, w = frame.shape[:2]
        img_cx = w // 2

        # ── Region of interest: horizon band (35%–70% from top) ─────────────
        # Lane markings appear at the horizon in the fisheye image.
        # Bottom rows = green inner island; top rows = sky. Scan the middle.
        roi_y0 = int(h * ROI_TOP_FRAC)
        roi_y1 = int(h * ROI_BOT_FRAC)
        roi    = frame[roi_y0:roi_y1, :]
        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── Colour masks ──────────────────────────────────────────────────────
        white_mask  = cv2.inRange(hsv, WHITE_LO,  WHITE_HI)
        yellow_mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)

        # Morphological cleanup — OPEN only (remove speckle).
        # No CLOSE: it was merging the sky blob with road markings into one giant contour.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        white_mask  = cv2.morphologyEx(white_mask,  cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        # ── Detect lane features ──────────────────────────────────────────────
        left_x   = self._edge_centroid(white_mask,  side='left')
        right_x  = self._edge_centroid(white_mask,  side='right')
        yellow_x = self._largest_centroid(yellow_mask)

        # Log every frame so we can see what's being detected (camera is ~1.7 Hz)
        w_px = int(np.count_nonzero(white_mask))
        y_px = int(np.count_nonzero(yellow_mask))

        # ── Debug image (publish every N frames to save bandwidth) ────────────
        if self._frame_count % DEBUG_EVERY_N == 0:
            self._publish_debug(frame, roi_y0, roi_y1, white_mask, yellow_mask,
                                left_x, right_x, yellow_x, img_cx)

        # ── Compute steering error ────────────────────────────────────────────
        error, valid, case = self._compute_error(left_x, right_x, yellow_x, img_cx)

        # Diagnostic log — shows which detection case fires and the raw error
        self.get_logger().info(
            f'[Case{case}] L={left_x} R={right_x} Y={yellow_x} '
            f'white={w_px}px yellow={y_px}px err={error:+d} state={self.state}',
            throttle_duration_sec=0.5)

        # ── PID controller time-step ──────────────────────────────────────────────
        if self._last_time is None:
            dt = 0.6                      # ~1.7 Hz actual frame rate (not 30 Hz)
        else:
            dt = max(now - self._last_time, 0.005)   # clamp to avoid div-by-zero
        self._last_time = now

        # ── State machine ─────────────────────────────────────────────────────
        if valid:
            self.lost_counter = 0
            if self.state != State.FOLLOWING:
                self.get_logger().info('→ FOLLOWING')
                self.state = State.FOLLOWING
            self._cmd_follow(error, dt)
            self._prev_error = error
        else:
            self.lost_counter += 1
            self._prev_error     = 0.0    # reset derivative on lost detection
            self._filtered_deriv = 0.0    # clear filtered derivative too
            if self.lost_counter >= LANE_LOST_TTL:
                if self.state != State.SEARCHING:
                    self.get_logger().info('→ SEARCHING')
                    self.state = State.SEARCHING
                    self._integral = 0.0  # reset integral to prevent windup
            if self.state == State.SEARCHING:
                self._cmd_search()
            else:
                # Grace period: keep moving forward, no correction
                self._cmd_follow(0.0, dt)

    # ── Feature detectors ─────────────────────────────────────────────────────

    def _edge_centroid(self, mask: np.ndarray, side: str):
        """
        Return x-centroid of the leftmost (side='left') or rightmost (side='right')
        qualifying white blob, or None.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours
                 if MIN_CONTOUR_AREA < cv2.contourArea(c) < MAX_CONTOUR_AREA]
        if not valid:
            return None

        if side == 'left':
            best = min(valid, key=lambda c: cv2.boundingRect(c)[0])
        else:
            best = max(valid,
                       key=lambda c: cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2])

        M = cv2.moments(best)
        return int(M['m10'] / M['m00']) if M['m00'] != 0 else None

    def _largest_centroid(self, mask: np.ndarray):
        """Return x-centroid of the largest qualifying blob, or None."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours
                 if MIN_CONTOUR_AREA < cv2.contourArea(c) < MAX_CONTOUR_AREA]
        if not valid:
            return None
        best = max(valid, key=cv2.contourArea)
        M = cv2.moments(best)
        return int(M['m10'] / M['m00']) if M['m00'] != 0 else None

    # ── Error computation (Indian LHT: stay in inner / left lane) ─────────────

    def _compute_error(self, left_x, right_x, yellow_x, img_cx):
        """
        Returns (error_pixels, is_valid, case_num).

        error > 0  → desired point is to the right of image centre → turn right (−ω)
        error < 0  → desired point is to the left               → turn left  (+ω)

        Fallback priority:
          1a. Yellow + left white   → direct midpoint (most accurate)
          1b. Both white edges      → offset from inner edge using LANE_CENTER_OFFSET_FRAC
          2.  Yellow + right white  → mirror yellow to infer inner boundary
          3.  Yellow only           → steer yellow to YELLOW_TARGET_FRAC of image width
          4.  Left white only       → steer it to LEFT_EDGE_TARGET_FRAC of image width
          5.  Nothing               → not valid

        All cases apply FISHEYE_OUTWARD_BIAS_PX to compensate for the 182° fisheye
        lens compressing objects near the image edges (inner white appears closer
        to center than it physically is).
        """
        w = img_cx * 2  # full image width

        # Case 1a – yellow + left white (best: direct boundaries of the left lane)
        if yellow_x is not None and left_x is not None:
            left_lane_center = (left_x + yellow_x) // 2 + FISHEYE_OUTWARD_BIAS_PX
            return left_lane_center - img_cx, True, 1

        # Case 1b – both white edges (no yellow visible)
        if left_x is not None and right_x is not None:
            road_width       = right_x - left_x
            left_lane_center = left_x + int(road_width * LANE_CENTER_OFFSET_FRAC) + FISHEYE_OUTWARD_BIAS_PX
            return left_lane_center - img_cx, True, 2

        # Case 2 – yellow + right white (mirror yellow to estimate inner boundary)
        if yellow_x is not None and right_x is not None:
            inferred_left    = 2 * yellow_x - right_x
            left_lane_center = (inferred_left + yellow_x) // 2 + FISHEYE_OUTWARD_BIAS_PX
            return left_lane_center - img_cx, True, 3

        # Case 3 – yellow only: keep yellow at YELLOW_TARGET_FRAC of image width
        if yellow_x is not None:
            target = int(w * YELLOW_TARGET_FRAC)
            return yellow_x - target, True, 4

        # Case 4 – left white only: keep it at LEFT_EDGE_TARGET_FRAC of image width
        if left_x is not None:
            target = int(w * LEFT_EDGE_TARGET_FRAC)
            return left_x - target, True, 5

        # Case 5 – nothing detected
        return 0, False, 6

    # ── Publishers ────────────────────────────────────────────────────────────

    def _cmd_follow(self, error: float, dt: float):
        """PID controller with speed scaling, anti-windup, and filtered derivative."""
        # -- Proportional -------------------------------------------------------
        P = KP * error

        # -- Integral with anti-windup ------------------------------------------
        self._integral += error * dt
        self._integral  = float(np.clip(self._integral, -INTEGRAL_MAX, INTEGRAL_MAX))
        I = KI * self._integral

        # -- Derivative with low-pass filter ------------------------------------
        # Raw derivative is noisy at 1.7 Hz: a 3-pixel jitter produces
        # d_error ~ 5 px/s.  The low-pass filter smooths this out.
        raw_deriv            = (error - self._prev_error) / dt
        self._filtered_deriv = (DERIVATIVE_ALPHA * raw_deriv
                                + (1 - DERIVATIVE_ALPHA) * self._filtered_deriv)
        D = KD * self._filtered_deriv

        raw_angular = -(P + I + D)
        angular_z   = float(np.clip(raw_angular, -ANGULAR_MAX, ANGULAR_MAX))

        # Scale linear speed down as angular demand grows
        turn_ratio  = abs(angular_z) / ANGULAR_MAX           # 0 ... 1
        linear_x    = LINEAR_MAX - (LINEAR_MAX - LINEAR_MIN) * turn_ratio

        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = angular_z
        self.pub.publish(twist)

    def _cmd_search(self):
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = float(SEARCH_OMEGA)
        self.pub.publish(twist)

    # ── Debug visualisation ───────────────────────────────────────────────────

    def _publish_debug(self, frame, roi_y0, roi_y1,
                       white_mask, yellow_mask,
                       left_x, right_x, yellow_x, img_cx):
        """
        Overlay white (blue tint) and yellow (yellow tint) masks on the ROI band,
        draw feature x-positions as vertical lines, and publish.
        """
        h, w = frame.shape[:2]
        debug = frame.copy()
        roi_debug = debug[roi_y0:roi_y1, :]

        # Tint white detections blue
        blue_overlay = roi_debug.copy()
        blue_overlay[white_mask > 0] = (255, 100, 0)
        roi_debug[:] = cv2.addWeighted(roi_debug, 0.6, blue_overlay, 0.4, 0)

        # Tint yellow detections yellow
        yellow_overlay = roi_debug.copy()
        yellow_overlay[yellow_mask > 0] = (0, 220, 220)
        roi_debug[:] = cv2.addWeighted(roi_debug, 0.6, yellow_overlay, 0.4, 0)

        # Draw feature lines into the full frame
        full_left_x   = left_x   # x coords are relative to full-width ROI = same as frame
        full_right_x  = right_x
        full_yellow_x = yellow_x

        if full_left_x is not None:
            cv2.line(debug, (full_left_x, roi_y0), (full_left_x, roi_y1), (255, 0, 0), 2)
        if full_right_x is not None:
            cv2.line(debug, (full_right_x, roi_y0), (full_right_x, roi_y1), (0, 0, 255), 2)
        if full_yellow_x is not None:
            cv2.line(debug, (full_yellow_x, roi_y0), (full_yellow_x, roi_y1), (0, 220, 220), 2)

        # Image centre line
        cv2.line(debug, (img_cx, 0), (img_cx, h), (128, 128, 128), 1)
        # ROI band boundaries (green lines)
        cv2.line(debug, (0, roi_y0), (w, roi_y0), (0, 255, 0), 1)
        cv2.line(debug, (0, roi_y1), (w, roi_y1), (0, 255, 0), 1)

        # State label
        label = self.state
        cv2.putText(debug, label, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception as exc:
            self.get_logger().warning(f'debug publish: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
