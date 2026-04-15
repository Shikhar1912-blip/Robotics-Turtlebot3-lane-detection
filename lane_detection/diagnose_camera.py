#!/usr/bin/env python3
"""
Camera + lane-detection diagnostic tool.

Run WHILE the simulation is live:
  ros2 run lane_detection diagnose_camera

What it does
------------
1. Waits for ONE frame from /camera/image_raw.
2. Saves the raw BGR frame to   /tmp/diag_raw.png
3. Saves the ROI HSV frame to   /tmp/diag_hsv_roi.png
4. Applies the current white/yellow masks and saves them.
5. Prints a per-channel HSV histogram summary (min/max/mean/non-zero%).
6. Reports the pixel counts that pass each colour threshold.
7. Quits automatically after the analysis.
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

ROI_TOP_FRAC     = 0.50

# ── Exact thresholds from lane_detector_node.py ───────────────────────────────
WHITE_LO  = np.array([  0,   0, 160], dtype=np.uint8)
WHITE_HI  = np.array([180,  80, 255], dtype=np.uint8)
YELLOW_LO = np.array([ 10,  80,  80], dtype=np.uint8)
YELLOW_HI = np.array([ 40, 255, 255], dtype=np.uint8)


class DiagNode(Node):
    def __init__(self):
        super().__init__('diag_camera')
        self.bridge = CvBridge()
        self.done   = False
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._cb, 10)
        self.get_logger().info(
            'Waiting for /camera/image_raw …  (Ctrl-C to abort)')

    def _cb(self, msg: Image):
        if self.done:
            return
        self.done = True

        self.get_logger().info(
            f'Frame received — encoding={msg.encoding} '
            f'size={msg.width}×{msg.height} '
            f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            raise SystemExit(1)

        h, w = bgr.shape[:2]
        roi_y0 = int(h * ROI_TOP_FRAC)
        roi    = bgr[roi_y0:h, :]

        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white_mask  = cv2.inRange(hsv, WHITE_LO,  WHITE_HI)
        yellow_mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)

        # ── Save images ───────────────────────────────────────────────────────
        cv2.imwrite('/tmp/diag_raw.png', bgr)
        cv2.imwrite('/tmp/diag_roi_bgr.png', roi)
        cv2.imwrite('/tmp/diag_roi_hsv.png', hsv)
        cv2.imwrite('/tmp/diag_white_mask.png', white_mask)
        cv2.imwrite('/tmp/diag_yellow_mask.png', yellow_mask)

        # Overlay masks on ROI for easy visual inspection
        overlay = roi.copy()
        overlay[white_mask  > 0] = (255, 100,   0)   # blue  = white mask hit
        overlay[yellow_mask > 0] = (  0, 220, 220)   # cyan  = yellow mask hit
        cv2.imwrite('/tmp/diag_overlay.png', overlay)

        # ── HSV channel statistics ────────────────────────────────────────────
        roi_px   = hsv.reshape(-1, 3).astype(np.float32)
        n_pixels = roi_px.shape[0]

        print('\n' + '='*60)
        print(f'IMAGE SIZE         : {w}×{h}  (ROI rows {roi_y0}:{h})')
        print(f'ROI pixels         : {n_pixels}')
        print()
        for i, ch in enumerate(['Hue', 'Sat', 'Val']):
            col = roi_px[:, i]
            print(f'  {ch:3s}  min={col.min():.0f}  max={col.max():.0f}  '
                  f'mean={col.mean():.1f}  '
                  f'non-zero={100*np.count_nonzero(col)/n_pixels:.1f}%')

        # ── White pixel analysis ──────────────────────────────────────────────
        wh_px  = np.count_nonzero(white_mask)
        ye_px  = np.count_nonzero(yellow_mask)
        print()
        print(f'WHITE mask hits    : {wh_px} px  ({100*wh_px/n_pixels:.2f}% of ROI)')
        print(f'YELLOW mask hits   : {ye_px} px  ({100*ye_px/n_pixels:.2f}% of ROI)')

        if wh_px == 0 and ye_px == 0:
            print()
            print('*** ZERO hits — thresholds do NOT match the image content.')
            print('    Checking what the brightest pixels actually look like …')
            # Find top-10% brightest pixels by V channel
            v_channel = hsv[:, :, 2]
            thresh_v  = np.percentile(v_channel, 90)
            bright    = hsv[v_channel >= thresh_v]
            if len(bright):
                print(f'    Top-10% bright pixels  '
                      f'H: {bright[:,0].min()}-{bright[:,0].max()}  '
                      f'S: {bright[:,1].min()}-{bright[:,1].max()}  '
                      f'V: {bright[:,2].min()}-{bright[:,2].max()}')

            # Check if ROI is mostly uniform (camera pointing at sky/wall)
            v_std = np.std(v_channel)
            print(f'    V-channel std-dev      : {v_std:.1f}  '
                  f'(< 10 suggests a blank/uniform image)')

        # ── Broader threshold test ────────────────────────────────────────────
        print()
        print('Broader threshold test (V >= 120, S <= 120 for white-ish):')
        broad_white = cv2.inRange(hsv,
                                  np.array([0,   0, 120], dtype=np.uint8),
                                  np.array([180, 120, 255], dtype=np.uint8))
        bw_px = np.count_nonzero(broad_white)
        print(f'  Broad-white hits : {bw_px} px  ({100*bw_px/n_pixels:.2f}%)')

        print()
        print('Broader threshold test (H 5-50, S >= 40, V >= 40 for yellow-ish):')
        broad_yellow = cv2.inRange(hsv,
                                   np.array([ 5,  40,  40], dtype=np.uint8),
                                   np.array([50, 255, 255], dtype=np.uint8))
        by_px = np.count_nonzero(broad_yellow)
        print(f'  Broad-yellow hits: {by_px} px  ({100*by_px/n_pixels:.2f}%)')

        print()
        print('Saved images:')
        for path in ['/tmp/diag_raw.png', '/tmp/diag_roi_bgr.png',
                     '/tmp/diag_white_mask.png', '/tmp/diag_yellow_mask.png',
                     '/tmp/diag_overlay.png']:
            print(f'  {path}')
        print('='*60)
        print()
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DiagNode()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        pass
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
