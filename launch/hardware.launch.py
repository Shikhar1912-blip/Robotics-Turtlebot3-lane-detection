#!/usr/bin/env python3
"""
Hardware launch file for physical TurtleBot3 with OV2710 USB camera.

Starts:
  1. usb_cam_node  — reads /dev/video0 (OV2710), publishes /camera/image_raw at 15 Hz
  2. lane_detector_node — subscribes to /camera/image_raw, publishes /cmd_vel

Usage:
    ros2 launch lane_detection hardware.launch.py
    ros2 launch lane_detection hardware.launch.py device:=/dev/video1
    ros2 launch lane_detection hardware.launch.py width:=1280 height:=720

Verify:
    ros2 topic hz /camera/image_raw        # expect ~15 Hz
    ros2 run rqt_image_view rqt_image_view # subscribe to /lane_detection/debug_img
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video0',
        description='V4L2 device path for the OV2710 USB camera')

    width_arg = DeclareLaunchArgument(
        'width', default_value='1920',
        description='Capture width in pixels')

    height_arg = DeclareLaunchArgument(
        'height', default_value='1080',
        description='Capture height in pixels')

    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='15.0',
        description='Camera capture frame rate (Hz)')

    # ── OV2710 USB camera node ────────────────────────────────────────────────
    # namespace='camera' makes the node publish to /camera/image_raw and
    # /camera/camera_info, which matches the lane_detector_node subscription.
    #
    # pixel_format 'mjpeg': OV2710 hardware encodes MJPEG natively — lowest
    # USB bandwidth; allows 1920x1080 at 15 Hz without saturating the bus.
    # usb_cam decompresses MJPEG internally and publishes sensor_msgs/Image (bgr8).
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device':       LaunchConfiguration('device'),
            'image_width':        LaunchConfiguration('width'),
            'image_height':       LaunchConfiguration('height'),
            'pixel_format':       'mjpeg',
            'framerate':          LaunchConfiguration('framerate'),
            'camera_name':        'camera',
            'camera_frame_id':    'camera_link',
            # Leave V4L2 exposure/colour controls at driver defaults (-1 = no override)
            'brightness':         -1,
            'contrast':           -1,
            'saturation':         -1,
            'sharpness':          -1,
            'auto_white_balance': True,
        }],
    )

    # ── Lane detector ─────────────────────────────────────────────────────────
    # use_sim_time defaults to False — hardware uses the real wall clock.
    lane_detector = Node(
        package='lane_detection',
        executable='lane_detector_node',
        name='lane_detector',
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        framerate_arg,
        camera_node,
        lane_detector,
    ])
