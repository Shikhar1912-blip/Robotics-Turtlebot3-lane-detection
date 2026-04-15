import os
import subprocess
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _start_xvfb(display_num=1, width=1280, height=1024, depth=24):
    """
    Start Xvfb on :<display_num> if not already running.
    Returns the display string, e.g. ':1'.
    Xvfb provides a virtual X11 server with software GLX, which is required
    for Gazebo's OGRE renderer (camera sensors) on WSL2 where the real
    X11 server (WSLg/XWayland) lacks GLX support.
    """
    display = f':{display_num}'
    # Check if an X server is already running on this display
    result = subprocess.run(
        ['xdpyinfo', '-display', display],
        capture_output=True)
    if result.returncode == 0:
        return display  # already running

    subprocess.Popen(
        ['Xvfb', display,
         '-screen', '0', f'{width}x{height}x{depth}',
         '+extension', 'GLX',
         '+extension', 'RANDR',
         '+render', '-noreset'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # Wait briefly for Xvfb to start
    import time
    for _ in range(10):
        time.sleep(0.5)
        r = subprocess.run(['xdpyinfo', '-display', display],
                           capture_output=True)
        if r.returncode == 0:
            break
    return display


def generate_launch_description():

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    pkg_path = os.path.join(
        os.path.expanduser('~'), 'turtlebot3_lane_ws/src/lane_detection')

    # ── Start Xvfb for gzserver (provides software GLX for camera sensors) ────
    xvfb_display = _start_xvfb(display_num=1)

    # ── Random spawn on the top straight of the capsule track ────────────────
    # Track: capsule shape, straight sections along X-axis.
    # Lane centre on top straight: y = +1.35 m, x in [−2.0, +2.0].
    # Robot spawns at a random position on the top straight facing +x (yaw=0).
    #
    # NOTE: These coordinates are computed ONCE at launch time.
    # Gazebo's "Reset World" (GUI or /reset_simulation service) resets physics
    # but does NOT re-run this code — the robot returns to THIS position.
    # To respawn at a NEW random position, use:
    #     ros2 run lane_detection respawn_robot
    spawn_x   = random.uniform(-1.5, 1.5)   # within the 4.0 m straight
    spawn_y   = 1.35                          # top-straight lane centre
    spawn_yaw = 0.0                           # facing +x along the straight

    # ── File paths ────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_path, 'worlds', 'two_lane_circle.world')

    robot_sdf = os.path.join(
        tb3_gazebo_pkg, 'models', 'turtlebot3_burger_cam', 'model.sdf')

    robot_urdf_path = os.path.join(
        tb3_gazebo_pkg, 'urdf', 'turtlebot3_burger_cam.urdf')
    with open(robot_urdf_path, 'r') as f:
        robot_desc = f.read()

    gazebo_model_path = (
        os.path.join(tb3_gazebo_pkg, 'models')
        + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # ── gzserver — physics + plugins + camera rendering (uses Xvfb GLX) ──────
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'GAZEBO_IP': '127.0.0.1',
            'DISPLAY': xvfb_display,       # Xvfb: camera sensors work via software GLX
            'LIBGL_ALWAYS_SOFTWARE': '1',  # Mesa software rasterizer
            'OGRE_RTT_MODE': 'Copy',       # prevents OGRE render-to-texture crash
        },
    )

    # ── gzclient — 3D GUI window, shown on WSLg desktop ─────────────────────
    # Uses the real WSLg display (:0) so the window appears on screen.
    # Delayed 5 s to let gzserver finish loading the world.
    gzclient = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient', '--verbose'],
                output='screen',
                additional_env={
                    'GAZEBO_IP': '127.0.0.1',
                    'DISPLAY': ':0',           # WSLg: renders in your Windows desktop
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'OGRE_RTT_MODE': 'Copy',
                },
            )
        ],
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }],
    )

    # ── Spawn robot at t=6 s ──────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger_cam',
                    '-file',   robot_sdf,
                    '-x',      str(spawn_x),
                    '-y',      str(spawn_y),
                    '-z',      '0.01',
                    '-Y',      str(spawn_yaw),
                ],
                output='screen',
            )
        ],
    )

    # ── Lane detector at t=9 s ────────────────────────────────────────────────
    lane_detector = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='lane_detection',
                executable='lane_detector_node',
                name='lane_detector',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        lane_detector,
    ])
