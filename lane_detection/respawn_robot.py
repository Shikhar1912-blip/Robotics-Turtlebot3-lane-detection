#!/usr/bin/env python3
"""
Respawn the TurtleBot3 at a random position on the road ring.

Usage (while simulation is running):
    ros2 run lane_detection respawn_robot

What it does
------------
1. Calls /delete_entity to remove the current robot from Gazebo.
2. Computes a new random (x, y, yaw) on the road ring (radius 2.95–3.95 m).
3. Reads the robot SDF file.
4. Calls /spawn_entity to place the robot at the new pose.
5. Prints the new spawn coordinates and exits.

This solves the problem where Gazebo's "Reset World" only resets physics
but does NOT re-randomize the spawn position (because spawn_entity.py
only runs once during the original launch).
"""

import os
import sys
import math
import random

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion


# ── Road ring geometry (must match two_lane_circle.world) ─────────────────────
SPAWN_RADIUS_MIN = 2.95   # inner edge of road ring (just outside inner white at 2.85 m)
SPAWN_RADIUS_MAX = 3.95   # outer edge of road ring (just inside outer white at 4.2 m)
SPAWN_Z          = 0.01   # slight lift to avoid ground-plane penetration

ENTITY_NAME      = 'turtlebot3_burger_cam'

# ── Timeout for service calls ─────────────────────────────────────────────────
SERVICE_TIMEOUT  = 10.0   # seconds


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _random_road_pose() -> tuple:
    """
    Return (x, y, yaw) at a random position on the road ring.
    Yaw is set tangential to the circle (facing along the road).
    """
    angle  = random.uniform(0, 2 * math.pi)
    radius = random.uniform(SPAWN_RADIUS_MIN, SPAWN_RADIUS_MAX)
    x      = radius * math.cos(angle)
    y      = radius * math.sin(angle)
    yaw    = angle + math.pi / 2   # tangential to circle
    return x, y, yaw


class RespawnNode(Node):
    def __init__(self):
        super().__init__('respawn_robot')

        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')

        # Load robot SDF
        tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
        sdf_path = os.path.join(
            tb3_gazebo_pkg, 'models', 'turtlebot3_burger_cam', 'model.sdf')
        with open(sdf_path, 'r') as f:
            self._robot_sdf = f.read()
        self.get_logger().info(f'Loaded robot SDF from {sdf_path}')

    def run(self) -> bool:
        """Delete existing entity, then spawn at a new random pose. Returns True on success."""

        # ── Step 1: Wait for services ─────────────────────────────────────────
        self.get_logger().info('Waiting for /delete_entity service…')
        if not self._delete_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error('/delete_entity service not available — is Gazebo running?')
            return False

        self.get_logger().info('Waiting for /spawn_entity service…')
        if not self._spawn_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error('/spawn_entity service not available — is Gazebo running?')
            return False

        # ── Step 2: Delete existing robot ─────────────────────────────────────
        self.get_logger().info(f'Deleting entity "{ENTITY_NAME}"…')
        del_req = DeleteEntity.Request()
        del_req.name = ENTITY_NAME

        del_future = self._delete_client.call_async(del_req)
        rclpy.spin_until_future_complete(self, del_future, timeout_sec=SERVICE_TIMEOUT)

        if del_future.result() is not None:
            result = del_future.result()
            if result.success:
                self.get_logger().info(f'Deleted "{ENTITY_NAME}": {result.status_message}')
            else:
                self.get_logger().warn(
                    f'Delete returned success=False: {result.status_message} '
                    f'(entity may not exist yet — proceeding with spawn)')
        else:
            self.get_logger().warn('Delete service call timed out — proceeding with spawn')

        # ── Step 3: Compute random spawn pose ─────────────────────────────────
        x, y, yaw = _random_road_pose()
        self.get_logger().info(
            f'New spawn: x={x:.3f}  y={y:.3f}  yaw={yaw:.3f} rad '
            f'({math.degrees(yaw):.1f}°)  '
            f'radius={math.sqrt(x*x + y*y):.3f} m')

        # ── Step 4: Spawn at new pose ─────────────────────────────────────────
        spawn_req = SpawnEntity.Request()
        spawn_req.name = ENTITY_NAME
        spawn_req.xml  = self._robot_sdf
        spawn_req.robot_namespace = ''
        spawn_req.reference_frame = 'world'
        spawn_req.initial_pose = Pose(
            position=Point(x=x, y=y, z=SPAWN_Z),
            orientation=_yaw_to_quaternion(yaw),
        )

        self.get_logger().info(f'Spawning "{ENTITY_NAME}" at new pose…')
        spawn_future = self._spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, spawn_future, timeout_sec=SERVICE_TIMEOUT)

        if spawn_future.result() is not None:
            result = spawn_future.result()
            if result.success:
                self.get_logger().info(f'Spawned successfully: {result.status_message}')
                return True
            else:
                self.get_logger().error(f'Spawn failed: {result.status_message}')
                return False
        else:
            self.get_logger().error('Spawn service call timed out')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = RespawnNode()
    try:
        success = node.run()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
