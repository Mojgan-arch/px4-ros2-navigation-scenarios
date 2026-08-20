#!/usr/bin/env python3
"""
Target Simulator — Phase 22

Animates a virtual moving "target" (a yellow sphere) along a continuous
pattern (circle, by default) and publishes:
  - /target_position   (geometry_msgs/PointStamped)  ENU XYZ
  - /target_marker     (visualization_msgs/Marker)   for RViz

Also drives the Gazebo model named "moving_target" via gz-transport's
/world/<world>/set_pose service every 0.1 s, so the same sphere visibly
moves inside Gazebo. The drone's follow node only needs the ROS topic;
the gz call is just so you can SEE the target moving in Gazebo too.

The follow node (target_follower_node) keeps the drone hovering at a
fixed altitude above this position, so the drone tracks the sphere in
real time.
"""

import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from ...common.interfaces import declare_interface_params


class TargetSimulator(Node):
    def __init__(self):
        super().__init__('target_simulator')
        declare_interface_params(self)

        # ========== Parameters ==========
        # Pattern: 'circle' | 'figure8' | 'line'
        self.declare_parameter('pattern', 'circle')
        self.declare_parameter('radius', 5.0)         # m
        self.declare_parameter('linear_speed', 0.5)   # m/s
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('target_z', 0.5)       # height of sphere centre
        # World name as known by Gazebo (must match the SDF). We call
        # /world/<world>/set_pose to physically move the model.
        self.declare_parameter('gz_world', 'walled_arena')
        self.declare_parameter('gz_model_name', 'moving_target')
        # Set false to skip the Gazebo pose update (useful if the model
        # isn't in the world for some reason). The drone-side node only
        # reads /target_position anyway.
        self.declare_parameter('drive_gazebo', True)

        self.pattern       = self.get_parameter('pattern').value
        self.radius        = float(self.get_parameter('radius').value)
        self.linear_speed  = float(self.get_parameter('linear_speed').value)
        self.center_x      = float(self.get_parameter('center_x').value)
        self.center_y      = float(self.get_parameter('center_y').value)
        self.target_z      = float(self.get_parameter('target_z').value)
        self.gz_world      = self.get_parameter('gz_world').value
        self.gz_model_name = self.get_parameter('gz_model_name').value
        self.drive_gazebo  = bool(self.get_parameter('drive_gazebo').value)

        # Angular speed so that linear_speed = radius * omega
        self.omega = (
            self.linear_speed / self.radius if self.radius > 0 else 0.0
        )

        # ========== State ==========
        self.t0 = time.time()
        self.current_x = self.center_x + self.radius
        self.current_y = self.center_y
        self.gz_lock = threading.Lock()

        # ========== Publishers ==========
        self.pos_pub = self.create_publisher(
            PointStamped, '/target_position', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/target_marker', 10
        )
        self.status_pub = self.create_publisher(
            String, '/target_status', 10
        )

        # ========== Timers ==========
        # ROS topic at 20 Hz (smooth for drone)
        self.create_timer(0.05, self.tick)
        # Marker at 5 Hz (good for RViz)
        self.create_timer(0.2, self.publish_marker)
        # Status at 1 Hz
        self.create_timer(1.0, self.publish_status)
        # Gazebo pose update at 10 Hz (smooth visual motion)
        if self.drive_gazebo:
            self.create_timer(0.1, self.update_gazebo_model)

        self.get_logger().info(
            f'Target Simulator started — pattern={self.pattern}, '
            f'radius={self.radius:.2f} m, speed={self.linear_speed:.2f} m/s'
        )

    # =====================================================
    def compute_position(self, t):
        """Return (x, y) for the target at elapsed time t (seconds)."""
        if self.pattern == 'circle':
            theta = self.omega * t
            x = self.center_x + self.radius * math.cos(theta)
            y = self.center_y + self.radius * math.sin(theta)
        elif self.pattern == 'figure8':
            theta = self.omega * t
            x = self.center_x + self.radius * math.sin(theta)
            y = self.center_y + (self.radius / 2.0) * math.sin(2.0 * theta)
        elif self.pattern == 'line':
            # back-and-forth along X over [-radius, +radius]
            half_period = self.radius / max(self.linear_speed, 1e-3)
            phase = (t / half_period) % 2.0
            d = phase if phase <= 1.0 else 2.0 - phase
            x = self.center_x + (-self.radius + 2.0 * self.radius * d)
            y = self.center_y
        else:
            x, y = self.center_x, self.center_y
        return x, y

    def tick(self):
        t = time.time() - self.t0
        x, y = self.compute_position(t)
        self.current_x, self.current_y = x, y

        msg = PointStamped()
        msg.header.frame_id = self.frame('map')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = self.target_z
        self.pos_pub.publish(msg)

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = self.frame('map')
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.current_x
        m.pose.position.y = self.current_y
        m.pose.position.z = self.target_z
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 1.0
        m.color.g = 0.95
        m.color.b = 0.0
        m.color.a = 0.9
        self.marker_pub.publish(m)

    def publish_status(self):
        s = String()
        s.data = (
            f'pattern={self.pattern} | pos=({self.current_x:+.2f}, '
            f'{self.current_y:+.2f}, {self.target_z:+.2f}) | '
            f'speed={self.linear_speed:.2f} m/s | radius={self.radius:.2f}'
        )
        self.status_pub.publish(s)

    def update_gazebo_model(self):
        """Drive the Gazebo model to the current target position."""
        # Skip if another call is still in flight (rare but possible).
        if not self.gz_lock.acquire(blocking=False):
            return
        try:
            req = (
                f'name: "{self.gz_model_name}", '
                f'position: {{x: {self.current_x:.3f}, '
                f'y: {self.current_y:.3f}, z: {self.target_z:.3f}}}, '
                f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            )
            try:
                subprocess.run(
                    [
                        'gz', 'service', '-s',
                        f'/world/{self.gz_world}/set_pose',
                        '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '200',
                        '--req', req,
                    ],
                    check=False,
                    capture_output=True,
                    timeout=0.3,
                )
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
        finally:
            self.gz_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = TargetSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
