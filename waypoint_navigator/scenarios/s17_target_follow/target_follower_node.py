#!/usr/bin/env python3
"""
Target Follower — Phase 22

Drives the drone to hover above a moving target. Subscribes to
/target_position (PointStamped) and publishes /goal_pose (PoseStamped)
at follow_z metres altitude, so the state machine treats every fresh
target position as a new navigation goal.

This is the canonical "follow-me / follow-the-animal" pattern:
  - target moves -> drone follows
  - target stops -> drone hovers above

Visualisation:
  - /follower_marker  : red line from drone to target + arrow
  - /follower_status  : text status (distance / dx / dy / dz)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from ...common.interfaces import declare_interface_params


class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        declare_interface_params(self)

        # ========== Parameters ==========
        # Altitude at which drone hovers above the target.
        self.declare_parameter('follow_z', 3.0)
        # Horizontal offset (drone hovers offset behind target). For
        # straight-above tracking use (0, 0).
        self.declare_parameter('follow_offset_x', 0.0)
        self.declare_parameter('follow_offset_y', 0.0)
        # Send a fresh /goal_pose at this rate (Hz). Higher = drone reacts
        # faster but spams the state machine; 4 Hz is a good balance.
        self.declare_parameter('goal_rate_hz', 4.0)
        # Wait until drone is hovering before publishing goals.
        self.declare_parameter('takeoff_threshold', 2.0)

        self.follow_z          = float(self.get_parameter('follow_z').value)
        self.follow_offset_x   = float(self.get_parameter('follow_offset_x').value)
        self.follow_offset_y   = float(self.get_parameter('follow_offset_y').value)
        self.goal_rate_hz      = float(self.get_parameter('goal_rate_hz').value)
        self.takeoff_threshold = float(self.get_parameter('takeoff_threshold').value)

        # ========== State ==========
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_pos_received = False

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_received = False

        # WAIT_TAKEOFF -> FOLLOW
        self.phase = 'WAIT_TAKEOFF'

        # ========== Subscribers ==========
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos,
        )
        self.create_subscription(
            PointStamped, '/target_position', self.target_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/follower_marker', 10
        )
        self.status_pub = self.create_publisher(
            String, '/follower_status', 10
        )

        # ========== Timers ==========
        period = 1.0 / max(self.goal_rate_hz, 1.0)
        self.create_timer(period, self.publish_goal)
        self.create_timer(0.2, self.publish_marker)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'Target Follower started — follow_z={self.follow_z:.2f} m, '
            f'offset=({self.follow_offset_x:.2f}, {self.follow_offset_y:.2f}), '
            f'rate={self.goal_rate_hz:.1f} Hz'
        )
        self.get_logger().info('Waiting for drone to reach hover altitude.')

    # =====================================================
    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.drone_x = msg.y
        self.drone_y = msg.x
        self.drone_z = -msg.z
        self.drone_pos_received = True

    def target_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.target_z = msg.point.z
        self.target_received = True

    # =====================================================
    def publish_goal(self):
        if not self.target_received or not self.drone_pos_received:
            return

        if self.phase == 'WAIT_TAKEOFF':
            if self.drone_z >= self.takeoff_threshold:
                self.phase = 'FOLLOW'
                self.get_logger().info(
                    f'Drone at z={self.drone_z:.2f} m -> FOLLOW phase started.'
                )
            else:
                return

        gx = self.target_x + self.follow_offset_x
        gy = self.target_y + self.follow_offset_y
        gz = self.follow_z

        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    # =====================================================
    def publish_marker(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Line from drone to target
        line = Marker()
        line.header.frame_id = self.frame('map')
        line.header.stamp = now
        line.ns = 'follow_link'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.r = 1.0
        line.color.g = 0.2
        line.color.b = 0.2
        line.color.a = 0.9
        p1 = Point()
        p1.x = self.drone_x
        p1.y = self.drone_y
        p1.z = self.drone_z
        line.points.append(p1)
        p2 = Point()
        p2.x = self.target_x
        p2.y = self.target_y
        p2.z = self.target_z
        line.points.append(p2)
        ma.markers.append(line)

        # Goal sphere (the point above the target where drone wants to be)
        goal = Marker()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = now
        goal.ns = 'follow_goal'
        goal.id = 0
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = self.target_x + self.follow_offset_x
        goal.pose.position.y = self.target_y + self.follow_offset_y
        goal.pose.position.z = self.follow_z
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.5
        goal.color.r = 0.1
        goal.color.g = 1.0
        goal.color.b = 0.3
        goal.color.a = 0.7
        ma.markers.append(goal)

        self.marker_pub.publish(ma)

    def publish_status(self):
        dx = self.target_x - self.drone_x
        dy = self.target_y - self.drone_y
        dz = self.follow_z - self.drone_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        msg = String()
        msg.data = (
            f'phase={self.phase} | drone=({self.drone_x:+.2f},'
            f'{self.drone_y:+.2f},{self.drone_z:+.2f}) | '
            f'target=({self.target_x:+.2f},{self.target_y:+.2f},'
            f'{self.target_z:+.2f}) | dist={dist:.2f}m'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C already shuts the context down via rclpy's signal
        # handler, so an unguarded shutdown() raises on ROS 2 Jazzy.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
