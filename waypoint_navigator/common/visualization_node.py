#!/usr/bin/env python3
"""
Visualization Node - publishes RViz markers for navigation state.

Markers:
  - Home position (blue cylinder)
  - Current goal (yellow sphere)
  - Drone trail (green line strip, last N positions)
  - State text (white floating text above drone)

This is a passive observer - it doesn't control anything.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from collections import deque
from .interfaces import declare_interface_params


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== Parameters ==========
        self.trail_max_points = 300   # ~30 seconds at 10Hz
        self.trail_min_distance = 0.1  # m - don't add if too close to last

        # ========== State ==========
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_set = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.has_goal = False

        self.current_nav_state = 'UNKNOWN'

        # Trail history
        self.trail = deque(maxlen=self.trail_max_points)

        # ========== Subscribers ==========
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )

        # ========== Publisher ==========
        self.marker_pub = self.create_publisher(
            MarkerArray, '/viz_markers', 10
        )

        # ========== Timer ==========
        # Publish markers at 5 Hz
        self.timer = self.create_timer(0.2, self.publish_markers)

        self.get_logger().info('Visualization Node started.')
        self.get_logger().info(
            'In RViz, add MarkerArray display with topic: /viz_markers'
        )

    # =====================================================
    # Subscribers
    # =====================================================
    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

        # First position = home
        if not self.home_set:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.current_z
            self.home_set = True
            self.get_logger().info(
                f'Home recorded at ({self.home_x:.2f}, {self.home_y:.2f}, '
                f'{self.home_z:.2f})'
            )

        # Add to trail (with minimum distance filter to avoid clutter)
        if self.trail:
            last = self.trail[-1]
            dx = self.current_x - last[0]
            dy = self.current_y - last[1]
            dz = self.current_z - last[2]
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < self.trail_min_distance ** 2:
                return
        self.trail.append((self.current_x, self.current_y, self.current_z))

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z if msg.pose.position.z > 0 else 2.0
        self.has_goal = True

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data  # full string with battery

    # =====================================================
    # Markers
    # =====================================================
    def publish_markers(self):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ----- Home marker (blue cylinder) -----
        if self.home_set:
            home = Marker()
            home.header.frame_id = self.frame('map')
            home.header.stamp = now
            home.ns = 'home'
            home.id = 0
            home.type = Marker.CYLINDER
            home.action = Marker.ADD
            home.pose.position.x = self.home_x
            home.pose.position.y = self.home_y
            home.pose.position.z = self.home_z + 0.25
            home.pose.orientation.w = 1.0
            home.scale = Vector3(x=0.6, y=0.6, z=0.5)
            home.color = ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.85)
            array.markers.append(home)

            # Home label
            home_text = Marker()
            home_text.header.frame_id = self.frame('map')
            home_text.header.stamp = now
            home_text.ns = 'home_label'
            home_text.id = 1
            home_text.type = Marker.TEXT_VIEW_FACING
            home_text.action = Marker.ADD
            home_text.pose.position.x = self.home_x
            home_text.pose.position.y = self.home_y
            home_text.pose.position.z = self.home_z + 1.0
            home_text.scale.z = 0.5
            home_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            home_text.text = 'HOME'
            array.markers.append(home_text)

        # ----- Goal marker (yellow sphere) -----
        if self.has_goal:
            goal = Marker()
            goal.header.frame_id = self.frame('map')
            goal.header.stamp = now
            goal.ns = 'goal'
            goal.id = 0
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.position.z = self.goal_z
            goal.pose.orientation.w = 1.0
            goal.scale = Vector3(x=0.6, y=0.6, z=0.6)
            goal.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)
            array.markers.append(goal)

            # Goal label
            goal_text = Marker()
            goal_text.header.frame_id = self.frame('map')
            goal_text.header.stamp = now
            goal_text.ns = 'goal_label'
            goal_text.id = 1
            goal_text.type = Marker.TEXT_VIEW_FACING
            goal_text.action = Marker.ADD
            goal_text.pose.position.x = self.goal_x
            goal_text.pose.position.y = self.goal_y
            goal_text.pose.position.z = self.goal_z + 0.8
            goal_text.scale.z = 0.4
            goal_text.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)
            goal_text.text = 'GOAL'
            array.markers.append(goal_text)

            # Line from drone to goal (helps see the target)
            if self.position_received:
                line = Marker()
                line.header.frame_id = self.frame('map')
                line.header.stamp = now
                line.ns = 'goal_line'
                line.id = 0
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.05
                line.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.4)
                line.points.append(Point(
                    x=self.current_x, y=self.current_y, z=self.current_z
                ))
                line.points.append(Point(
                    x=self.goal_x, y=self.goal_y, z=self.goal_z
                ))
                array.markers.append(line)

        # ----- Trail (green line strip) -----
        if len(self.trail) >= 2:
            trail = Marker()
            trail.header.frame_id = self.frame('map')
            trail.header.stamp = now
            trail.ns = 'trail'
            trail.id = 0
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = 0.08
            trail.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)
            for (px, py, pz) in self.trail:
                trail.points.append(Point(x=px, y=py, z=pz))
            array.markers.append(trail)

        # ----- State text (white, above drone) -----
        if self.position_received:
            state_text = Marker()
            state_text.header.frame_id = self.frame('map')
            state_text.header.stamp = now
            state_text.ns = 'state_text'
            state_text.id = 0
            state_text.type = Marker.TEXT_VIEW_FACING
            state_text.action = Marker.ADD
            state_text.pose.position.x = self.current_x
            state_text.pose.position.y = self.current_y
            state_text.pose.position.z = self.current_z + 1.5
            state_text.scale.z = 0.5
            state_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            state_text.text = self.current_nav_state
            array.markers.append(state_text)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
