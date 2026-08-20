#!/usr/bin/env python3
"""
Mission Editor Node - interactive waypoint mission builder.

User clicks points in RViz using the 'Publish Point' tool.
Each click adds a numbered waypoint. When the user triggers /mission_start,
the drone visits each waypoint in order, then returns home and lands.

Topics from user:
  /clicked_point   (RViz 'Publish Point' tool)  -> add waypoint
  /mission_start   (Bool true)                  -> begin mission
  /mission_clear   (Bool true)                  -> clear waypoint list
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import time
from ...common.interfaces import declare_interface_params


class MP:
    """Mission phase constants."""
    COLLECTING = 'COLLECTING'
    WAITING_FOR_TAKEOFF = 'WAITING_FOR_TAKEOFF'
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    NEXT_WP = 'NEXT_WP'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'


class MissionEditor(Node):
    def __init__(self):
        super().__init__('mission_editor')
        declare_interface_params(self)

        # ========== Parameters ==========
        self.default_altitude = 2.0
        self.goal_send_timeout = 3.0
        self.settle_time_between_wps = 1.5  # seconds at each waypoint

        # ========== State ==========
        self.waypoints = []         # list of (x, y, z)
        self.mission_phase = MP.COLLECTING
        self.current_index = 0
        self.mission_started = False
        self.current_nav_state = 'UNKNOWN'
        self.send_attempt_time = None
        self.settle_start_time = None

        # ========== Subscribers ==========
        # RViz 'Publish Point' tool sends PointStamped to /clicked_point
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_start', self.start_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_clear', self.clear_callback, 10
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/mission_waypoints', 10
        )

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.viz_timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info('=' * 55)
        self.get_logger().info('MISSION EDITOR active')
        self.get_logger().info('-' * 55)
        self.get_logger().info('In RViz:')
        self.get_logger().info('  1. Add MarkerArray on /mission_waypoints')
        self.get_logger().info('  2. Click "Publish Point" tool, click on map')
        self.get_logger().info('  3. Repeat for each waypoint')
        self.get_logger().info('-' * 55)
        self.get_logger().info('From a terminal:')
        self.get_logger().info(
            '  Start:  ros2 topic pub --once /mission_start '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info(
            '  Clear:  ros2 topic pub --once /mission_clear '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 55)

    # =====================================================
    # Subscribers
    # =====================================================
    def clicked_callback(self, msg):
        if self.mission_phase != MP.COLLECTING:
            self.get_logger().warn(
                'Mission already started - ignoring click. '
                'Use /mission_clear to reset.'
            )
            return
        # Use RViz click x,y; override z with default altitude
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = self.default_altitude
        self.waypoints.append((x, y, z))
        self.get_logger().info(
            f'Added WP{len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})'
        )

    def start_callback(self, msg):
        if not msg.data:
            return
        if self.mission_started:
            self.get_logger().warn('Mission already running.')
            return
        if not self.waypoints:
            self.get_logger().error(
                'No waypoints! Click points in RViz first.'
            )
            return
        self.mission_started = True
        self.get_logger().info(
            f'Mission starting with {len(self.waypoints)} waypoints'
        )
        self.mission_phase = MP.WAITING_FOR_TAKEOFF

    def clear_callback(self, msg):
        if not msg.data:
            return
        self.waypoints = []
        self.current_index = 0
        self.mission_started = False
        self.mission_phase = MP.COLLECTING
        self.get_logger().info('Waypoint list cleared.')

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    # =====================================================
    # Mission tick
    # =====================================================
    def tick(self):
        p = self.mission_phase

        if p == MP.COLLECTING:
            return

        elif p == MP.WAITING_FOR_TAKEOFF:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone hovering. Starting tour.')
                self.set_phase(MP.SEND_WP)
            else:
                self.get_logger().info(
                    f'Waiting for takeoff... state: {self.current_nav_state}',
                    throttle_duration_sec=2.0
                )

        elif p == MP.SEND_WP:
            if self.current_index >= len(self.waypoints):
                self.get_logger().info('All waypoints visited.')
                self.set_phase(MP.SEND_RTH)
                return
            x, y, z = self.waypoints[self.current_index]
            self.send_goal(x, y, z)
            self.send_attempt_time = time.time()
            self.set_phase(MP.WAIT_NAV)

        elif p == MP.WAIT_NAV:
            if self.current_nav_state == 'NAVIGATING':
                self.set_phase(MP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.get_logger().warn('Goal not accepted - resending')
                self.set_phase(MP.SEND_WP)

        elif p == MP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info(
                    f'Reached WP {self.current_index + 1}/'
                    f'{len(self.waypoints)}'
                )
                self.settle_start_time = time.time()
                self.set_phase(MP.NEXT_WP)

        elif p == MP.NEXT_WP:
            if time.time() - self.settle_start_time >= self.settle_time_between_wps:
                self.current_index += 1
                self.set_phase(MP.SEND_WP)

        elif p == MP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info('RTH triggered.')
            self.set_phase(MP.WAIT_LANDED)

        elif p == MP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.get_logger().info('Mission complete.')
                self.set_phase(MP.DONE)

        elif p == MP.DONE:
            self.get_logger().info(
                'Done. Send /mission_clear to start a new mission.',
                throttle_duration_sec=5.0
            )

    # =====================================================
    # Helpers
    # =====================================================
    def set_phase(self, new_phase):
        if new_phase != self.mission_phase:
            self.get_logger().info(
                f'EDITOR: {self.mission_phase} -> {new_phase}'
            )
            self.mission_phase = new_phase

    def send_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'>>> Going to WP {self.current_index + 1}: ({x:.2f}, {y:.2f}, {z:.2f})'
        )

    # =====================================================
    # Visualization
    # =====================================================
    def publish_markers(self):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (x, y, z) in enumerate(self.waypoints):
            is_current = (
                self.mission_phase in (MP.SEND_WP, MP.WAIT_NAV, MP.WAIT_HOVER, MP.NEXT_WP)
                and i == self.current_index
            )
            is_done = (
                self.mission_started and i < self.current_index
            )

            # Sphere marker for the waypoint
            sphere = Marker()
            sphere.header.frame_id = self.frame('map')
            sphere.header.stamp = now
            sphere.ns = 'mission_wp'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=0.5, y=0.5, z=0.5)
            if is_current:
                sphere.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # orange
                sphere.scale = Vector3(x=0.8, y=0.8, z=0.8)
            elif is_done:
                sphere.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)  # gray
            else:
                sphere.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.9)  # blue
            array.markers.append(sphere)

            # Number label
            label = Marker()
            label.header.frame_id = self.frame('map')
            label.header.stamp = now
            label.ns = 'mission_wp_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.7
            label.scale.z = 0.4
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = str(i + 1)
            array.markers.append(label)

        # Path line connecting waypoints
        if len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = self.frame('map')
            line.header.stamp = now
            line.ns = 'mission_path'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.5)
            for (x, y, z) in self.waypoints:
                line.points.append(Point(x=x, y=y, z=z))
            array.markers.append(line)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
