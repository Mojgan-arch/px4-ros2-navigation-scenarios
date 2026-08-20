#!/usr/bin/env python3
"""
Coverage Path Planner (Stage 14).

Generates a boustrophedon (lawnmower) pattern over a user-defined
rectangular area. Drone covers every point with configurable spacing.

Workflow:
  1. Click corner 1 (PointStamped) in RViz
  2. Click corner 2 - pattern appears in YELLOW
  3. Confirm or reject
  4. On confirm: drone sweeps the area, then RTH+land

Use cases: aerial mapping, photogrammetry, agricultural spraying,
search-pattern flights, systematic area coverage.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import time
from ...common.interfaces import declare_interface_params


class CP:
    """Coverage planner phases."""
    WAITING_FOR_HOVER = 'WAITING_FOR_HOVER'
    WAITING_CORNER_1 = 'WAITING_CORNER_1'
    WAITING_CORNER_2 = 'WAITING_CORNER_2'
    WAIT_CONFIRM = 'WAIT_CONFIRM'
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    NEXT_WP = 'NEXT_WP'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        declare_interface_params(self)

        # ========== Parameters ==========
        # Match the state machine's default_altitude (~2.5 m) so the drone
        # doesn't have to climb between every sweep waypoint.
        self.altitude = 2.5          # m
        self.line_spacing = 2.0      # m between sweep lines
        self.sweep_axis = 'x'        # 'x' or 'y' - which axis to sweep along
        self.goal_send_timeout = 5.0

        # ========== State ==========
        self.current_nav_state = 'UNKNOWN'

        self.corner1 = None    # (x, y)
        self.corner2 = None    # (x, y)
        self.waypoints = []    # list of (x, y, z)
        self.path_index = 0

        self.phase = CP.WAITING_FOR_HOVER
        self.send_attempt_time = None

        # ========== Subscribers ==========
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, 10
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.create_subscription(
            Bool, '/confirm_path', self.confirm_callback, 10
        )
        self.create_subscription(
            Bool, '/reject_path', self.reject_callback, 10
        )
        self.create_subscription(
            Bool, '/clear_coverage', self.clear_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage_pattern', 10
        )

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('=' * 60)
        self.get_logger().info('COVERAGE PATH PLANNER (lawnmower pattern)')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Altitude:     {self.altitude}m')
        self.get_logger().info(f'Line spacing: {self.line_spacing}m')
        self.get_logger().info(f'Sweep axis:   {self.sweep_axis}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Steps:')
        self.get_logger().info('  1. Click first corner with Publish Point')
        self.get_logger().info('  2. Click opposite corner')
        self.get_logger().info('  3. Yellow pattern appears - review it')
        self.get_logger().info('  4. Confirm to execute, or reject to retry')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Confirm: ros2 topic pub --once /confirm_path std_msgs/Bool "data: true"')
        self.get_logger().info('Reject:  ros2 topic pub --once /reject_path  std_msgs/Bool "data: true"')
        self.get_logger().info('Clear:   ros2 topic pub --once /clear_coverage std_msgs/Bool "data: true"')
        self.get_logger().info('=' * 60)

    # =====================================================
    # Subscribers
    # =====================================================
    def clicked_callback(self, msg):
        x = float(msg.point.x)
        y = float(msg.point.y)
        if self.phase == CP.WAITING_CORNER_1:
            self.corner1 = (x, y)
            self.get_logger().info(f'Corner 1: ({x:.2f}, {y:.2f})')
            self.set_phase(CP.WAITING_CORNER_2)
        elif self.phase == CP.WAITING_CORNER_2:
            self.corner2 = (x, y)
            self.get_logger().info(f'Corner 2: ({x:.2f}, {y:.2f})')
            self.generate_pattern()
            self.set_phase(CP.WAIT_CONFIRM)
        else:
            self.get_logger().warn(
                f'Cannot accept click now (phase={self.phase}). '
                'Send /clear_coverage to reset.'
            )

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    def confirm_callback(self, msg):
        if not msg.data:
            return
        if self.phase != CP.WAIT_CONFIRM:
            return
        self.get_logger().info('>>> COVERAGE CONFIRMED - executing <<<')
        self.path_index = 0
        self.set_phase(CP.SEND_WP)

    def reject_callback(self, msg):
        if not msg.data:
            return
        if self.phase != CP.WAIT_CONFIRM:
            return
        self.get_logger().info('Coverage REJECTED - resetting')
        self.reset_to_waiting()

    def clear_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Cleared.')
        self.reset_to_waiting()

    def reset_to_waiting(self):
        self.corner1 = None
        self.corner2 = None
        self.waypoints = []
        self.path_index = 0
        self.set_phase(CP.WAITING_CORNER_1)

    # =====================================================
    # Pattern generation
    # =====================================================
    def generate_pattern(self):
        if self.corner1 is None or self.corner2 is None:
            return

        x1, y1 = self.corner1
        x2, y2 = self.corner2
        # Normalize: get min/max so we can sweep
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)

        pts = []

        if self.sweep_axis == 'x':
            # Sweep parallel to x, advance in y
            y = y_min
            direction = 1   # +1 = left to right, -1 = right to left
            while y <= y_max + 0.01:
                if direction > 0:
                    pts.append((x_min, y, self.altitude))
                    pts.append((x_max, y, self.altitude))
                else:
                    pts.append((x_max, y, self.altitude))
                    pts.append((x_min, y, self.altitude))
                y += self.line_spacing
                direction *= -1
        else:
            # Sweep parallel to y, advance in x
            x = x_min
            direction = 1
            while x <= x_max + 0.01:
                if direction > 0:
                    pts.append((x, y_min, self.altitude))
                    pts.append((x, y_max, self.altitude))
                else:
                    pts.append((x, y_max, self.altitude))
                    pts.append((x, y_min, self.altitude))
                x += self.line_spacing
                direction *= -1

        self.waypoints = pts

        # Compute area + length
        area = abs(x_max - x_min) * abs(y_max - y_min)
        length = 0.0
        for i in range(len(pts) - 1):
            ax, ay, _ = pts[i]
            bx, by, _ = pts[i + 1]
            length += ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5

        self.get_logger().info('=' * 50)
        self.get_logger().info('COVERAGE PATTERN READY')
        self.get_logger().info(f'  Area:      {area:.1f} m^2')
        self.get_logger().info(f'  Path:      {length:.1f} m')
        self.get_logger().info(f'  Waypoints: {len(pts)}')
        self.get_logger().info(f'  Sweep lines: {len(pts) // 2}')
        self.get_logger().info('=' * 50)

    # =====================================================
    # Phase tick
    # =====================================================
    def tick(self):
        if self.phase == CP.WAITING_FOR_HOVER:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone hovering. Ready.')
                self.set_phase(CP.WAITING_CORNER_1)

        elif self.phase == CP.SEND_WP:
            if self.path_index >= len(self.waypoints):
                self.get_logger().info('Coverage complete.')
                self.set_phase(CP.SEND_RTH)
                return
            x, y, z = self.waypoints[self.path_index]
            self.send_goal(x, y, z)
            self.send_attempt_time = time.time()
            self.set_phase(CP.WAIT_NAV)

        elif self.phase == CP.WAIT_NAV:
            # Accept NAVIGATING or HOVER (drone might fly through fast)
            if self.current_nav_state in ('NAVIGATING', 'HOVER'):
                self.set_phase(CP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.set_phase(CP.SEND_WP)

        elif self.phase == CP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                self.path_index += 1
                self.set_phase(CP.NEXT_WP)
            elif (self.send_attempt_time is not None
                  and time.time() - self.send_attempt_time > 30.0):
                # Stuck — skip this waypoint and try the next one
                self.get_logger().warn(
                    f'WP {self.path_index + 1} timeout — skipping'
                )
                self.path_index += 1
                self.set_phase(CP.NEXT_WP)

        elif self.phase == CP.NEXT_WP:
            self.set_phase(CP.SEND_WP)

        elif self.phase == CP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info('RTH triggered.')
            self.set_phase(CP.WAIT_LANDED)

        elif self.phase == CP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.get_logger().info('Mission complete.')
                self.set_phase(CP.DONE)

        elif self.phase == CP.DONE:
            self.get_logger().info(
                'Send /clear_coverage to start a new coverage.',
                throttle_duration_sec=5.0
            )

    def set_phase(self, new_phase):
        if new_phase != self.phase:
            self.get_logger().info(f'COVERAGE: {self.phase} -> {new_phase}')
            self.phase = new_phase

    # =====================================================
    # Helpers
    # =====================================================
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
            f'>>> WP {self.path_index + 1}/{len(self.waypoints)}: '
            f'({x:.2f}, {y:.2f}, {z:.2f})'
        )

    # =====================================================
    # Visualization
    # =====================================================
    def publish_visualization(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Show corners if any
        for i, corner in enumerate([self.corner1, self.corner2]):
            if corner is None:
                continue
            cm = Marker()
            cm.header.frame_id = self.frame('map')
            cm.header.stamp = now
            cm.ns = 'corners'
            cm.id = i
            cm.type = Marker.SPHERE
            cm.action = Marker.ADD
            cm.pose.position.x = corner[0]
            cm.pose.position.y = corner[1]
            cm.pose.position.z = self.altitude
            cm.pose.orientation.w = 1.0
            cm.scale = Vector3(x=0.6, y=0.6, z=0.6)
            cm.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)  # magenta
            arr.markers.append(cm)

            txt = Marker()
            txt.header.frame_id = self.frame('map')
            txt.header.stamp = now
            txt.ns = 'corner_labels'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = corner[0]
            txt.pose.position.y = corner[1]
            txt.pose.position.z = self.altitude + 0.7
            txt.scale.z = 0.4
            txt.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            txt.text = f'Corner {i + 1}'
            arr.markers.append(txt)

        # Show rectangle boundary if both corners set
        if self.corner1 and self.corner2:
            x1, y1 = self.corner1
            x2, y2 = self.corner2
            box = Marker()
            box.header.frame_id = self.frame('map')
            box.header.stamp = now
            box.ns = 'area_box'
            box.id = 0
            box.type = Marker.LINE_STRIP
            box.action = Marker.ADD
            box.scale.x = 0.05
            box.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5)
            corners = [
                (x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)
            ]
            for (cx, cy) in corners:
                box.points.append(
                    Point(x=cx, y=cy, z=self.altitude)
                )
            arr.markers.append(box)

        # Show pattern (color by phase)
        if self.waypoints:
            if self.phase == CP.WAIT_CONFIRM:
                color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # YELLOW
                status = 'AWAITING CONFIRMATION'
            elif self.phase in (CP.SEND_WP, CP.WAIT_NAV, CP.WAIT_HOVER, CP.NEXT_WP):
                color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)  # GREEN
                status = 'EXECUTING'
            else:
                color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
                status = ''

            # Line
            line = Marker()
            line.header.frame_id = self.frame('map')
            line.header.stamp = now
            line.ns = 'cov_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = color
            for (x, y, z) in self.waypoints:
                line.points.append(Point(x=x, y=y, z=z))
            arr.markers.append(line)

            # Waypoint spheres
            for i, (x, y, z) in enumerate(self.waypoints):
                sp = Marker()
                sp.header.frame_id = self.frame('map')
                sp.header.stamp = now
                sp.ns = 'cov_pts'
                sp.id = i
                sp.type = Marker.SPHERE
                sp.action = Marker.ADD
                sp.pose.position.x = x
                sp.pose.position.y = y
                sp.pose.position.z = z
                sp.pose.orientation.w = 1.0
                sp.scale = Vector3(x=0.2, y=0.2, z=0.2)
                if (i == self.path_index and
                        self.phase in (CP.SEND_WP, CP.WAIT_NAV, CP.WAIT_HOVER)):
                    sp.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                    sp.scale = Vector3(x=0.4, y=0.4, z=0.4)
                elif i < self.path_index:
                    sp.color = ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.6)
                else:
                    sp.color = color
                arr.markers.append(sp)

            # Status label
            if status:
                txt = Marker()
                txt.header.frame_id = self.frame('map')
                txt.header.stamp = now
                txt.ns = 'cov_status'
                txt.id = 0
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                # Place text in the center of the area
                cx = (self.corner1[0] + self.corner2[0]) / 2
                cy = (self.corner1[1] + self.corner2[1]) / 2
                txt.pose.position.x = cx
                txt.pose.position.y = cy
                txt.pose.position.z = self.altitude + 1.5
                txt.scale.z = 0.6
                txt.color = color
                txt.text = status
                arr.markers.append(txt)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
