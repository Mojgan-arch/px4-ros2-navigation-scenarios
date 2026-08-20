#!/usr/bin/env python3
"""
Waypoint Navigator - Phase 1 (basic)

Drives the drone through a hard-coded list of (x, y, z) waypoints in ENU
metres. The C++ offboard_control_node owns the IDLE -> ARMING -> TAKEOFF ->
HOVER sequence; this node only sends cmd_vel once HOVER altitude is reached,
then sends arm=false at the end so the drone lands and disarms cleanly.

Matches the topic/coordinate conventions used by every other phase node
in this workspace (state_machine_node, path_planner_node, ...).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from ...common.interfaces import declare_interface_params


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        declare_interface_params(self)

        # PX4-compatible QoS (best_effort + transient_local)
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # The C++ offboard_control_node lifts the drone to this altitude
        # before it accepts cmd_vel. Match it here.
        self.takeoff_altitude = 2.5
        self.takeoff_threshold = self.takeoff_altitude - 0.5  # 2.0 m

        # 4 waypoints forming a closed loop, ENU (x, y, z) in metres.
        # Small 2x2 square — stays inside the walled test area.
        # 1) forward, 2) right, 3) back, 4) home.
        self.waypoints = [
            (2.0, 0.0, self.takeoff_altitude),
            (2.0, 2.0, self.takeoff_altitude),
            (0.0, 2.0, self.takeoff_altitude),
            (0.0, 0.0, self.takeoff_altitude),
        ]
        self.current_wp_index = 0
        self.tolerance = 0.5     # m — "reached"
        self.max_speed = 1.0     # m/s

        # WAIT_TAKEOFF -> NAV -> LAND -> DONE
        self.phase = 'WAIT_TAKEOFF'

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        # Position from PX4 (NED -> ENU in callback)
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos,
        )

        # Velocity + arm to the C++ offboard_control_node
        self.velocity_pub = self.create_publisher(
            Twist, self.topic('cmd_vel'), 10
        )
        self.arm_pub = self.create_publisher(
            Bool, self.topic('arm'), 10
        )

        # Human-readable status
        self.status_pub = self.create_publisher(String, '/waypoint_status', 10)

        # RViz visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10
        )

        # Loops
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.5, self.publish_markers)

        self.get_logger().info(
            f'Waypoint Navigator started. {len(self.waypoints)} waypoints, '
            f'tolerance={self.tolerance} m, max_speed={self.max_speed} m/s.'
        )
        self.get_logger().info(
            'Waiting for arm + takeoff '
            '(publish Bool(true) to /px4_offboard_sim/offboard_control/arm).'
        )

    # -----------------------------------------------------
    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def publish_velocity(self, vx, vy, vz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0
        self.velocity_pub.publish(cmd)

    def publish_markers(self):
        """Draw waypoints + path + home + current target in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ---- 1) Sphere at each waypoint (colored by status) ----
        for i, (x, y, z) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.frame('map')
            m.header.stamp = now
            m.ns = 'wp_spheres'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = 0.6
            m.scale.y = 0.6
            m.scale.z = 0.6
            # gray=not yet, orange=current target, green=reached
            if i < self.current_wp_index:
                m.color.r, m.color.g, m.color.b = 0.2, 0.9, 0.2  # green
            elif i == self.current_wp_index and self.phase == 'NAV':
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0  # orange
            else:
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5  # gray
            m.color.a = 0.9
            ma.markers.append(m)

            # vertical pillar from ground to waypoint — shows the projection
            pillar = Marker()
            pillar.header.frame_id = self.frame('map')
            pillar.header.stamp = now
            pillar.ns = 'wp_pillars'
            pillar.id = i
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.pose.position.x = x
            pillar.pose.position.y = y
            pillar.pose.position.z = z / 2.0
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = 0.05
            pillar.scale.y = 0.05
            pillar.scale.z = z
            pillar.color.r, pillar.color.g, pillar.color.b = 0.8, 0.8, 0.0
            pillar.color.a = 0.5
            ma.markers.append(pillar)

            # disk on the ground — clear "footprint"
            disk = Marker()
            disk.header.frame_id = self.frame('map')
            disk.header.stamp = now
            disk.ns = 'wp_ground'
            disk.id = i
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position.x = x
            disk.pose.position.y = y
            disk.pose.position.z = 0.05
            disk.pose.orientation.w = 1.0
            disk.scale.x = 0.8
            disk.scale.y = 0.8
            disk.scale.z = 0.1
            disk.color.r, disk.color.g, disk.color.b = 1.0, 1.0, 0.0
            disk.color.a = 0.8
            ma.markers.append(disk)

            # text label "WP1", "WP2", ...
            text = Marker()
            text.header.frame_id = self.frame('map')
            text.header.stamp = now
            text.ns = 'wp_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 0.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.5
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'WP{i + 1}'
            ma.markers.append(text)

        # ---- 2) Line strip connecting all waypoints ----
        line = Marker()
        line.header.frame_id = self.frame('map')
        line.header.stamp = now
        line.ns = 'wp_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.08  # line width
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 0.7
        # start from home (0,0) at altitude so the line shows the takeoff leg too
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = self.takeoff_altitude
        line.points.append(p0)
        for (x, y, z) in self.waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            line.points.append(p)
        ma.markers.append(line)

        # ---- 3) Home marker (red X-like cross) ----
        home = Marker()
        home.header.frame_id = self.frame('map')
        home.header.stamp = now
        home.ns = 'home'
        home.id = 0
        home.type = Marker.CUBE
        home.action = Marker.ADD
        home.pose.position.x = 0.0
        home.pose.position.y = 0.0
        home.pose.position.z = 0.1
        home.pose.orientation.w = 1.0
        home.scale.x = 1.0
        home.scale.y = 1.0
        home.scale.z = 0.05
        home.color.r = 1.0
        home.color.g = 0.0
        home.color.b = 0.0
        home.color.a = 0.7
        ma.markers.append(home)

        home_label = Marker()
        home_label.header.frame_id = self.frame('map')
        home_label.header.stamp = now
        home_label.ns = 'home_label'
        home_label.id = 0
        home_label.type = Marker.TEXT_VIEW_FACING
        home_label.action = Marker.ADD
        home_label.pose.position.x = 0.0
        home_label.pose.position.y = 0.0
        home_label.pose.position.z = 0.6
        home_label.pose.orientation.w = 1.0
        home_label.scale.z = 0.5
        home_label.color.r = 1.0
        home_label.color.g = 1.0
        home_label.color.b = 1.0
        home_label.color.a = 1.0
        home_label.text = 'HOME'
        ma.markers.append(home_label)

        # ---- 4) Arrow from drone to current target ----
        if self.phase == 'NAV' and self.current_wp_index < len(self.waypoints):
            tx, ty, tz = self.waypoints[self.current_wp_index]
            arrow = Marker()
            arrow.header.frame_id = self.frame('map')
            arrow.header.stamp = now
            arrow.ns = 'target_arrow'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.1  # shaft diameter
            arrow.scale.y = 0.2  # head diameter
            arrow.scale.z = 0.3  # head length
            arrow.color.r = 1.0
            arrow.color.g = 0.5
            arrow.color.b = 0.0
            arrow.color.a = 0.9
            p_start = Point()
            p_start.x = self.current_x
            p_start.y = self.current_y
            p_start.z = self.current_z
            arrow.points.append(p_start)
            p_end = Point()
            p_end.x = tx
            p_end.y = ty
            p_end.z = tz
            arrow.points.append(p_end)
            ma.markers.append(arrow)

        self.marker_pub.publish(ma)

    def publish_status(self):
        if self.phase == 'NAV' and self.current_wp_index < len(self.waypoints):
            tx, ty, tz = self.waypoints[self.current_wp_index]
            dist = math.sqrt(
                (tx - self.current_x) ** 2
                + (ty - self.current_y) ** 2
                + (tz - self.current_z) ** 2
            )
            extra = (
                f' | wp={self.current_wp_index + 1}/{len(self.waypoints)} '
                f'target=({tx:.1f},{ty:.1f},{tz:.1f}) dist={dist:.2f}'
            )
        else:
            extra = ''
        msg = String()
        msg.data = (
            f'phase={self.phase} | pos=({self.current_x:.2f}, '
            f'{self.current_y:.2f}, {self.current_z:.2f}){extra}'
        )
        self.status_pub.publish(msg)

    # -----------------------------------------------------
    def control_loop(self):
        if not self.position_received:
            return

        if self.phase == 'WAIT_TAKEOFF':
            if self.current_z >= self.takeoff_threshold:
                self.phase = 'NAV'
                self.get_logger().info(
                    f'Takeoff complete (z={self.current_z:.2f} m). '
                    f'Starting waypoint navigation.'
                )
            return

        if self.phase == 'NAV':
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info(
                    'All waypoints reached. Sending land command.'
                )
                self.publish_velocity(0.0, 0.0, 0.0)
                msg = Bool()
                msg.data = False
                self.arm_pub.publish(msg)
                self.phase = 'LAND'
                return

            tx, ty, tz = self.waypoints[self.current_wp_index]
            dx = tx - self.current_x
            dy = ty - self.current_y
            dz = tz - self.current_z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance < self.tolerance:
                self.get_logger().info(
                    f'Reached waypoint '
                    f'{self.current_wp_index + 1}/{len(self.waypoints)}: '
                    f'({tx:.1f}, {ty:.1f}, {tz:.1f})'
                )
                self.current_wp_index += 1
                self.publish_velocity(0.0, 0.0, 0.0)
                return

            vx = (dx / distance) * self.max_speed
            vy = (dy / distance) * self.max_speed
            vz = (dz / distance) * self.max_speed

            if distance < 2.0:
                scale = max(distance / 2.0, 0.2)
                vx *= scale
                vy *= scale
                vz *= scale

            self.publish_velocity(vx, vy, vz)
            self.get_logger().info(
                f'WP {self.current_wp_index + 1}: dist={distance:.2f} m, '
                f'vel=({vx:.2f}, {vy:.2f}, {vz:.2f})',
                throttle_duration_sec=1.0,
            )
            return

        if self.phase == 'LAND':
            # C++ offboard handles descent on arm=false; we just stop XY motion.
            self.publish_velocity(0.0, 0.0, 0.0)
            if self.current_z < 0.3:
                self.phase = 'DONE'
                self.get_logger().info('Landed. Mission complete.')
            return

        if self.phase == 'DONE':
            return


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
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
