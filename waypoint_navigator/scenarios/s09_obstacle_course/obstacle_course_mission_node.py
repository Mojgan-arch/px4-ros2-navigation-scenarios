#!/usr/bin/env python3
"""
Obstacle Course Auto Mission

Drives the drone through a fixed, narrative sequence of waypoints inside
the obstacle_course world. The user does NOT click anything — the mission
is fully autonomous.

Mission narrative (matches obstacle_course.sdf colors):
  1. Right side of a BLUE column   (col_r2_2 at X=-3, Y=+3)
  2. Left side of an ORANGE column (col_r1_2 at X=-6, Y=0)
  3. One lap around a YELLOW box   (box_3 at X=+4.5, Y=+5.5)
  4. Return to start (0, 0) and land

The mission node only publishes /goal_pose — obstacle_avoidance_node
handles LiDAR-based avoidance and cmd_vel.

A per-waypoint timeout forces the mission to advance if the drone gets
stuck (e.g., the wall pushes it away from a tight waypoint). This keeps
the mission deterministic from start to finish.
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
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from ...common.interfaces import declare_interface_params


class ObstacleCourseMission(Node):
    def __init__(self):
        super().__init__('obstacle_course_mission')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ===== Mission tuning =====
        self.altitude = 2.5
        self.waypoint_tolerance = 0.6    # m — tight for crisper lap corners
        self.waypoint_timeout = 12.0     # s — force advance after this long
        self.takeoff_threshold = 2.0     # m — wait until drone reaches this z

        # When false, the per-waypoint / LAP / HOME text labels are not
        # published. Useful when you just want clean rings + spheres in RViz.
        self.declare_parameter('show_text_labels', True)
        self.show_text_labels = self.get_parameter('show_text_labels').value

        # ===== Build mission waypoint list — TWO LAPS =====
        # Both laps are 8-point CCW circles, entered from the side
        # closest to the drone's current position (so the transit
        # line goes straight in, not across the lap).
        #
        # Lap 1: PURPLE col_r4_2 at (+3, -1) — closest PURPLE column
        #        to home. Drone enters from the WEST side (closest
        #        to home (0,0)) and laps CCW.
        # Lap 2: YELLOW box_3 at (+4.5, +5.5) — the big tan box
        #        north-east of home. Drone enters from the SW side
        #        (closest to Lap 1 exit) and laps CCW.
        # Return to home (0, 0) and land.

        self.lap1_cx, self.lap1_cy, self.lap1_r = 3.0, -1.0, 1.8
        self.lap2_cx, self.lap2_cy, self.lap2_r = 4.5,  5.5, 1.5

        # Direction names by index. CCW means index increases.
        DIRS = [
            ('E',   0.0),
            ('NE', 45.0),
            ('N',  90.0),
            ('NW', 135.0),
            ('W',  180.0),
            ('SW', 225.0),
            ('S',  270.0),
            ('SE', 315.0),
        ]

        def lap_waypoints(cx, cy, r, label, start_idx):
            """8 CCW waypoints around (cx,cy) starting at DIRS[start_idx]."""
            pts = []
            for i in range(8):
                name, ang_deg = DIRS[(start_idx + i) % 8]
                rad = math.radians(ang_deg)
                pts.append((
                    f'{label} {name}',
                    cx + r * math.cos(rad),
                    cy + r * math.sin(rad),
                    self.altitude,
                ))
            # Close the lap by going back to the starting waypoint
            start_name, start_deg = DIRS[start_idx]
            srad = math.radians(start_deg)
            pts.append((
                f'Close {label} ({start_name})',
                cx + r * math.cos(srad),
                cy + r * math.sin(srad),
                self.altitude,
            ))
            return pts

        # DIRS index: W=4, SW=5
        wps = []
        wps += lap_waypoints(self.lap1_cx, self.lap1_cy, self.lap1_r,
                             'PURPLE col', start_idx=4)   # enter from W
        wps += lap_waypoints(self.lap2_cx, self.lap2_cy, self.lap2_r,
                             'YELLOW box', start_idx=5)   # enter from SW
        wps.append(('Return home', 0.0, 0.0, self.altitude))

        self.waypoints = wps
        self.wp_index = 0
        self.wp_start_time = None

        # WAIT_TAKEOFF -> MISSION -> LAND -> DONE
        self.phase = 'WAIT_TAKEOFF'

        # Drone position (ENU)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos,
        )

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arm_pub = self.create_publisher(
            Bool, self.topic('arm'), 10
        )
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        # Trajectory & odometry (FAST-LIO style RViz visualisation)
        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone_odom', 10)
        self._path_msg = Path()
        self._path_msg.header.frame_id = self.frame('map')
        self._last_path_xy = None  # only append when moved >= path_min_step

        # Timers
        self.create_timer(0.5, self.control_loop)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.5, self.publish_markers)
        # Republish current goal every 2s so obstacle_avoidance reliably picks it up
        self.create_timer(2.0, self.republish_current_goal)
        # Trajectory at 10 Hz, odom at 20 Hz
        self.create_timer(0.1, self.publish_path)
        self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info(
            f'Obstacle Course Auto Mission started. {len(self.waypoints)} waypoints.'
        )
        for i, (name, x, y, z) in enumerate(self.waypoints):
            self.get_logger().info(
                f'  WP {i + 1}: {name:24s} ({x:+.2f}, {y:+.2f}, {z:+.2f})'
            )

    # -----------------------------------------------------
    def position_callback(self, msg):
        # NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def publish_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = self.frame('map')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def republish_current_goal(self):
        if self.phase == 'MISSION' and self.wp_index < len(self.waypoints):
            _, x, y, z = self.waypoints[self.wp_index]
            self.publish_goal(x, y, z)

    def send_current_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            return
        name, x, y, z = self.waypoints[self.wp_index]
        self.publish_goal(x, y, z)
        self.wp_start_time = time.time()
        self.get_logger().info(
            f'>>> WP {self.wp_index + 1}/{len(self.waypoints)}: '
            f'{name} at ({x:+.2f}, {y:+.2f}, {z:+.2f})'
        )

    def advance_waypoint(self, reason):
        name, x, y, _ = self.waypoints[self.wp_index]
        self.get_logger().info(
            f'<<< WP {self.wp_index + 1}/{len(self.waypoints)} done '
            f'({reason}): {name}'
        )
        self.wp_index += 1
        if self.wp_index < len(self.waypoints):
            self.send_current_waypoint()

    # -----------------------------------------------------
    def control_loop(self):
        if not self.position_received:
            return

        if self.phase == 'WAIT_TAKEOFF':
            if self.current_z >= self.takeoff_threshold:
                self.phase = 'MISSION'
                self.send_current_waypoint()
                self.get_logger().info(
                    f'Takeoff complete (z={self.current_z:.2f} m). '
                    f'Starting mission.'
                )
            return

        if self.phase == 'MISSION':
            if self.wp_index >= len(self.waypoints):
                self.phase = 'LAND'
                self.get_logger().info(
                    'All waypoints visited. Sending land command.'
                )
                msg = Bool()
                msg.data = False
                self.arm_pub.publish(msg)
                return

            _, x, y, _ = self.waypoints[self.wp_index]
            dx = x - self.current_x
            dy = y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)

            elapsed = time.time() - (self.wp_start_time or time.time())
            if distance < self.waypoint_tolerance:
                self.advance_waypoint(f'reached, dist={distance:.2f}m')
                return
            if elapsed > self.waypoint_timeout:
                self.advance_waypoint(
                    f'TIMEOUT after {elapsed:.1f}s, dist={distance:.2f}m'
                )
                return
            return

        if self.phase == 'LAND':
            if self.current_z < 0.3:
                self.phase = 'DONE'
                self.get_logger().info('Landed. Mission DONE.')
            return

        if self.phase == 'DONE':
            return

    # -----------------------------------------------------
    def publish_status(self):
        msg = String()
        if self.phase == 'MISSION' and self.wp_index < len(self.waypoints):
            name, x, y, _ = self.waypoints[self.wp_index]
            dx = x - self.current_x
            dy = y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)
            elapsed = time.time() - (self.wp_start_time or time.time())
            msg.data = (
                f'phase={self.phase} | wp={self.wp_index + 1}/{len(self.waypoints)} | '
                f'"{name}" | target=({x:+.1f},{y:+.1f}) | '
                f'pos=({self.current_x:+.2f},{self.current_y:+.2f},'
                f'{self.current_z:+.2f}) | dist={dist:.2f}m | '
                f'elapsed={elapsed:.1f}s'
            )
        else:
            msg.data = (
                f'phase={self.phase} | '
                f'pos=({self.current_x:+.2f},{self.current_y:+.2f},'
                f'{self.current_z:+.2f}) | '
                f'wp_index={self.wp_index}/{len(self.waypoints)}'
            )
        self.status_pub.publish(msg)

    # -----------------------------------------------------
    def publish_path(self):
        """Append current drone XY to the trajectory and republish."""
        if not self.position_received:
            return
        # only sample when drone has moved >= 0.1 m (avoid huge lists)
        if self._last_path_xy is not None:
            ldx = self.current_x - self._last_path_xy[0]
            ldy = self.current_y - self._last_path_xy[1]
            if ldx * ldx + ldy * ldy < 0.01:
                # nothing new to add, but still republish what we have
                self._path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self._path_msg)
                return

        ps = PoseStamped()
        ps.header.frame_id = self.frame('map')
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.current_x
        ps.pose.position.y = self.current_y
        ps.pose.position.z = self.current_z
        ps.pose.orientation.w = 1.0
        self._path_msg.poses.append(ps)
        # cap history at 2000 points (~3 min @ 10 Hz)
        if len(self._path_msg.poses) > 2000:
            self._path_msg.poses = self._path_msg.poses[-2000:]
        self._last_path_xy = (self.current_x, self.current_y)

        self._path_msg.header.stamp = ps.header.stamp
        self.path_pub.publish(self._path_msg)

    def publish_odometry(self):
        """Publish drone pose as Odometry — RViz draws it as an arrow."""
        if not self.position_received:
            return
        odom = Odometry()
        odom.header.frame_id = self.frame('map')
        odom.child_frame_id = self.frame('base_link')
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = self.current_z
        # Yaw from velocity direction (last segment of path), else identity
        yaw = 0.0
        if len(self._path_msg.poses) >= 2:
            a = self._path_msg.poses[-2].pose.position
            b = self._path_msg.poses[-1].pose.position
            yaw = math.atan2(b.y - a.y, b.x - a.x)
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

    # -----------------------------------------------------
    def publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Spheres + labels at each waypoint
        for i, (name, x, y, z) in enumerate(self.waypoints):
            # Sphere
            m = Marker()
            m.header.frame_id = self.frame('map')
            m.header.stamp = now
            m.ns = 'mission_wps'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            if i < self.wp_index:
                m.color.r, m.color.g, m.color.b = 0.2, 0.9, 0.2  # green (done)
            elif i == self.wp_index and self.phase == 'MISSION':
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0  # orange (current)
            else:
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5  # gray (todo)
            m.color.a = 0.9
            ma.markers.append(m)

            # Disk on the ground (footprint)
            disk = Marker()
            disk.header.frame_id = self.frame('map')
            disk.header.stamp = now
            disk.ns = 'mission_disks'
            disk.id = i
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position.x = x
            disk.pose.position.y = y
            disk.pose.position.z = 0.05
            disk.pose.orientation.w = 1.0
            disk.scale.x = 0.6
            disk.scale.y = 0.6
            disk.scale.z = 0.1
            disk.color.r = m.color.r
            disk.color.g = m.color.g
            disk.color.b = m.color.b
            disk.color.a = 0.6
            ma.markers.append(disk)

            # Vertical pillar
            pillar = Marker()
            pillar.header.frame_id = self.frame('map')
            pillar.header.stamp = now
            pillar.ns = 'mission_pillars'
            pillar.id = i
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.pose.position.x = x
            pillar.pose.position.y = y
            pillar.pose.position.z = z / 2.0
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = 0.04
            pillar.scale.y = 0.04
            pillar.scale.z = z
            pillar.color.r = m.color.r
            pillar.color.g = m.color.g
            pillar.color.b = m.color.b
            pillar.color.a = 0.4
            ma.markers.append(pillar)

            # Text label (skip when show_text_labels:=false)
            if self.show_text_labels:
                t = Marker()
                t.header.frame_id = self.frame('map')
                t.header.stamp = now
                t.ns = 'mission_labels'
                t.id = i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = x
                t.pose.position.y = y
                t.pose.position.z = z + 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.4
                t.color.r = t.color.g = t.color.b = 1.0
                t.color.a = 1.0
                t.text = f'{i + 1}: {name}'
                ma.markers.append(t)

        # Line strip from home through all waypoints
        line = Marker()
        line.header.frame_id = self.frame('map')
        line.header.stamp = now
        line.ns = 'mission_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 0.7
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = self.altitude
        line.points.append(p0)
        for (_, x, y, z) in self.waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            line.points.append(p)
        ma.markers.append(line)

        # ---- Lap-1 ring marker (PURPLE column) ----
        ring1 = Marker()
        ring1.header.frame_id = self.frame('map')
        ring1.header.stamp = now
        ring1.ns = 'lap1_ring'
        ring1.id = 0
        ring1.type = Marker.LINE_STRIP
        ring1.action = Marker.ADD
        ring1.pose.orientation.w = 1.0
        ring1.scale.x = 0.08
        ring1.color.r = 0.6
        ring1.color.g = 0.2
        ring1.color.b = 0.9
        ring1.color.a = 0.8
        for i in range(33):  # 32 segments + close
            ang = i * (2.0 * math.pi / 32.0)
            p = Point()
            p.x = self.lap1_cx + self.lap1_r * math.cos(ang)
            p.y = self.lap1_cy + self.lap1_r * math.sin(ang)
            p.z = self.altitude
            ring1.points.append(p)
        ma.markers.append(ring1)

        if self.show_text_labels:
            ring1_label = Marker()
            ring1_label.header.frame_id = self.frame('map')
            ring1_label.header.stamp = now
            ring1_label.ns = 'lap1_label'
            ring1_label.id = 0
            ring1_label.type = Marker.TEXT_VIEW_FACING
            ring1_label.action = Marker.ADD
            ring1_label.pose.position.x = self.lap1_cx
            ring1_label.pose.position.y = self.lap1_cy
            ring1_label.pose.position.z = self.altitude + 1.5
            ring1_label.pose.orientation.w = 1.0
            ring1_label.scale.z = 0.5
            ring1_label.color.r = 0.6
            ring1_label.color.g = 0.2
            ring1_label.color.b = 0.9
            ring1_label.color.a = 1.0
            ring1_label.text = 'LAP 1: PURPLE col_r4_2'
            ma.markers.append(ring1_label)

        # ---- Lap-2 ring marker (BLUE col + YELLOW box pair) ----
        ring2 = Marker()
        ring2.header.frame_id = self.frame('map')
        ring2.header.stamp = now
        ring2.ns = 'lap2_ring'
        ring2.id = 0
        ring2.type = Marker.LINE_STRIP
        ring2.action = Marker.ADD
        ring2.pose.orientation.w = 1.0
        ring2.scale.x = 0.08
        ring2.color.r = 0.2
        ring2.color.g = 0.7
        ring2.color.b = 1.0
        ring2.color.a = 0.8
        for i in range(33):
            ang = i * (2.0 * math.pi / 32.0)
            p = Point()
            p.x = self.lap2_cx + self.lap2_r * math.cos(ang)
            p.y = self.lap2_cy + self.lap2_r * math.sin(ang)
            p.z = self.altitude
            ring2.points.append(p)
        ma.markers.append(ring2)

        if self.show_text_labels:
            ring2_label = Marker()
            ring2_label.header.frame_id = self.frame('map')
            ring2_label.header.stamp = now
            ring2_label.ns = 'lap2_label'
            ring2_label.id = 0
            ring2_label.type = Marker.TEXT_VIEW_FACING
            ring2_label.action = Marker.ADD
            ring2_label.pose.position.x = self.lap2_cx
            ring2_label.pose.position.y = self.lap2_cy
            ring2_label.pose.position.z = self.altitude + 1.5
            ring2_label.pose.orientation.w = 1.0
            ring2_label.scale.z = 0.5
            ring2_label.color.r = 0.2
            ring2_label.color.g = 0.7
            ring2_label.color.b = 1.0
            ring2_label.color.a = 1.0
            ring2_label.text = 'LAP 2: YELLOW box_3'
            ma.markers.append(ring2_label)

        # Home marker (red cylinder on ground)
        home = Marker()
        home.header.frame_id = self.frame('map')
        home.header.stamp = now
        home.ns = 'home'
        home.id = 0
        home.type = Marker.CYLINDER
        home.action = Marker.ADD
        home.pose.position.x = 0.0
        home.pose.position.y = 0.0
        home.pose.position.z = 0.1
        home.pose.orientation.w = 1.0
        home.scale.x = 0.8
        home.scale.y = 0.8
        home.scale.z = 0.1
        home.color.r = 1.0
        home.color.g = 0.0
        home.color.b = 0.0
        home.color.a = 0.8
        ma.markers.append(home)

        if self.show_text_labels:
            home_t = Marker()
            home_t.header.frame_id = self.frame('map')
            home_t.header.stamp = now
            home_t.ns = 'home_label'
            home_t.id = 0
            home_t.type = Marker.TEXT_VIEW_FACING
            home_t.action = Marker.ADD
            home_t.pose.position.x = 0.0
            home_t.pose.position.y = 0.0
            home_t.pose.position.z = 0.5
            home_t.pose.orientation.w = 1.0
            home_t.scale.z = 0.4
            home_t.color.r = home_t.color.g = home_t.color.b = 1.0
            home_t.color.a = 1.0
            home_t.text = 'HOME'
            ma.markers.append(home_t)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCourseMission()
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
