#!/usr/bin/env python3
"""
Path Planner with Preview & Confirm (Stage 13).

Same A* planner as Stage 12, but adds a CONFIRMATION step:
  click goal -> plan -> show in YELLOW -> wait for user confirm
              -> on confirm: execute (path turns GREEN)
              -> on reject:  discard, wait for new click

Topics:
  Input:
    /clicked_point        - goal (RViz Publish Point tool)
    /confirm_path (Bool)  - user accepts the plan
    /reject_path  (Bool)  - user rejects the plan
  Output:
    /goal_pose            - waypoints to state machine
    /planned_path         - MarkerArray for visualization (color: yellow/green)
    /preview_status       - String: planner phase + summary
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3
from std_msgs.msg import Bool, String, ColorRGBA
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

import heapq
import math
import time
from ...common.interfaces import declare_interface_params


class PP:
    """Planner phase constants."""
    IDLE = 'IDLE'
    WAITING_FOR_HOVER = 'WAITING_FOR_HOVER'
    PLANNING = 'PLANNING'
    WAIT_CONFIRM = 'WAIT_CONFIRM'      # <-- NEW: planned, waiting user
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    NEXT_WP = 'NEXT_WP'
    DONE = 'DONE'
    FAILED = 'FAILED'


class PathPlannerPreview(Node):
    def __init__(self):
        super().__init__('path_planner_preview')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ========== Parameters ==========
        self.grid_min = (-15.0, -15.0, 0.0)
        self.grid_max = (15.0, 15.0, 8.0)
        self.resolution = 0.5
        self.inflation = 1.0
        # Match the state machine's default_altitude (the altitude the C++
        # offboard node actually holds), so planned goals are reachable.
        self.default_altitude = 2.5
        self.min_lidar_range = 0.5
        self.max_lidar_range = 15.0
        # Look-ahead so the planner sends the next waypoint as soon as the
        # drone is within this distance, instead of waiting for full stop.
        self.lookahead_distance = 0.5

        self.gx = int((self.grid_max[0] - self.grid_min[0]) / self.resolution)
        self.gy = int((self.grid_max[1] - self.grid_min[1]) / self.resolution)
        self.gz = int((self.grid_max[2] - self.grid_min[2]) / self.resolution)
        self.inflation_cells = int(round(self.inflation / self.resolution))

        # ========== State ==========
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude

        self.current_nav_state = 'UNKNOWN'

        self.occupied_cells = set()
        self.lidar_points = []

        # TF buffer to transform LiDAR points from sensor frame to map frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.phase = PP.WAITING_FOR_HOVER
        self.planned_path = []
        self.path_index = 0
        self.send_attempt_time = None
        self.path_length_meters = 0.0
        self.plan_time_seconds = 0.0

        # ========== Subscribers ==========
        # NOTE: PX4 v1.16+ versioned topic + sim's LiDAR topic.
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, 10
        )
        self.create_subscription(
            PointCloud2, self.topic('pointcloud'), self.lidar_callback, lidar_qos
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

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, '/planned_path', 10
        )
        self.grid_marker_pub = self.create_publisher(
            MarkerArray, '/grid_obstacles', 10
        )
        self.status_pub = self.create_publisher(
            String, '/preview_status', 10
        )

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('PATH PLANNER WITH PREVIEW & CONFIRM')
        self.get_logger().info('-' * 60)
        self.get_logger().info('In RViz:')
        self.get_logger().info('  Add MarkerArray on /planned_path')
        self.get_logger().info('  Use Publish Point to set goal')
        self.get_logger().info('-' * 60)
        self.get_logger().info('To confirm a previewed path (terminal):')
        self.get_logger().info(
            '  ros2 topic pub --once /confirm_path '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('To reject:')
        self.get_logger().info(
            '  ros2 topic pub --once /reject_path '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 60)

    # =====================================================
    # Subscribers
    # =====================================================
    def position_callback(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def lidar_callback(self, msg):
        if not self.position_received:
            return

        # Use TF to transform sensor-frame points to the map frame. Falls
        # back to treating points as already in map frame if TF is missing.
        frame_id = msg.header.frame_id or 'map'
        tx, ty, tz = 0.0, 0.0, 0.0
        r00, r01, r02 = 1.0, 0.0, 0.0
        r10, r11, r12 = 0.0, 1.0, 0.0
        r20, r21, r22 = 0.0, 0.0, 1.0
        if frame_id and frame_id != 'map':
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', frame_id, msg.header.stamp,
                    timeout=Duration(seconds=0.05),
                )
                tx = tf.transform.translation.x
                ty = tf.transform.translation.y
                tz = tf.transform.translation.z
                qx = tf.transform.rotation.x
                qy = tf.transform.rotation.y
                qz = tf.transform.rotation.z
                qw = tf.transform.rotation.w
                xx, yy, zz = qx * qx, qy * qy, qz * qz
                xy, xz, yz = qx * qy, qx * qz, qy * qz
                wxq, wyq, wzq = qw * qx, qw * qy, qw * qz
                r00 = 1.0 - 2.0 * (yy + zz)
                r01 = 2.0 * (xy - wzq)
                r02 = 2.0 * (xz + wyq)
                r10 = 2.0 * (xy + wzq)
                r11 = 1.0 - 2.0 * (xx + zz)
                r12 = 2.0 * (yz - wxq)
                r20 = 2.0 * (xz - wyq)
                r21 = 2.0 * (yz + wxq)
                r22 = 1.0 - 2.0 * (xx + yy)
            except Exception as e:
                self.get_logger().warn(
                    f'TF "{frame_id}" -> "map" unavailable, treating LiDAR '
                    f'points as already in map frame: {e}',
                    throttle_duration_sec=10.0,
                )

        pts = []
        try:
            points = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            for p in points:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                wx = r00 * x + r01 * y + r02 * z + tx
                wy = r10 * x + r11 * y + r12 * z + ty
                wz = r20 * x + r21 * y + r22 * z + tz
                dx = wx - self.current_x
                dy = wy - self.current_y
                dz = wz - self.current_z
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                if self.min_lidar_range < d < self.max_lidar_range:
                    pts.append((wx, wy, wz))
        except Exception as e:
            self.get_logger().warn(f'LiDAR error: {e}', throttle_duration_sec=5.0)
            return
        self.lidar_points = pts

    def clicked_callback(self, msg):
        # Always accept a click. If we were in the middle of an old plan/exec,
        # cancel it so the user is never stuck with a stale path.
        self.goal_x = float(msg.point.x)
        self.goal_y = float(msg.point.y)
        self.goal_z = self.default_altitude
        self.get_logger().info(
            f'>>> CLICK at ({self.goal_x:.2f}, {self.goal_y:.2f}) — '
            f'replanning. (was in phase {self.phase})'
        )
        self.planned_path = []
        self.path_index = 0
        self.set_phase(PP.PLANNING)

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    def confirm_callback(self, msg):
        if not msg.data:
            return
        if self.phase != PP.WAIT_CONFIRM:
            self.get_logger().warn(
                f'No path to confirm (phase={self.phase})'
            )
            return
        self.get_logger().info('>>> PATH CONFIRMED - executing <<<')
        self.set_phase(PP.SEND_WP)

    def reject_callback(self, msg):
        if not msg.data:
            return
        if self.phase != PP.WAIT_CONFIRM:
            self.get_logger().warn(
                f'No path to reject (phase={self.phase})'
            )
            return
        self.get_logger().info('Path REJECTED - clearing')
        self.planned_path = []
        self.path_index = 0
        self.set_phase(PP.IDLE)

    # =====================================================
    # Phase machine
    # =====================================================
    def tick(self):
        if self.phase == PP.WAITING_FOR_HOVER:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone hovering. Ready for goals.')
                self.set_phase(PP.IDLE)

        elif self.phase == PP.PLANNING:
            self.do_planning()

        elif self.phase == PP.WAIT_CONFIRM:
            # Wait for user. Path is visualized in yellow.
            pass

        elif self.phase == PP.SEND_WP:
            if self.path_index >= len(self.planned_path):
                self.get_logger().info('Path complete.')
                self.set_phase(PP.DONE)
                return
            x, y, z = self.planned_path[self.path_index]
            self.send_goal(x, y, z)
            self.send_attempt_time = time.time()
            self.set_phase(PP.WAIT_NAV)

        elif self.phase == PP.WAIT_NAV:
            # Accept NAVIGATING or HOVER (drone might fly through fast).
            if self.current_nav_state in ('NAVIGATING', 'HOVER'):
                self.set_phase(PP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > 5.0:
                self.set_phase(PP.SEND_WP)

        elif self.phase == PP.WAIT_HOVER:
            # Look-ahead for intermediate waypoints: advance early so motion
            # stays smooth. The final waypoint waits for a proper HOVER.
            is_final_wp = (self.path_index >= len(self.planned_path) - 1)
            if not is_final_wp and self.path_index < len(self.planned_path):
                wp = self.planned_path[self.path_index]
                dx = wp[0] - self.current_x
                dy = wp[1] - self.current_y
                dz = wp[2] - self.current_z
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                if d < self.lookahead_distance:
                    self.path_index += 1
                    self.set_phase(PP.NEXT_WP)
                    return

            if self.current_nav_state == 'HOVER':
                self.path_index += 1
                self.set_phase(PP.NEXT_WP)
            elif (self.send_attempt_time is not None
                  and time.time() - self.send_attempt_time > 20.0):
                self.get_logger().warn(
                    f'WP {self.path_index + 1} timeout — skipping to next'
                )
                self.path_index += 1
                self.set_phase(PP.NEXT_WP)

        elif self.phase == PP.NEXT_WP:
            self.set_phase(PP.SEND_WP)

        elif self.phase == PP.DONE:
            self.set_phase(PP.IDLE)

    def set_phase(self, new_phase):
        if new_phase != self.phase:
            self.get_logger().info(f'PLANNER: {self.phase} -> {new_phase}')
            self.phase = new_phase

    # =====================================================
    # Planning core
    # =====================================================
    def do_planning(self):
        if not self.position_received:
            self.set_phase(PP.FAILED)
            return

        self.build_grid_from_lidar()

        start = self.world_to_grid(self.current_x, self.current_y, self.current_z)
        goal = self.world_to_grid(self.goal_x, self.goal_y, self.goal_z)

        if start is None or goal is None:
            self.get_logger().error('Start/goal outside grid bounds')
            self.set_phase(PP.FAILED)
            return

        if goal in self.occupied_cells:
            self.get_logger().warn('Goal in obstacle - finding nearest free cell')
            goal = self.find_nearest_free(goal)
            if goal is None:
                self.set_phase(PP.FAILED)
                return

        t0 = time.time()
        path_cells = self.a_star(start, goal)
        self.plan_time_seconds = time.time() - t0

        if not path_cells:
            self.get_logger().error(f'No path found ({self.plan_time_seconds:.2f}s)')
            self.set_phase(PP.FAILED)
            return

        world_path = [self.grid_to_world(c) for c in path_cells]
        downsampled = world_path[::2]
        if downsampled[-1] != world_path[-1]:
            downsampled.append(world_path[-1])

        # Drop leading waypoints the drone is already on.
        skip_tol = 0.7
        while (len(downsampled) > 1 and
               math.hypot(downsampled[0][0] - self.current_x,
                          downsampled[0][1] - self.current_y) < skip_tol):
            downsampled.pop(0)

        # Snap the final waypoint to the exact click XY for precision landing.
        if downsampled:
            downsampled[-1] = (self.goal_x, self.goal_y, self.goal_z)

        self.planned_path = downsampled
        self.path_index = 0

        # Compute path length
        total = 0.0
        for i in range(len(self.planned_path) - 1):
            ax, ay, az = self.planned_path[i]
            bx, by, bz = self.planned_path[i + 1]
            total += math.sqrt(
                (bx - ax) ** 2 + (by - ay) ** 2 + (bz - az) ** 2
            )
        self.path_length_meters = total

        self.get_logger().info('=' * 50)
        self.get_logger().info('PATH READY FOR REVIEW')
        self.get_logger().info(f'  Waypoints: {len(self.planned_path)}')
        self.get_logger().info(f'  Length:    {self.path_length_meters:.2f} m')
        self.get_logger().info(f'  Plan time: {self.plan_time_seconds:.2f} s')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  CONFIRM:')
        self.get_logger().info(
            '    ros2 topic pub --once /confirm_path '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('  REJECT:')
        self.get_logger().info(
            '    ros2 topic pub --once /reject_path '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 50)
        self.set_phase(PP.WAIT_CONFIRM)

    # =====================================================
    # Grid + A* (same as stage 12)
    # =====================================================
    def world_to_grid(self, x, y, z):
        i = int((x - self.grid_min[0]) / self.resolution)
        j = int((y - self.grid_min[1]) / self.resolution)
        k = int((z - self.grid_min[2]) / self.resolution)
        if 0 <= i < self.gx and 0 <= j < self.gy and 0 <= k < self.gz:
            return (i, j, k)
        return None

    def grid_to_world(self, cell):
        i, j, k = cell
        x = self.grid_min[0] + (i + 0.5) * self.resolution
        y = self.grid_min[1] + (j + 0.5) * self.resolution
        z = self.grid_min[2] + (k + 0.5) * self.resolution
        return (x, y, z)

    def build_grid_from_lidar(self):
        occupied = set()
        for (wx, wy, wz) in self.lidar_points:
            cell = self.world_to_grid(wx, wy, wz)
            if cell is not None:
                occupied.add(cell)

        inflated = set()
        r = self.inflation_cells
        for (i, j, k) in occupied:
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    for dk in range(-r, r + 1):
                        ni, nj, nk = i + di, j + dj, k + dk
                        if (0 <= ni < self.gx and
                                0 <= nj < self.gy and
                                0 <= nk < self.gz):
                            inflated.add((ni, nj, nk))
        self.occupied_cells = inflated

    def find_nearest_free(self, target):
        ti, tj, tk = target
        for r in range(1, 10):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    for dk in range(-r, r + 1):
                        if abs(di) != r and abs(dj) != r and abs(dk) != r:
                            continue
                        ni, nj, nk = ti + di, tj + dj, tk + dk
                        if (0 <= ni < self.gx and
                                0 <= nj < self.gy and
                                0 <= nk < self.gz and
                                (ni, nj, nk) not in self.occupied_cells):
                            return (ni, nj, nk)
        return None

    def a_star(self, start, goal):
        def heuristic(a, b):
            return math.sqrt(
                (a[0] - b[0]) ** 2 +
                (a[1] - b[1]) ** 2 +
                (a[2] - b[2]) ** 2
            )

        neighbors = [
            (di, dj, dk)
            for di in (-1, 0, 1)
            for dj in (-1, 0, 1)
            for dk in (-1, 0, 1)
            if not (di == 0 and dj == 0 and dk == 0)
        ]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        max_iter = 100000
        it = 0

        while open_heap and it < max_iter:
            it += 1
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))

            if current in closed:
                continue
            closed.add(current)

            ci, cj, ck = current
            for (di, dj, dk) in neighbors:
                ni, nj, nk = ci + di, cj + dj, ck + dk
                if not (0 <= ni < self.gx and 0 <= nj < self.gy and 0 <= nk < self.gz):
                    continue
                neighbor = (ni, nj, nk)
                if neighbor in self.occupied_cells:
                    continue
                if neighbor in closed:
                    continue
                step = math.sqrt(di * di + dj * dj + dk * dk)
                tentative_g = g_score[current] + step
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))
        return None

    # =====================================================
    # Publishing
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
            f'>>> WP {self.path_index + 1}/{len(self.planned_path)}: '
            f'({x:.2f}, {y:.2f}, {z:.2f})'
        )

    def publish_status(self):
        status = f'{self.phase}'
        if self.phase == PP.WAIT_CONFIRM:
            status += (
                f' | wps={len(self.planned_path)} | '
                f'len={self.path_length_meters:.2f}m'
            )
        elif self.phase in (PP.SEND_WP, PP.WAIT_NAV, PP.WAIT_HOVER, PP.NEXT_WP):
            status += f' | wp={self.path_index + 1}/{len(self.planned_path)}'
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_visualization(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Color depends on phase
        if self.phase == PP.WAIT_CONFIRM:
            line_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # YELLOW
            point_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            status_text = 'AWAITING CONFIRMATION'
            status_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        elif self.phase in (PP.SEND_WP, PP.WAIT_NAV, PP.WAIT_HOVER, PP.NEXT_WP):
            line_color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)  # GREEN
            point_color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)
            status_text = 'EXECUTING'
            status_color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)
        else:
            line_color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
            point_color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
            status_text = ''
            status_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        if self.planned_path:
            line = Marker()
            line.header.frame_id = self.frame('map')
            line.header.stamp = now
            line.ns = 'preview_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = line_color
            for (x, y, z) in self.planned_path:
                line.points.append(Point(x=x, y=y, z=z))
            arr.markers.append(line)

            for i, (x, y, z) in enumerate(self.planned_path):
                sp = Marker()
                sp.header.frame_id = self.frame('map')
                sp.header.stamp = now
                sp.ns = 'preview_pts'
                sp.id = i
                sp.type = Marker.SPHERE
                sp.action = Marker.ADD
                sp.pose.position.x = x
                sp.pose.position.y = y
                sp.pose.position.z = z
                sp.pose.orientation.w = 1.0
                sp.scale = Vector3(x=0.25, y=0.25, z=0.25)
                if (i == self.path_index and
                        self.phase in (PP.SEND_WP, PP.WAIT_NAV, PP.WAIT_HOVER)):
                    sp.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                    sp.scale = Vector3(x=0.4, y=0.4, z=0.4)
                elif i < self.path_index:
                    sp.color = ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.6)
                else:
                    sp.color = point_color
                arr.markers.append(sp)

            # Status text near goal
            if status_text:
                txt = Marker()
                txt.header.frame_id = self.frame('map')
                txt.header.stamp = now
                txt.ns = 'preview_text'
                txt.id = 0
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                last = self.planned_path[-1]
                txt.pose.position.x = last[0]
                txt.pose.position.y = last[1]
                txt.pose.position.z = last[2] + 1.0
                txt.scale.z = 0.5
                txt.color = status_color
                txt.text = status_text
                arr.markers.append(txt)

        self.path_marker_pub.publish(arr)

        # Obstacles (downsampled)
        if self.occupied_cells:
            obs = MarkerArray()
            cube = Marker()
            cube.header.frame_id = self.frame('map')
            cube.header.stamp = now
            cube.ns = 'grid_obstacles'
            cube.id = 0
            cube.type = Marker.CUBE_LIST
            cube.action = Marker.ADD
            cube.scale = Vector3(
                x=self.resolution, y=self.resolution, z=self.resolution
            )
            cube.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.35)
            for cell in list(self.occupied_cells)[::3]:
                x, y, z = self.grid_to_world(cell)
                cube.points.append(Point(x=x, y=y, z=z))
            obs.markers.append(cube)
            self.grid_marker_pub.publish(obs)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerPreview()
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
