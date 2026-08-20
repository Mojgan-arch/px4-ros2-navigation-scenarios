#!/usr/bin/env python3
"""
Path Planner Node - A* on a 3D occupancy grid built from LiDAR.

Workflow:
  1. User clicks 'Publish Point' in RViz -> /clicked_point
  2. Planner reads current LiDAR scan, builds 3D occupancy grid
  3. A* finds shortest collision-free path from drone to goal
  4. Path is published as MarkerArray for RViz
  5. Waypoints are sent one-by-one to the state machine via /goal_pose

Architecture:
  This is a HIGH-LEVEL planner. It sits above the state machine
  (same pattern as inspection_mission / mission_editor).
  On hardware, set topics.pointcloud to your mapping stack's output
  (octomap_server output) - same logic, denser map.
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
    EXECUTING = 'EXECUTING'
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    NEXT_WP = 'NEXT_WP'
    DONE = 'DONE'
    FAILED = 'FAILED'


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
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

        # ========== Grid parameters ==========
        self.grid_min = (-15.0, -15.0,  0.0)   # x_min, y_min, z_min
        self.grid_max = ( 15.0,  15.0,  8.0)   # x_max, y_max, z_max
        self.resolution = 0.5                  # meters per cell
        self.inflation = 1.0                   # meters - safety buffer
        # Match the state machine's default_altitude (the altitude the C++
        # offboard node actually holds), so planned goals are reachable.
        self.default_altitude = 2.5
        # Vertical dip applied to mid-path waypoints. Disabled (0.0) — every
        # dip value tried interacted with the doorway-traversal forces and
        # introduced bobbing. With TF-correct planning the drone now follows
        # the green line precisely; we keep the path flat at default_altitude
        # for a clean, smooth flight.
        self.altitude_dip = 0.0                # meters (down, mid-path)
        # Look-ahead: send the next waypoint as soon as the drone is within
        # this distance of the current one, instead of waiting for it to fully
        # stop. Removes the stop-and-go wobble between intermediate waypoints.
        # Only the FINAL waypoint waits for a proper HOVER (precision).
        # Must be SMALLER than the waypoint spacing (~1 m after [::2] downsample
        # on a 0.5 m grid) otherwise the planner advances before the drone
        # really visits each waypoint, and the drone effectively cuts corners
        # straight through walls instead of following the green line.
        self.lookahead_distance = 0.5          # meters
        self.min_lidar_range = 0.5             # ignore points closer than this
        self.max_lidar_range = 15.0

        # Grid dimensions
        self.gx = int((self.grid_max[0] - self.grid_min[0]) / self.resolution)
        self.gy = int((self.grid_max[1] - self.grid_min[1]) / self.resolution)
        self.gz = int((self.grid_max[2] - self.grid_min[2]) / self.resolution)
        self.inflation_cells = int(round(self.inflation / self.resolution))

        # ========== State ==========
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0          # ENU yaw (rad), CCW from +x_East
        self.position_received = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude

        self.current_nav_state = 'UNKNOWN'

        self.occupied_cells = set()       # set of (i, j, k) - inflated obstacles
        self.lidar_points = []            # list of (x, y, z) in world frame

        # TF buffer so we can transform LiDAR points from sensor frame to map
        # frame properly (handles yaw/pitch/roll without the assumptions the
        # manual code had). Falls back to "raw points" if TF isn't available.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.phase = PP.WAITING_FOR_HOVER
        self.planned_path = []            # list of (x, y, z) waypoints
        self.path_index = 0
        self.send_attempt_time = None

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

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_marker_pub = self.create_publisher(
            MarkerArray, '/planned_path', 10
        )
        self.grid_marker_pub = self.create_publisher(
            MarkerArray, '/grid_obstacles', 10
        )

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('=' * 55)
        self.get_logger().info('PATH PLANNER (A* on 3D grid)')
        self.get_logger().info(
            f'Grid: X[{self.grid_min[0]}, {self.grid_max[0]}] '
            f'Y[{self.grid_min[1]}, {self.grid_max[1]}] '
            f'Z[{self.grid_min[2]}, {self.grid_max[2]}]'
        )
        self.get_logger().info(
            f'Resolution: {self.resolution}m | Inflation: {self.inflation}m'
        )
        self.get_logger().info(
            f'Grid cells: {self.gx} x {self.gy} x {self.gz}'
        )
        self.get_logger().info('-' * 55)
        self.get_logger().info('In RViz:')
        self.get_logger().info('  Add MarkerArray on /planned_path')
        self.get_logger().info('  Use Publish Point to set goal')
        self.get_logger().info('=' * 55)

    # =====================================================
    # Subscribers
    # =====================================================
    def position_callback(self, msg):
        # NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        # PX4 heading is yaw from North in NED (CW positive). ENU yaw is
        # measured from East CCW, so: enu_yaw = pi/2 - heading. Without this
        # the LiDAR rotation below silently assumes the drone is facing East
        # and the obstacle grid ends up rotated off from the real walls —
        # A* then plans straight through them.
        self.current_yaw = (math.pi / 2.0) - float(msg.heading)
        self.position_received = True

    def lidar_callback(self, msg):
        if not self.position_received:
            return

        # Resolve the sensor->map transform via TF so we don't have to guess
        # the LiDAR's body-frame convention (FLU vs FRD) or fight the yaw
        # sign. If TF is unavailable, fall back to treating the points as
        # already being in the map frame (Gazebo+ROS bridges often publish
        # LiDAR clouds in the world frame, which made the previous manual
        # body-frame math add the drone position twice).
        frame_id = msg.header.frame_id or 'map'
        tx, ty, tz = 0.0, 0.0, 0.0
        # Identity rotation as default (used if TF lookup fails or frame is map)
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
                # Build a 3x3 rotation matrix from the quaternion
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
                # No TF — use points as-is (assume map frame). Throttle the log
                # so it doesn't spam at 10 Hz.
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
                # Apply the homogeneous transform to get the map-frame point.
                wx = r00 * x + r01 * y + r02 * z + tx
                wy = r10 * x + r11 * y + r12 * z + ty
                wz = r20 * x + r21 * y + r22 * z + tz
                # Range filter is relative to the drone, not the origin.
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
        # Always accept a new click — cancel any in-progress plan/execution so
        # the user is never wondering whether their click registered.
        self.goal_x = float(msg.point.x)
        self.goal_y = float(msg.point.y)
        self.goal_z = self.default_altitude
        self.get_logger().info(
            f'>>> CLICK at ({self.goal_x:.2f}, {self.goal_y:.2f}) — '
            f'replanning. (was in phase {self.phase})'
        )
        # Cancel the in-progress mission (if any) and start fresh planning.
        self.planned_path = []
        self.path_index = 0
        self.set_phase(PP.PLANNING)

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

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
            # Accept either NAVIGATING or HOVER — if the drone already reached
            # the waypoint quickly we may have skipped past NAVIGATING entirely.
            if self.current_nav_state in ('NAVIGATING', 'HOVER'):
                self.set_phase(PP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > 5.0:
                self.set_phase(PP.SEND_WP)

        elif self.phase == PP.WAIT_HOVER:
            # Look-ahead: for any waypoint that is NOT the final one, advance
            # as soon as the drone enters lookahead_distance of it. The state
            # machine then smoothly retargets to the next waypoint without
            # ever stopping at the intermediate one.
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

            # Final waypoint (or still far from intermediate): wait for the
            # state machine to report HOVER, with a timeout as a safety net.
            if self.current_nav_state == 'HOVER':
                self.path_index += 1
                self.set_phase(PP.NEXT_WP)
            elif (self.send_attempt_time is not None
                  and time.time() - self.send_attempt_time > 20.0):
                # The drone can't reach this waypoint (probably stuck against
                # a wall or oscillating in a narrow corridor). Skip it and try
                # the next one so the mission doesn't hang forever.
                self.get_logger().warn(
                    f'WP {self.path_index + 1} timeout — skipping to next'
                )
                self.path_index += 1
                self.set_phase(PP.NEXT_WP)

        elif self.phase == PP.NEXT_WP:
            self.set_phase(PP.SEND_WP)

        elif self.phase == PP.DONE:
            # Wait for next click
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
            self.get_logger().warn('No position yet')
            self.set_phase(PP.FAILED)
            return

        self.get_logger().info('Building occupancy grid from LiDAR...')
        self.build_grid_from_lidar()
        self.get_logger().info(
            f'Grid built: {len(self.occupied_cells)} inflated occupied cells'
        )

        start = self.world_to_grid(
            self.current_x, self.current_y, self.current_z
        )
        goal = self.world_to_grid(self.goal_x, self.goal_y, self.goal_z)

        if start is None or goal is None:
            self.get_logger().error('Start or goal outside grid bounds')
            self.set_phase(PP.FAILED)
            return

        # If start cell is occupied, try to find nearest free cell
        if start in self.occupied_cells:
            self.get_logger().warn('Start cell occupied - drone is too close to obstacle')
        if goal in self.occupied_cells:
            self.get_logger().warn('Goal cell is in an obstacle - finding nearest free')
            goal = self.find_nearest_free(goal)
            if goal is None:
                self.set_phase(PP.FAILED)
                return

        self.get_logger().info(f'Running A* from {start} to {goal}...')
        t0 = time.time()
        path_cells = self.a_star(start, goal)
        elapsed = time.time() - t0

        if not path_cells:
            self.get_logger().error(f'No path found ({elapsed:.2f}s)')
            self.set_phase(PP.FAILED)
            return

        # Convert grid path to world coordinates
        world_path = [self.grid_to_world(c) for c in path_cells]
        # Downsample - keep every 2nd point + last. Denser than [::3] so the
        # drone tracks the planned curve more faithfully (otherwise it skips
        # most of the visible green line and ends up flying near-straight).
        downsampled = world_path[::2]
        if downsampled[-1] != world_path[-1]:
            downsampled.append(world_path[-1])

        # Drop any leading waypoints the drone is essentially already on. If
        # the first waypoint sits at the drone's current XY (which happens
        # because A* starts from the drone's own grid cell) the state machine
        # reports "reached" in a single tick — too fast for the planner's
        # 0.5 Hz WAIT_NAV poll to see, so the mission stalls in a resend loop.
        skip_tol = 0.7  # m — slightly larger than state machine tolerance (0.5)
        while (len(downsampled) > 1 and
               math.hypot(downsampled[0][0] - self.current_x,
                          downsampled[0][1] - self.current_y) < skip_tol):
            downsampled.pop(0)

        # Snap the final waypoint to the exact click XY (not the grid cell
        # centre) so the drone lands right on the yellow marker. With the
        # planner's grid at 0.5 m resolution and inflation, snapping to the
        # cell centre could leave the drone up to ~0.7 m off the click —
        # which is what the user was seeing. We snap unconditionally; the
        # rest of the path stays in the safe corridor A* found.
        if downsampled:
            downsampled[-1] = (self.goal_x, self.goal_y, self.goal_z)

        # Add a sinusoidal altitude dip to the middle of the path so the drone
        # visibly descends/ascends along the way (purely horizontal motion is
        # hard to tell apart from straight-line travel). Endpoints unchanged so
        # the drone starts and ends at the requested altitude.
        # Each dipped point is checked against the occupancy grid; if it falls
        # inside an inflated obstacle (typical near walls) we keep the original
        # safe altitude instead, so the post-processed path stays collision-free.
        n = len(downsampled)
        varied = []
        unsafe_skipped = 0
        for i, (x, y, z) in enumerate(downsampled):
            if 0 < i < n - 1 and n > 2:
                dip = -self.altitude_dip * math.sin(math.pi * i / (n - 1))
                candidate_z = z + dip
                cell = self.world_to_grid(x, y, candidate_z)
                if cell is not None and cell not in self.occupied_cells:
                    z = candidate_z
                else:
                    unsafe_skipped += 1
            varied.append((x, y, z))
        if unsafe_skipped:
            self.get_logger().info(
                f'Altitude-dip skipped on {unsafe_skipped} waypoint(s) '
                '(would intersect obstacles); kept original altitude.'
            )
        self.planned_path = varied
        self.path_index = 0

        self.get_logger().info(
            f'Path found: {len(path_cells)} cells, '
            f'{len(self.planned_path)} waypoints ({elapsed:.2f}s)'
        )
        self.set_phase(PP.SEND_WP)

    # =====================================================
    # Grid helpers
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

        # Inflate obstacles
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
        """Spiral outward from target to find a free cell."""
        ti, tj, tk = target
        for r in range(1, 10):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    for dk in range(-r, r + 1):
                        if abs(di) != r and abs(dj) != r and abs(dk) != r:
                            continue  # only the shell
                        ni, nj, nk = ti + di, tj + dj, tk + dk
                        if (0 <= ni < self.gx and
                                0 <= nj < self.gy and
                                0 <= nk < self.gz and
                                (ni, nj, nk) not in self.occupied_cells):
                            return (ni, nj, nk)
        return None

    # =====================================================
    # A* search
    # =====================================================
    def a_star(self, start, goal):
        def heuristic(a, b):
            return math.sqrt(
                (a[0] - b[0]) ** 2 +
                (a[1] - b[1]) ** 2 +
                (a[2] - b[2]) ** 2
            )

        # 26-connectivity neighbors
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
                # Reconstruct
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

    def publish_visualization(self):
        # Planned path: green line + spheres at each waypoint, plus a red
        # sphere for the clicked goal so the user can see exactly where the
        # drone is supposed to end up vs where it actually is.
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clicked goal marker — show as soon as any click came in (even if
        # planning fails or is still running) so the user always sees where
        # they clicked. Big yellow sphere on the ground + red sphere at the
        # flight altitude they're targeting, plus a thin line connecting them.
        if self.phase not in (PP.WAITING_FOR_HOVER,) and (
                self.goal_x != 0.0 or self.goal_y != 0.0):
            # Ground marker at the actual click location (z=0)
            gm0 = Marker()
            gm0.header.frame_id = self.frame('map')
            gm0.header.stamp = now
            gm0.ns = 'clicked_ground'
            gm0.id = 0
            gm0.type = Marker.CYLINDER
            gm0.action = Marker.ADD
            gm0.pose.position.x = self.goal_x
            gm0.pose.position.y = self.goal_y
            gm0.pose.position.z = 0.05
            gm0.pose.orientation.w = 1.0
            gm0.scale = Vector3(x=0.8, y=0.8, z=0.1)
            gm0.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            arr.markers.append(gm0)

            # Flight-altitude target marker (red sphere)
            gm = Marker()
            gm.header.frame_id = self.frame('map')
            gm.header.stamp = now
            gm.ns = 'clicked_goal'
            gm.id = 0
            gm.type = Marker.SPHERE
            gm.action = Marker.ADD
            gm.pose.position.x = self.goal_x
            gm.pose.position.y = self.goal_y
            gm.pose.position.z = self.goal_z
            gm.pose.orientation.w = 1.0
            gm.scale = Vector3(x=0.5, y=0.5, z=0.5)
            gm.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            arr.markers.append(gm)

            # Vertical line connecting them, so the user can see the click
            # XY came from the ground and the drone target sits above it.
            line = Marker()
            line.header.frame_id = self.frame('map')
            line.header.stamp = now
            line.ns = 'clicked_pole'
            line.id = 0
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            line.pose.orientation.w = 1.0
            p1 = Point(x=self.goal_x, y=self.goal_y, z=0.1)
            p2 = Point(x=self.goal_x, y=self.goal_y, z=self.goal_z)
            line.points.extend([p1, p2])
            arr.markers.append(line)

        if self.planned_path:
            line = Marker()
            line.header.frame_id = self.frame('map')
            line.header.stamp = now
            line.ns = 'planned_path_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)
            for (x, y, z) in self.planned_path:
                line.points.append(Point(x=x, y=y, z=z))
            arr.markers.append(line)

            for i, (x, y, z) in enumerate(self.planned_path):
                sp = Marker()
                sp.header.frame_id = self.frame('map')
                sp.header.stamp = now
                sp.ns = 'planned_path_pts'
                sp.id = i
                sp.type = Marker.SPHERE
                sp.action = Marker.ADD
                sp.pose.position.x = x
                sp.pose.position.y = y
                sp.pose.position.z = z
                sp.pose.orientation.w = 1.0
                sp.scale = Vector3(x=0.25, y=0.25, z=0.25)
                if i == self.path_index and self.phase in (PP.SEND_WP, PP.WAIT_NAV, PP.WAIT_HOVER):
                    sp.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                elif i < self.path_index:
                    sp.color = ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.6)
                else:
                    sp.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)
                arr.markers.append(sp)
        self.path_marker_pub.publish(arr)

        # Grid obstacles (downsampled, every 3rd to keep RViz fast)
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
            sampled = list(self.occupied_cells)[::3]
            for cell in sampled:
                x, y, z = self.grid_to_world(cell)
                cube.points.append(Point(x=x, y=y, z=z))
            obs.markers.append(cube)
            self.grid_marker_pub.publish(obs)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
