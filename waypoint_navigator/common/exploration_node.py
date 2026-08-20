#!/usr/bin/env python3
"""
Autonomous Frontier-based Exploration (Stage 15).

Builds a 2D occupancy grid at a fixed altitude. Cells are:
  -1  UNKNOWN  (never observed)
   0  FREE     (observed and empty)
   1  OCCUPIED (observed and blocked)

Frontier cells are FREE cells adjacent to at least one UNKNOWN cell.
The drone repeatedly:
  1. Updates the map from current LiDAR scan (ray-casting).
  2. Detects frontiers and clusters them.
  3. Picks the best cluster (largest, weighted by inverse distance).
  4. Sends a goal to the state machine to navigate there.
  5. Repeats until no frontiers remain -> RTH and land.

Trigger:
  ros2 topic pub --once /start_exploration std_msgs/Bool "data: true"
  ros2 topic pub --once /stop_exploration  std_msgs/Bool "data: true"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import Bool, String, ColorRGBA
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from collections import deque
import math
import time
from .interfaces import declare_interface_params


class EP:
    """Exploration phases."""
    WAITING_FOR_HOVER = 'WAITING_FOR_HOVER'
    IDLE = 'IDLE'
    WARMUP = 'WARMUP'                # NEW: accumulate LiDAR before deciding
    BUILDING_MAP = 'BUILDING_MAP'
    FIND_FRONTIER = 'FIND_FRONTIER'
    SEND_GOAL = 'SEND_GOAL'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    NO_FRONTIERS = 'NO_FRONTIERS'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
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
        # Match the state machine's default_altitude (the altitude the C++
        # offboard node actually holds), so frontier goals are reachable.
        self.default_altitude = 2.5
        self.altitude = self.default_altitude
        # Bigger grid so we don't silently drop points when the PX4 local frame
        # is offset from the Gazebo origin (previous (-20,20) was too tight).
        self.grid_min = (-60.0, -60.0)
        self.grid_max = ( 60.0,  60.0)
        self.resolution = 0.5
        self.min_lidar_range = 0.5
        self.max_lidar_range = 12.0
        self.lidar_z_band = 5.0         # m - project points within this z range
        self.min_cluster_size = 4       # cells for valid frontier
        self.tolerance = 1.0            # m - "reached" frontier
        self.scan_settle_time = 2.0     # seconds to hover and scan before next plan
        self.hover_timeout = 30.0       # seconds before giving up on a frontier
        # WARMUP: build the map for at least this long before deciding whether
        # the room has any frontiers at all. Without this the very first
        # FIND_FRONTIER may run with an almost-empty grid (just one LiDAR
        # snapshot) and conclude "no frontiers" -> RTH -> immediate landing.
        self.warmup_seconds = 5.0
        self.warmup_start_time = None

        # Grid dimensions
        self.gx = int((self.grid_max[0] - self.grid_min[0]) / self.resolution)
        self.gy = int((self.grid_max[1] - self.grid_min[1]) / self.resolution)

        # ========== State ==========
        # Grid: dict (i, j) -> -1 / 0 / 1
        # Using dict so we don't allocate full 2D array; UNKNOWN is default
        self.grid = {}
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False
        # LiDAR points already transformed into the map frame
        self.lidar_points_map = []
        self.current_nav_state = 'UNKNOWN'

        # TF buffer so we can transform LiDAR points from sensor frame to map
        # frame properly (handles yaw/pitch/roll). Falls back to "raw points"
        # if TF isn't available — same pattern as path_planner_node.py.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.phase = EP.WAITING_FOR_HOVER
        self.exploring = False
        self.current_frontier = None  # (x, y, z) of current target
        self.frontier_clusters = []   # list of list of cells
        self.send_attempt_time = None
        self.settle_start_time = None
        self.iteration = 0

        # ========== Subscribers ==========
        # NOTE: PX4 v1.16+ versioned topic + sim's LiDAR topic.
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )
        self.create_subscription(
            PointCloud2, self.topic('pointcloud'),
            self.lidar_callback, lidar_qos
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.create_subscription(
            Bool, '/start_exploration', self.start_callback, 10
        )
        self.create_subscription(
            Bool, '/stop_exploration', self.stop_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.map_marker_pub = self.create_publisher(
            MarkerArray, '/exploration_map', 10
        )
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10
        )
        # Status string so the operator can `ros2 topic echo /exploration_status`
        # from another terminal and see what the node is doing, without having
        # to find pane 6 in Terminator.
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.viz_timer = self.create_timer(2.0, self.publish_visualization)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('AUTONOMOUS EXPLORATION (frontier-based)')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Altitude: {self.altitude}m')
        self.get_logger().info(
            f'Grid: X[{self.grid_min[0]}, {self.grid_max[0]}] '
            f'Y[{self.grid_min[1]}, {self.grid_max[1]}] '
            f'at {self.resolution}m'
        )
        self.get_logger().info('-' * 60)
        self.get_logger().info('Start exploration:')
        self.get_logger().info(
            '  ros2 topic pub --once /start_exploration '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('Stop exploration (RTH):')
        self.get_logger().info(
            '  ros2 topic pub --once /stop_exploration '
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
        self.get_logger().info(
            f'lidar_callback fired: position_received={self.position_received}, '
            f'frame_id={msg.header.frame_id!r}',
            throttle_duration_sec=2.0,
        )
        if not self.position_received:
            return

        # Resolve the sensor->map transform via TF so we don't have to guess
        # the LiDAR's body-frame convention (FLU vs FRD) or fight the yaw
        # sign. If TF is unavailable, fall back to treating the points as
        # already being in the map frame (with a throttled warn). Same
        # pattern as path_planner_node.py.
        frame_id = msg.header.frame_id or 'map'
        tx, ty, tz = 0.0, 0.0, 0.0
        # Identity rotation as default (used if TF lookup fails or frame is map)
        r00, r01, r02 = 1.0, 0.0, 0.0
        r10, r11, r12 = 0.0, 1.0, 0.0
        r20, r21, r22 = 0.0, 0.0, 1.0
        tf_ok = False
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
                tf_ok = True
            except Exception as e:
                self.get_logger().warn(
                    f'TF "{frame_id}" -> "map" unavailable, falling back to '
                    f'drone-position translation: {e}',
                    throttle_duration_sec=10.0,
                )
        # Fallback: when TF could not place the LiDAR sensor in the map
        # frame, treat the points as drone-local and translate by the
        # drone's current map-frame position. This rescues the common
        # case where the sim publishes LiDAR with frame_id=self.frame('map') (which
        # we then skip the lookup for) but the points are actually in a
        # sensor frame at the drone's origin.
        if not tf_ok:
            tx = self.current_x
            ty = self.current_y
            tz = self.current_z

        pts = []
        n_raw = 0
        n_kept_z = 0
        n_kept_range = 0
        try:
            points = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            for p in points:
                n_raw += 1
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                # Apply the homogeneous transform to get the map-frame point.
                wx = r00 * x + r01 * y + r02 * z + tx
                wy = r10 * x + r11 * y + r12 * z + ty
                wz = r20 * x + r21 * y + r22 * z + tz
                # Only use points roughly in the drone's z plane (map frame).
                if abs(wz - self.current_z) > self.lidar_z_band:
                    continue
                n_kept_z += 1
                # Range filter is relative to the drone, not the origin.
                dx = wx - self.current_x
                dy = wy - self.current_y
                d = math.sqrt(dx * dx + dy * dy)
                if self.min_lidar_range < d < self.max_lidar_range:
                    n_kept_range += 1
                    pts.append((wx, wy, d))
        except Exception as e:
            self.get_logger().warn(f'LiDAR error: {e}', throttle_duration_sec=5.0)
            return
        # Diagnostic: prove the filter path is healthy.
        self.get_logger().info(
            f'LiDAR filter: raw={n_raw} after_z={n_kept_z} '
            f'after_range={n_kept_range} | tf_ok={tf_ok} '
            f'tx={tx:.2f} ty={ty:.2f} tz={tz:.2f} '
            f'drone_z={self.current_z:.2f}',
            throttle_duration_sec=2.0,
        )
        self.lidar_points_map = pts

    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    def start_callback(self, msg):
        if not msg.data:
            return
        if self.phase != EP.IDLE:
            self.get_logger().warn(
                f'Cannot start now (phase={self.phase})'
            )
            return
        self.get_logger().info('>>> EXPLORATION STARTED <<<')
        self.get_logger().info(
            f'Warming up for {self.warmup_seconds:.0f}s — accumulating LiDAR '
            f'scans before the first frontier search.'
        )
        self.exploring = True
        self.iteration = 0
        self.warmup_start_time = time.time()
        self.set_phase(EP.WARMUP)

    def stop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('Exploration stop requested.')
        self.exploring = False
        if self.phase not in (EP.SEND_RTH, EP.WAIT_LANDED, EP.DONE):
            self.set_phase(EP.SEND_RTH)

    # =====================================================
    # Phase tick
    # =====================================================
    def tick(self):
        if self.phase == EP.WAITING_FOR_HOVER:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone hovering. Ready to explore.')
                self.set_phase(EP.IDLE)
                return

        elif self.phase == EP.WARMUP:
            if not self.exploring:
                self.set_phase(EP.SEND_RTH)
                return
            # Keep ingesting LiDAR scans into the grid while we wait. Without
            # this we'd jump to FIND_FRONTIER on a near-empty grid.
            self.update_map_from_lidar()
            elapsed = time.time() - self.warmup_start_time
            n_free = sum(1 for v in self.grid.values() if v == 0)
            n_occ = sum(1 for v in self.grid.values() if v == 1)
            self.get_logger().info(
                f'WARMUP: {elapsed:.1f}/{self.warmup_seconds:.0f}s | '
                f'free={n_free} occupied={n_occ}',
                throttle_duration_sec=1.0,
            )
            if elapsed >= self.warmup_seconds:
                self.get_logger().info(
                    f'Warmup done. Grid has {n_free} FREE and {n_occ} '
                    f'OCCUPIED cells. Searching for frontiers.'
                )
                self.set_phase(EP.BUILDING_MAP)

        elif self.phase == EP.BUILDING_MAP:
            if not self.exploring:
                self.set_phase(EP.SEND_RTH)
                return
            self.update_map_from_lidar()
            self.set_phase(EP.FIND_FRONTIER)

        elif self.phase == EP.FIND_FRONTIER:
            self.find_and_cluster_frontiers()
            if not self.frontier_clusters:
                self.get_logger().warn('No frontiers found - exploration complete')
                self.set_phase(EP.NO_FRONTIERS)
                return
            chosen = self.choose_best_frontier()
            if chosen is None:
                self.set_phase(EP.NO_FRONTIERS)
                return
            self.current_frontier = chosen
            self.set_phase(EP.SEND_GOAL)

        elif self.phase == EP.SEND_GOAL:
            x, y, z = self.current_frontier
            self.iteration += 1
            self.get_logger().info(
                f'[Iter {self.iteration}] Frontier goal: ({x:.2f}, {y:.2f})'
            )
            self.send_goal(x, y, z)
            self.send_attempt_time = time.time()
            self.set_phase(EP.WAIT_NAV)

        elif self.phase == EP.WAIT_NAV:
            # Accept either NAVIGATING or HOVER — if the drone already reached
            # the frontier quickly we may have skipped past NAVIGATING entirely
            # between the 0.5 Hz ticks.
            if self.current_nav_state in ('NAVIGATING', 'HOVER'):
                self.set_phase(EP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > 3.0:
                self.set_phase(EP.SEND_GOAL)

        elif self.phase == EP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                if self.settle_start_time is None:
                    self.settle_start_time = time.time()
                elapsed = time.time() - self.settle_start_time
                if elapsed >= self.scan_settle_time:
                    self.settle_start_time = None
                    self.set_phase(EP.BUILDING_MAP)
            elif (self.send_attempt_time is not None
                  and time.time() - self.send_attempt_time > self.hover_timeout):
                # The drone can't reach this frontier (stuck against a wall or
                # oscillating in a narrow corridor). Skip it and try a new one
                # so the mission never hangs.
                self.get_logger().warn(
                    f'Frontier #{self.iteration} timeout '
                    f'({self.hover_timeout:.0f}s) — advancing to next frontier'
                )
                self.settle_start_time = None
                self.current_frontier = None
                self.set_phase(EP.BUILDING_MAP)

        elif self.phase == EP.NO_FRONTIERS:
            self.exploring = False
            self.set_phase(EP.SEND_RTH)

        elif self.phase == EP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info(
                f'Exploration done after {self.iteration} iterations. RTH.'
            )
            self.set_phase(EP.WAIT_LANDED)

        elif self.phase == EP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.set_phase(EP.DONE)

        elif self.phase == EP.DONE:
            self.get_logger().info(
                'Exploration complete. Mission done.',
                throttle_duration_sec=5.0
            )

    def set_phase(self, new_phase):
        if new_phase != self.phase:
            self.get_logger().info(f'EXPLORE: {self.phase} -> {new_phase}')
            self.phase = new_phase

    # =====================================================
    # Grid + ray casting
    # =====================================================
    def world_to_cell(self, x, y):
        i = int((x - self.grid_min[0]) / self.resolution)
        j = int((y - self.grid_min[1]) / self.resolution)
        if 0 <= i < self.gx and 0 <= j < self.gy:
            return (i, j)
        return None

    def cell_to_world(self, cell):
        i, j = cell
        x = self.grid_min[0] + (i + 0.5) * self.resolution
        y = self.grid_min[1] + (j + 0.5) * self.resolution
        return (x, y)

    def get_cell(self, cell):
        return self.grid.get(cell, -1)

    def set_cell(self, cell, value):
        # OCCUPIED is sticky - don't overwrite with FREE
        existing = self.grid.get(cell, -1)
        if existing == 1 and value == 0:
            return
        self.grid[cell] = value

    def ray_cast_cells(self, x0, y0, x1, y1):
        """Bresenham-style integer line from cell (x0,y0) to (x1,y1)."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def update_map_from_lidar(self):
        if not self.position_received:
            self.get_logger().warn(
                'update_map_from_lidar: position not received yet',
                throttle_duration_sec=3.0,
            )
            return
        if not self.lidar_points_map:
            self.get_logger().warn(
                'update_map_from_lidar: lidar_points_map is empty',
                throttle_duration_sec=3.0,
            )
            return

        drone_cell = self.world_to_cell(self.current_x, self.current_y)
        if drone_cell is None:
            self.get_logger().error(
                f'update_map_from_lidar: drone outside grid! '
                f'drone=({self.current_x:.1f}, {self.current_y:.1f}) '
                f'grid_x=[{self.grid_min[0]}, {self.grid_max[0]}] '
                f'grid_y=[{self.grid_min[1]}, {self.grid_max[1]}]',
                throttle_duration_sec=3.0,
            )
            return

        # One-time diagnostic: prove cells are actually being written.
        if not getattr(self, '_diag_logged', False):
            sample = self.lidar_points_map[: min(3, len(self.lidar_points_map))]
            self.get_logger().info(
                f'DIAG: drone=({self.current_x:.2f}, {self.current_y:.2f}) '
                f'drone_cell={drone_cell} '
                f'first_pts={sample} '
                f'total_pts={len(self.lidar_points_map)}'
            )
            self._diag_logged = True

        skipped_outside = 0
        for (wx, wy, d) in self.lidar_points_map:
            # Points are already in map frame (TF-transformed in callback).
            target_cell = self.world_to_cell(wx, wy)
            if target_cell is None:
                skipped_outside += 1
                continue
            ray = self.ray_cast_cells(
                drone_cell[0], drone_cell[1],
                target_cell[0], target_cell[1]
            )
            # All cells except last -> FREE
            for c in ray[:-1]:
                self.set_cell(c, 0)
            # Last cell: OCCUPIED if it's a real hit, FREE if at max range
            if d >= self.max_lidar_range - 0.5:
                self.set_cell(ray[-1], 0)
            else:
                self.set_cell(ray[-1], 1)

        if skipped_outside > 0 and skipped_outside == len(self.lidar_points_map):
            self.get_logger().error(
                f'update_map_from_lidar: ALL {skipped_outside} LiDAR points '
                f'mapped to cells OUTSIDE the grid! Increase grid bounds.',
                throttle_duration_sec=3.0,
            )

    # =====================================================
    # Frontier detection
    # =====================================================
    def find_and_cluster_frontiers(self):
        # Find all frontier cells: FREE cells adjacent to UNKNOWN
        frontiers = set()
        for cell, value in self.grid.items():
            if value != 0:
                continue
            i, j = cell
            for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (i + di, j + dj)
                if not (0 <= neighbor[0] < self.gx and 0 <= neighbor[1] < self.gy):
                    continue
                if self.get_cell(neighbor) == -1:
                    frontiers.add(cell)
                    break

        # Cluster via BFS
        visited = set()
        clusters = []
        for f in frontiers:
            if f in visited:
                continue
            cluster = []
            queue = deque([f])
            visited.add(f)
            while queue:
                c = queue.popleft()
                cluster.append(c)
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        if di == 0 and dj == 0:
                            continue
                        n = (c[0] + di, c[1] + dj)
                        if n in frontiers and n not in visited:
                            visited.add(n)
                            queue.append(n)
            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)
        self.frontier_clusters = clusters
        self.get_logger().info(
            f'Frontiers: {len(frontiers)} cells, {len(clusters)} clusters'
        )

    def choose_best_frontier(self):
        """Best = largest cluster scored by size / (1 + distance)."""
        if not self.frontier_clusters:
            return None

        best_score = -math.inf
        best_centroid = None

        for cluster in self.frontier_clusters:
            # Centroid in grid coords
            ci = sum(c[0] for c in cluster) / len(cluster)
            cj = sum(c[1] for c in cluster) / len(cluster)
            wx, wy = self.cell_to_world((int(ci), int(cj)))
            dist = math.sqrt(
                (wx - self.current_x) ** 2 + (wy - self.current_y) ** 2
            )
            score = len(cluster) / (1.0 + dist)
            if score > best_score:
                best_score = score
                best_centroid = (wx, wy, self.altitude)
        return best_centroid

    # =====================================================
    # Helpers
    # =====================================================
    def publish_status(self):
        """Concise human-readable status so the operator can echo
        /exploration_status from any terminal and follow what's happening."""
        n_free = sum(1 for v in self.grid.values() if v == 0)
        n_occ = sum(1 for v in self.grid.values() if v == 1)
        n_clusters = len(self.frontier_clusters)
        n_frontiers = sum(len(c) for c in self.frontier_clusters)
        msg = String()
        bits = [
            f'phase={self.phase}',
            f'iter={self.iteration}',
            f'free={n_free}',
            f'occ={n_occ}',
            f'frontiers={n_frontiers}',
            f'clusters={n_clusters}',
            f'nav={self.current_nav_state}',
            f'lidar_pts={len(self.lidar_points_map)}',
        ]
        if self.current_frontier is not None:
            cx, cy, _ = self.current_frontier
            bits.append(f'goal=({cx:.1f},{cy:.1f})')
        msg.data = ' | '.join(bits)
        self.status_pub.publish(msg)

    def send_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    # =====================================================
    # Visualization
    # =====================================================
    def publish_visualization(self):
        now = self.get_clock().now().to_msg()

        # ----- Map markers (FREE: green, OCCUPIED: red) -----
        map_arr = MarkerArray()

        free_cube = Marker()
        free_cube.header.frame_id = self.frame('map')
        free_cube.header.stamp = now
        free_cube.ns = 'map_free'
        free_cube.id = 0
        free_cube.type = Marker.CUBE_LIST
        free_cube.action = Marker.ADD
        free_cube.scale = Vector3(
            x=self.resolution, y=self.resolution, z=0.05
        )
        free_cube.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.25)

        occ_cube = Marker()
        occ_cube.header.frame_id = self.frame('map')
        occ_cube.header.stamp = now
        occ_cube.ns = 'map_occupied'
        occ_cube.id = 0
        occ_cube.type = Marker.CUBE_LIST
        occ_cube.action = Marker.ADD
        occ_cube.scale = Vector3(
            x=self.resolution, y=self.resolution, z=0.3
        )
        occ_cube.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.7)

        for cell, value in self.grid.items():
            x, y = self.cell_to_world(cell)
            p = Point(x=x, y=y, z=self.altitude - 1.0)
            if value == 0:
                free_cube.points.append(p)
            elif value == 1:
                occ_cube.points.append(Point(x=x, y=y, z=self.altitude))

        map_arr.markers.append(free_cube)
        map_arr.markers.append(occ_cube)
        self.map_marker_pub.publish(map_arr)

        # ----- Frontier markers (cyan) + chosen target (orange) -----
        fr_arr = MarkerArray()
        if self.frontier_clusters:
            cluster_cube = Marker()
            cluster_cube.header.frame_id = self.frame('map')
            cluster_cube.header.stamp = now
            cluster_cube.ns = 'frontier_cells'
            cluster_cube.id = 0
            cluster_cube.type = Marker.CUBE_LIST
            cluster_cube.action = Marker.ADD
            cluster_cube.scale = Vector3(
                x=self.resolution, y=self.resolution, z=0.4
            )
            cluster_cube.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=0.7)
            for cluster in self.frontier_clusters:
                for cell in cluster:
                    x, y = self.cell_to_world(cell)
                    cluster_cube.points.append(
                        Point(x=x, y=y, z=self.altitude)
                    )
            fr_arr.markers.append(cluster_cube)

        if self.current_frontier is not None:
            target = Marker()
            target.header.frame_id = self.frame('map')
            target.header.stamp = now
            target.ns = 'frontier_target'
            target.id = 0
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = self.current_frontier[0]
            target.pose.position.y = self.current_frontier[1]
            target.pose.position.z = self.current_frontier[2]
            target.pose.orientation.w = 1.0
            target.scale = Vector3(x=0.8, y=0.8, z=0.8)
            target.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            fr_arr.markers.append(target)

            label = Marker()
            label.header.frame_id = self.frame('map')
            label.header.stamp = now
            label.ns = 'frontier_target_label'
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = self.current_frontier[0]
            label.pose.position.y = self.current_frontier[1]
            label.pose.position.z = self.current_frontier[2] + 1.0
            label.scale.z = 0.5
            label.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            label.text = f'FRONTIER #{self.iteration}'
            fr_arr.markers.append(label)

        self.frontier_marker_pub.publish(fr_arr)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
