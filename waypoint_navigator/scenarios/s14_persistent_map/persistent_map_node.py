#!/usr/bin/env python3
"""
Persistent SLAM Map (Stage 16).

Builds a 2D occupancy grid from LiDAR (continuously), and provides
save/load to a JSON file so the map survives between flights.

Topics:
  Input:
    /livox/lidar                       LiDAR point cloud
    /fmu/out/vehicle_local_position   drone position
    /save_map  (Bool true)             save current map to disk
    /load_map  (Bool true)             load map from disk
    /clear_map (Bool true)             reset to empty
  Output:
    /persistent_map (MarkerArray)      visualization for RViz
    /map_info       (String)           status (cells, last save, etc.)

Default file: /tmp/persistent_map.json
Auto-save: every 30 seconds (if map changed)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String, ColorRGBA
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

import json
import math
import os
from datetime import datetime
from ...common.interfaces import declare_interface_params


class PersistentMapNode(Node):
    def __init__(self):
        super().__init__('persistent_map')
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
        self.altitude = 3.0
        self.grid_min = (-20.0, -20.0)
        self.grid_max = ( 20.0,  20.0)
        self.resolution = 0.5
        self.min_lidar_range = 0.5
        self.max_lidar_range = 12.0
        self.lidar_z_band = 1.5

        self.map_file = '/tmp/persistent_map.json'
        self.auto_save_interval = 30.0  # seconds

        # Grid dimensions
        self.gx = int((self.grid_max[0] - self.grid_min[0]) / self.resolution)
        self.gy = int((self.grid_max[1] - self.grid_min[1]) / self.resolution)

        # ========== State ==========
        self.grid = {}     # (i, j) -> 0/1, missing = -1 unknown
        self.dirty = False  # changed since last save?
        self.last_save_time = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.position_received = False
        self.lidar_points_body = []
        self.cells_seen_this_session = 0

        # ========== Subscribers ==========
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )
        self.create_subscription(
            PointCloud2, self.topic('pointcloud'), self.lidar_callback, lidar_qos
        )
        self.create_subscription(Bool, '/save_map', self.save_callback, 10)
        self.create_subscription(Bool, '/load_map', self.load_callback, 10)
        self.create_subscription(Bool, '/clear_map', self.clear_callback, 10)

        # ========== Publishers ==========
        self.marker_pub = self.create_publisher(
            MarkerArray, '/persistent_map', 10
        )
        self.info_pub = self.create_publisher(String, '/map_info', 10)

        # ========== Timers ==========
        self.update_timer = self.create_timer(0.5, self.update_map_from_lidar)
        self.viz_timer = self.create_timer(2.0, self.publish_visualization)
        self.info_timer = self.create_timer(2.0, self.publish_info)
        self.auto_save_timer = self.create_timer(
            self.auto_save_interval, self.auto_save
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('PERSISTENT SLAM MAP')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'File: {self.map_file}')
        self.get_logger().info(f'Auto-save: every {self.auto_save_interval}s')
        self.get_logger().info(
            f'Grid: X[{self.grid_min[0]}, {self.grid_max[0]}] '
            f'Y[{self.grid_min[1]}, {self.grid_max[1]}] '
            f'@ {self.resolution}m'
        )
        self.get_logger().info('-' * 60)
        self.get_logger().info(
            'Save:  ros2 topic pub --once /save_map  std_msgs/Bool "data: true"'
        )
        self.get_logger().info(
            'Load:  ros2 topic pub --once /load_map  std_msgs/Bool "data: true"'
        )
        self.get_logger().info(
            'Clear: ros2 topic pub --once /clear_map std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 60)

        # Try to auto-load on startup
        if os.path.exists(self.map_file):
            self.get_logger().info('Existing map file found - auto-loading')
            self.load_map_from_disk()

    # =====================================================
    # Subscribers
    # =====================================================
    def position_callback(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
        self.position_received = True

    def lidar_callback(self, msg):
        pts = []
        try:
            points = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            for p in points:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if abs(z) > self.lidar_z_band:
                    continue
                d = math.sqrt(x * x + y * y)
                if self.min_lidar_range < d < self.max_lidar_range:
                    pts.append((x, y, d))
        except Exception as e:
            self.get_logger().warn(f'LiDAR error: {e}', throttle_duration_sec=5.0)
            return
        self.lidar_points_body = pts

    def save_callback(self, msg):
        if not msg.data:
            return
        self.save_map_to_disk()

    def load_callback(self, msg):
        if not msg.data:
            return
        self.load_map_from_disk()

    def clear_callback(self, msg):
        if not msg.data:
            return
        self.grid = {}
        self.dirty = True
        self.cells_seen_this_session = 0
        self.get_logger().info('Map cleared.')

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

    def set_cell(self, cell, value):
        existing = self.grid.get(cell, -1)
        if existing == value:
            return
        if existing == 1 and value == 0:
            return  # OCCUPIED is sticky
        self.grid[cell] = value
        self.dirty = True
        self.cells_seen_this_session += 1

    def ray_cast_cells(self, x0, y0, x1, y1):
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
            return
        if not self.lidar_points_body:
            return

        drone_cell = self.world_to_cell(self.current_x, self.current_y)
        if drone_cell is None:
            return

        for (bx, by, d) in self.lidar_points_body:
            wx = self.current_x + bx
            wy = self.current_y + by
            target_cell = self.world_to_cell(wx, wy)
            if target_cell is None:
                continue
            ray = self.ray_cast_cells(
                drone_cell[0], drone_cell[1],
                target_cell[0], target_cell[1]
            )
            for c in ray[:-1]:
                self.set_cell(c, 0)
            if d >= self.max_lidar_range - 0.5:
                self.set_cell(ray[-1], 0)
            else:
                self.set_cell(ray[-1], 1)

    # =====================================================
    # Save / Load
    # =====================================================
    def save_map_to_disk(self):
        try:
            data = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'grid_min': list(self.grid_min),
                    'grid_max': list(self.grid_max),
                    'resolution': self.resolution,
                    'cell_count': len(self.grid),
                },
                'cells': [
                    [int(i), int(j), int(v)]
                    for (i, j), v in self.grid.items()
                ],
            }
            with open(self.map_file, 'w') as f:
                json.dump(data, f)
            self.dirty = False
            self.last_save_time = datetime.now()
            self.get_logger().info(
                f'>>> Map saved: {len(self.grid)} cells -> {self.map_file}'
            )
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')

    def load_map_from_disk(self):
        if not os.path.exists(self.map_file):
            self.get_logger().error(f'File not found: {self.map_file}')
            return
        try:
            with open(self.map_file, 'r') as f:
                data = json.load(f)
            meta = data.get('metadata', {})
            # Compatibility check
            if (meta.get('resolution') != self.resolution or
                    meta.get('grid_min') != list(self.grid_min) or
                    meta.get('grid_max') != list(self.grid_max)):
                self.get_logger().warn(
                    'Loaded map params differ from current - loading anyway'
                )
            self.grid = {
                (int(c[0]), int(c[1])): int(c[2])
                for c in data.get('cells', [])
            }
            self.dirty = False
            self.get_logger().info(
                f'<<< Map loaded: {len(self.grid)} cells from {self.map_file}'
            )
            self.get_logger().info(
                f'    Saved at: {meta.get("timestamp", "unknown")}'
            )
        except Exception as e:
            self.get_logger().error(f'Load failed: {e}')

    def auto_save(self):
        if self.dirty and len(self.grid) > 0:
            self.get_logger().info('Auto-save (map changed)')
            self.save_map_to_disk()

    # =====================================================
    # Visualization & info
    # =====================================================
    def publish_info(self):
        free = sum(1 for v in self.grid.values() if v == 0)
        occ = sum(1 for v in self.grid.values() if v == 1)
        last = self.last_save_time.strftime('%H:%M:%S') if self.last_save_time else 'never'
        info = (
            f'cells={len(self.grid)} | free={free} occ={occ} | '
            f'dirty={self.dirty} | last_save={last}'
        )
        msg = String()
        msg.data = info
        self.info_pub.publish(msg)

    def publish_visualization(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        free_cube = Marker()
        free_cube.header.frame_id = self.frame('map')
        free_cube.header.stamp = now
        free_cube.ns = 'pmap_free'
        free_cube.id = 0
        free_cube.type = Marker.CUBE_LIST
        free_cube.action = Marker.ADD
        free_cube.scale = Vector3(
            x=self.resolution, y=self.resolution, z=0.05
        )
        free_cube.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.25)

        occ_cube = Marker()
        occ_cube.header.frame_id = self.frame('map')
        occ_cube.header.stamp = now
        occ_cube.ns = 'pmap_occupied'
        occ_cube.id = 0
        occ_cube.type = Marker.CUBE_LIST
        occ_cube.action = Marker.ADD
        occ_cube.scale = Vector3(
            x=self.resolution, y=self.resolution, z=0.4
        )
        occ_cube.color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.7)

        for cell, value in self.grid.items():
            x, y = self.cell_to_world(cell)
            if value == 0:
                free_cube.points.append(
                    Point(x=x, y=y, z=self.altitude - 2.0)
                )
            elif value == 1:
                occ_cube.points.append(Point(x=x, y=y, z=self.altitude))

        arr.markers.append(free_cube)
        arr.markers.append(occ_cube)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PersistentMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save on shutdown
        if node.dirty and len(node.grid) > 0:
            node.get_logger().info('Saving map before shutdown...')
            node.save_map_to_disk()
        node.destroy_node()
        # Ctrl+C already shuts the context down via rclpy's signal
        # handler, so an unguarded shutdown() raises on ROS 2 Jazzy.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
