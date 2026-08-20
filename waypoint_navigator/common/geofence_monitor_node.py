#!/usr/bin/env python3
"""
Geofence Monitor - Safety supervisor.

Defines a 3D box in space. The drone is only allowed inside.
If the drone exits, automatically triggers RTH via /rth_trigger.

Also publishes a visualization marker so the geofence shows up in RViz,
and a status string for monitoring.

Architecture:
  This is an independent SAFETY node. It does not control the drone
  directly - only triggers RTH on the state machine if a violation occurs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from .interfaces import declare_interface_params


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== Geofence box (ENU frame) ==========
        # Edit these to define your allowed flight zone
        self.min_x = -8.0    # West boundary
        self.max_x =  8.0    # East boundary
        self.min_y = -8.0    # South boundary
        self.max_y =  8.0    # North boundary
        self.min_z =  0.2    # Floor (slightly above ground)
        self.max_z =  6.0    # Ceiling

        # Warning margin: start warning when drone is within this distance
        # of any boundary
        self.warning_margin = 1.0

        # ========== State ==========
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False
        self.violated = False        # latched - only triggers RTH once

        # ========== Subscribers ==========
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )

        # ========== Publishers ==========
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.status_pub = self.create_publisher(String, '/geofence_status', 10)
        self.marker_pub = self.create_publisher(Marker, '/geofence_marker', 10)

        # ========== Timers ==========
        # Check position 10Hz
        self.check_timer = self.create_timer(0.1, self.check_geofence)
        # Publish status & marker 1Hz
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('=' * 50)
        self.get_logger().info('GEOFENCE MONITOR active')
        self.get_logger().info(
            f'Allowed zone: '
            f'X[{self.min_x}, {self.max_x}]m  '
            f'Y[{self.min_y}, {self.max_y}]m  '
            f'Z[{self.min_z}, {self.max_z}]m'
        )
        self.get_logger().info(f'Warning margin: {self.warning_margin}m')
        self.get_logger().info('=' * 50)

    # =====================================================
    # Subscribers
    # =====================================================
    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    # =====================================================
    # Geofence logic
    # =====================================================
    def is_outside(self, x, y, z):
        return (x < self.min_x or x > self.max_x or
                y < self.min_y or y > self.max_y or
                z < self.min_z or z > self.max_z)

    def distance_to_boundary(self, x, y, z):
        """Return the smallest distance to any of the 6 walls."""
        return min(
            x - self.min_x, self.max_x - x,
            y - self.min_y, self.max_y - y,
            z - self.min_z, self.max_z - z,
        )

    def check_geofence(self):
        if not self.position_received:
            return

        x, y, z = self.current_x, self.current_y, self.current_z

        if self.is_outside(x, y, z):
            if not self.violated:
                self.violated = True
                self.get_logger().error(
                    f'!!! GEOFENCE VIOLATION !!! '
                    f'Position ({x:.2f}, {y:.2f}, {z:.2f}) is outside box. '
                    f'TRIGGERING RTH.'
                )
                rth = Bool()
                rth.data = True
                self.rth_pub.publish(rth)
            else:
                # Keep publishing the RTH trigger periodically until inside
                rth = Bool()
                rth.data = True
                self.rth_pub.publish(rth)
        else:
            # Inside fence - reset violation latch
            if self.violated:
                self.get_logger().info('Back inside geofence. Latch reset.')
                self.violated = False

            # Warning if close to boundary
            d = self.distance_to_boundary(x, y, z)
            if d < self.warning_margin:
                self.get_logger().warn(
                    f'Near boundary (distance: {d:.2f}m)',
                    throttle_duration_sec=1.0
                )

    # =====================================================
    # Visualization & status
    # =====================================================
    def publish_visualization(self):
        # Publish status string
        if not self.position_received:
            status = 'NO_POSITION'
        elif self.violated:
            status = 'VIOLATION'
        else:
            d = self.distance_to_boundary(
                self.current_x, self.current_y, self.current_z
            )
            status = 'WARNING' if d < self.warning_margin else 'OK'
            status += f' | dist_to_boundary={d:.2f}m'
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

        # Publish wireframe box marker for RViz
        marker = Marker()
        marker.header.frame_id = self.frame('map')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'geofence'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.05, y=0.0, z=0.0)

        # Color: green if OK, yellow if warning, red if violated
        if self.violated:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        elif (self.position_received and self.distance_to_boundary(
                self.current_x, self.current_y, self.current_z
              ) < self.warning_margin):
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        else:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)

        # 8 corners of the box
        x0, x1 = self.min_x, self.max_x
        y0, y1 = self.min_y, self.max_y
        z0, z1 = self.min_z, self.max_z
        corners = [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # bottom
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # top
        ]
        # 12 edges (pairs of corner indices)
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # bottom square
            (4, 5), (5, 6), (6, 7), (7, 4),  # top square
            (0, 4), (1, 5), (2, 6), (3, 7),  # vertical edges
        ]
        for a, b in edges:
            ax, ay, az = corners[a]
            bx, by, bz = corners[b]
            marker.points.append(Point(x=float(ax), y=float(ay), z=float(az)))
            marker.points.append(Point(x=float(bx), y=float(by), z=float(bz)))

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
