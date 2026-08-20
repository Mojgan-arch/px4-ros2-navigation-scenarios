#!/usr/bin/env python3
"""
Telemetry Logger - records all key navigation data to a CSV file.

Subscribes to position, battery, nav state, geofence status, and goals.
Writes a row at 10Hz with the latest known values from each.

On shutdown (Ctrl+C), prints a summary (flight time, distance, max altitude,
min battery, state transition count).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, BatteryStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import csv
import math
from datetime import datetime
from ...common.interfaces import declare_interface_params


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== Output file ==========
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = f'/tmp/telemetry_{ts}.csv'
        self.summary_path = f'/tmp/telemetry_{ts}_summary.txt'

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'rel_time_s',
            'x', 'y', 'z',
            'vx', 'vy', 'vz', 'speed',
            'battery_pct',
            'nav_state', 'geofence_status',
            'goal_x', 'goal_y', 'goal_z',
        ])

        # ========== State (last known values) ==========
        self.start_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.position_received = False

        self.battery = 1.0
        self.nav_state = 'UNKNOWN'
        self.geofence_status = 'UNKNOWN'

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.has_goal = False

        # ========== Stats (for summary) ==========
        self.row_count = 0
        self.distance_traveled = 0.0
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.max_altitude = 0.0
        self.max_speed = 0.0
        self.min_battery = 1.0
        self.state_transitions = 0
        self.prev_nav_state = None
        self.state_history = []  # (rel_time, state)

        # ========== Subscribers ==========
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )
        self.create_subscription(
            BatteryStatus, self.topic('battery'),
            self.battery_callback, px4_qos
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.create_subscription(
            String, '/geofence_status', self.geofence_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        # ========== Timer: write row at 10 Hz ==========
        self.timer = self.create_timer(0.1, self.write_row)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TELEMETRY LOGGER active')
        self.get_logger().info(f'CSV file: {self.csv_path}')
        self.get_logger().info(f'Summary:  {self.summary_path}')
        self.get_logger().info('=' * 50)

    # =====================================================
    # Callbacks
    # =====================================================
    def position_callback(self, msg):
        # NED -> ENU
        self.x = msg.y
        self.y = msg.x
        self.z = -msg.z
        # PX4 publishes velocity in NED too
        self.vx = msg.vy
        self.vy = msg.vx
        self.vz = -msg.vz
        self.position_received = True

    def battery_callback(self, msg):
        self.battery = msg.remaining

    def nav_state_callback(self, msg):
        # Format: "STATE | bat=X%" -> take first token
        new_state = msg.data.split(' ')[0]
        if self.prev_nav_state is not None and new_state != self.prev_nav_state:
            self.state_transitions += 1
            rel = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.state_history.append((rel, new_state))
        self.prev_nav_state = new_state
        self.nav_state = new_state

    def geofence_callback(self, msg):
        # Take first word
        self.geofence_status = msg.data.split(' ')[0]

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z
        self.has_goal = True

    # =====================================================
    # Logger
    # =====================================================
    def write_row(self):
        if not self.position_received:
            return

        now = self.get_clock().now()
        rel = (now - self.start_time).nanoseconds / 1e9
        speed = math.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)

        # Update stats
        if self.last_x is not None:
            dx = self.x - self.last_x
            dy = self.y - self.last_y
            dz = self.z - self.last_z
            self.distance_traveled += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.last_x, self.last_y, self.last_z = self.x, self.y, self.z

        if self.z > self.max_altitude:
            self.max_altitude = self.z
        if speed > self.max_speed:
            self.max_speed = speed
        if self.battery < self.min_battery:
            self.min_battery = self.battery

        # Write row
        ts_str = datetime.now().isoformat()
        gx = self.goal_x if self.has_goal else ''
        gy = self.goal_y if self.has_goal else ''
        gz = self.goal_z if self.has_goal else ''

        self.csv_writer.writerow([
            ts_str, f'{rel:.3f}',
            f'{self.x:.3f}', f'{self.y:.3f}', f'{self.z:.3f}',
            f'{self.vx:.3f}', f'{self.vy:.3f}', f'{self.vz:.3f}',
            f'{speed:.3f}',
            f'{self.battery * 100:.1f}',
            self.nav_state, self.geofence_status,
            gx, gy, gz,
        ])
        self.csv_file.flush()
        self.row_count += 1

        if self.row_count % 50 == 0:  # every 5 seconds
            self.get_logger().info(
                f'Logged {self.row_count} rows | '
                f'time={rel:.1f}s | dist={self.distance_traveled:.1f}m | '
                f'bat={self.battery * 100:.0f}%'
            )

    # =====================================================
    # Summary on shutdown
    # =====================================================
    def write_summary(self):
        try:
            rel = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            with open(self.summary_path, 'w') as f:
                f.write('Telemetry Summary\n')
                f.write('=' * 50 + '\n')
                f.write(f'Date:               {datetime.now().isoformat()}\n')
                f.write(f'CSV file:           {self.csv_path}\n')
                f.write(f'Total flight time:  {rel:.1f} s\n')
                f.write(f'Distance traveled:  {self.distance_traveled:.2f} m\n')
                f.write(f'Max altitude:       {self.max_altitude:.2f} m\n')
                f.write(f'Max speed:          {self.max_speed:.2f} m/s\n')
                f.write(f'Min battery:        {self.min_battery * 100:.1f} %\n')
                f.write(f'State transitions:  {self.state_transitions}\n')
                f.write(f'Total rows logged:  {self.row_count}\n')
                f.write('-' * 50 + '\n')
                f.write('State history:\n')
                for (t, s) in self.state_history:
                    f.write(f'  t={t:7.2f}s  -> {s}\n')

            # Also print to console
            self.get_logger().info('=' * 50)
            self.get_logger().info('TELEMETRY SUMMARY')
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'Flight time:    {rel:.1f}s')
            self.get_logger().info(f'Distance:       {self.distance_traveled:.2f}m')
            self.get_logger().info(f'Max altitude:   {self.max_altitude:.2f}m')
            self.get_logger().info(f'Max speed:      {self.max_speed:.2f}m/s')
            self.get_logger().info(f'Min battery:    {self.min_battery * 100:.0f}%')
            self.get_logger().info(f'State changes:  {self.state_transitions}')
            self.get_logger().info(f'Files saved:')
            self.get_logger().info(f'  CSV:     {self.csv_path}')
            self.get_logger().info(f'  Summary: {self.summary_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write summary: {e}')

    def destroy_node(self):
        self.write_summary()
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
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
