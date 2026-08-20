#!/usr/bin/env python3
"""
Inspection Mission Node - high-level mission controller.

Sits on top of the navigation state machine. Sends inspection waypoints
one by one, waits at each point to 'capture', then triggers RTH at the end.

Architecture:
  This node publishes goals to /goal_pose (consumed by state_machine_node)
  and triggers /rth_trigger when mission is complete.

Mission flow:
  WAITING_FOR_TAKEOFF -> SEND_GOAL -> WAIT_NAVIGATING -> WAIT_HOVER
  -> SETTLE -> CAPTURE -> (next point or RTH)
  -> SEND_RTH -> WAIT_LANDED -> DONE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

import time
import os
from datetime import datetime
from ...common.interfaces import declare_interface_params


class MP:
    """Mission phase constants."""
    WAITING_FOR_TAKEOFF = 'WAITING_FOR_TAKEOFF'
    SEND_GOAL = 'SEND_GOAL'
    WAIT_NAVIGATING = 'WAIT_NAVIGATING'
    WAIT_HOVER = 'WAIT_HOVER'
    SETTLE = 'SETTLE'
    CAPTURE = 'CAPTURE'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'


class InspectionMission(Node):
    def __init__(self):
        super().__init__('inspection_mission')
        declare_interface_params(self)

        # ========== Mission configuration ==========
        self.target_name = 'Mock Wind Turbine'
        self.settle_time = 3.0          # seconds to hover at each point
        self.goal_send_timeout = 3.0    # seconds to wait for state machine ack
        self.results_file = f'/tmp/inspection_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'

        # Inspection points: (x, y, z, label)
        # Octagon (radius 2.5 m) at altitude 2.5 m around the origin. The room
        # walls are at +/-5 m, so radius 2.5 keeps the drone clear of the walls.
        # z = 2.5 matches the state machine's default_altitude (the altitude the
        # C++ offboard node actually holds), so points are reachable.
        self.inspection_points = [
            (2.5, 0.0, 2.5, 'East Face'),
            (1.8, 1.8, 2.5, 'North-East Corner'),
            (0.0, 2.5, 2.5, 'North Face'),
            (-1.8, 1.8, 2.5, 'North-West Corner'),
            (-2.5, 0.0, 2.5, 'West Face'),
            (-1.8, -1.8, 2.5, 'South-West Corner'),
            (0.0, -2.5, 2.5, 'South Face'),
            (1.8, -1.8, 2.5, 'South-East Corner'),
        ]

        # ========== State ==========
        self.current_nav_state = 'UNKNOWN'
        self.mission_phase = MP.WAITING_FOR_TAKEOFF
        self.mission_index = 0
        self.settle_start_time = None
        self.send_attempt_time = None
        self.captures = []

        # ========== Subscribers ==========
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)

        # ========== Timer ==========
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info('=' * 50)
        self.get_logger().info(f'INSPECTION MISSION: {self.target_name}')
        self.get_logger().info(f'Total inspection points: {len(self.inspection_points)}')
        self.get_logger().info(f'Settle time per point: {self.settle_time}s')
        self.get_logger().info(f'Results log: {self.results_file}')
        self.get_logger().info('=' * 50)

    # =====================================================
    # Subscribers
    # =====================================================
    def nav_state_callback(self, msg):
        # Format: "STATE | bat=X%"  -> take first token
        self.current_nav_state = msg.data.split(' ')[0]

    # =====================================================
    # Mission phase machine
    # =====================================================
    def tick(self):
        phase = self.mission_phase

        if phase == MP.WAITING_FOR_TAKEOFF:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone ready (HOVER). Starting mission.')
                self.set_phase(MP.SEND_GOAL)
            else:
                self.get_logger().info(
                    f'Waiting for takeoff... current state: {self.current_nav_state}',
                    throttle_duration_sec=2.0
                )

        elif phase == MP.SEND_GOAL:
            if self.mission_index >= len(self.inspection_points):
                self.get_logger().info('All inspection points visited.')
                self.set_phase(MP.SEND_RTH)
                return
            self.send_current_goal()
            self.send_attempt_time = time.time()
            self.set_phase(MP.WAIT_NAVIGATING)

        elif phase == MP.WAIT_NAVIGATING:
            if self.current_nav_state == 'NAVIGATING':
                self.set_phase(MP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.get_logger().warn('Goal not acknowledged - resending')
                self.set_phase(MP.SEND_GOAL)

        elif phase == MP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                _, _, _, label = self.inspection_points[self.mission_index]
                self.get_logger().info(
                    f'Reached point {self.mission_index + 1}/'
                    f'{len(self.inspection_points)}: {label}'
                )
                self.settle_start_time = time.time()
                self.set_phase(MP.SETTLE)

        elif phase == MP.SETTLE:
            elapsed = time.time() - self.settle_start_time
            remaining = self.settle_time - elapsed
            if remaining <= 0:
                self.set_phase(MP.CAPTURE)
            else:
                self.get_logger().info(
                    f'Settling: {remaining:.1f}s remaining',
                    throttle_duration_sec=1.0
                )

        elif phase == MP.CAPTURE:
            self.do_capture()
            self.mission_index += 1
            self.set_phase(MP.SEND_GOAL)

        elif phase == MP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info('RTH triggered - returning home')
            self.set_phase(MP.WAIT_LANDED)

        elif phase == MP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.write_summary()
                self.set_phase(MP.DONE)
            else:
                self.get_logger().info(
                    f'Waiting for landing... state: {self.current_nav_state}',
                    throttle_duration_sec=3.0
                )

        elif phase == MP.DONE:
            self.get_logger().info(
                'Mission complete. Press Ctrl+C to exit.',
                throttle_duration_sec=5.0
            )

    # =====================================================
    # Helpers
    # =====================================================
    def set_phase(self, new_phase):
        if new_phase != self.mission_phase:
            self.get_logger().info(
                f'MISSION: {self.mission_phase} -> {new_phase}'
            )
            self.mission_phase = new_phase

    def send_current_goal(self):
        x, y, z, label = self.inspection_points[self.mission_index]
        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'>>> Sending goal {self.mission_index + 1}/'
            f'{len(self.inspection_points)}: {label} ({x}, {y}, {z})'
        )

    def do_capture(self):
        x, y, z, label = self.inspection_points[self.mission_index]
        capture = {
            'index': self.mission_index + 1,
            'label': label,
            'x': x, 'y': y, 'z': z,
            'timestamp': datetime.now().isoformat(),
        }
        self.captures.append(capture)
        self.get_logger().info(
            f'[CAPTURE #{capture["index"]}] {label} @ ({x}, {y}, {z}) - logged'
        )

    def write_summary(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('INSPECTION SUMMARY')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Target: {self.target_name}')
        self.get_logger().info(f'Total captures: {len(self.captures)}')

        try:
            with open(self.results_file, 'w') as f:
                f.write(f'Inspection Mission Report\n')
                f.write(f'Target: {self.target_name}\n')
                f.write(f'Date: {datetime.now().isoformat()}\n')
                f.write(f'Total points: {len(self.captures)}\n')
                f.write('=' * 50 + '\n')
                for c in self.captures:
                    f.write(
                        f'#{c["index"]:2d} | {c["label"]:25s} | '
                        f'({c["x"]:6.2f}, {c["y"]:6.2f}, {c["z"]:6.2f}) | '
                        f'{c["timestamp"]}\n'
                    )
            self.get_logger().info(f'Report written to: {self.results_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to write report: {e}')

        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionMission()
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
