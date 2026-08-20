#!/usr/bin/env python3
"""
Indoor Delivery Mission - high-level mission controller.

For each delivery: go to pickup -> grab package -> go to dropoff -> deliver.
After all deliveries, RTH and land.

Architecture (same as inspection_mission):
  publishes /goal_pose, subscribes /nav_state, triggers /rth_trigger
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

import time
from datetime import datetime
from ...common.interfaces import declare_interface_params


class MP:
    """Mission phase constants."""
    WAITING_FOR_TAKEOFF = 'WAITING_FOR_TAKEOFF'
    GO_TO_PICKUP = 'GO_TO_PICKUP'
    WAIT_NAV_PICKUP = 'WAIT_NAV_PICKUP'
    WAIT_HOVER_PICKUP = 'WAIT_HOVER_PICKUP'
    PICKING_UP = 'PICKING_UP'
    GO_TO_DROPOFF = 'GO_TO_DROPOFF'
    WAIT_NAV_DROPOFF = 'WAIT_NAV_DROPOFF'
    WAIT_HOVER_DROPOFF = 'WAIT_HOVER_DROPOFF'
    DROPPING_OFF = 'DROPPING_OFF'
    NEXT_DELIVERY = 'NEXT_DELIVERY'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'


class DeliveryMission(Node):
    def __init__(self):
        super().__init__('delivery_mission')
        declare_interface_params(self)

        # ========== Mission configuration ==========
        self.pickup_time = 3.0      # seconds to hover and "load" the package
        self.dropoff_time = 3.0     # seconds to hover and "drop" the package
        self.goal_send_timeout = 3.0
        self.results_file = (
            f'/tmp/delivery_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        )

        # Delivery list. Each item:
        #   pickup:  (x, y, z, name)
        #   dropoff: (x, y, z, name)
        #   package_id
        self.deliveries = [
            {
                'package_id': 'PKG-001',
                'pickup':  (5.0,  5.0, 2.0, 'Warehouse A'),
                'dropoff': (-5.0, 5.0, 2.0, 'Customer 1'),
            },
            {
                'package_id': 'PKG-002',
                'pickup':  (5.0, -5.0, 2.0, 'Warehouse B'),
                'dropoff': (-5.0, -5.0, 2.0, 'Customer 2'),
            },
            {
                'package_id': 'PKG-003',
                'pickup':  (6.0,  0.0, 2.0, 'Warehouse C'),
                'dropoff': (0.0,  6.0, 2.0, 'Customer 3'),
            },
        ]

        # ========== State ==========
        self.current_nav_state = 'UNKNOWN'
        self.mission_phase = MP.WAITING_FOR_TAKEOFF
        self.delivery_index = 0
        self.has_package = False       # cargo state
        self.action_start_time = None
        self.send_attempt_time = None
        self.log_entries = []          # completed deliveries

        # ========== Subscribers ==========
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)

        # ========== Timer ==========
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info('=' * 55)
        self.get_logger().info('INDOOR DELIVERY MISSION')
        self.get_logger().info(f'Total deliveries: {len(self.deliveries)}')
        self.get_logger().info(f'Pickup time:  {self.pickup_time}s per package')
        self.get_logger().info(f'Dropoff time: {self.dropoff_time}s per package')
        self.get_logger().info(f'Log file: {self.results_file}')
        self.get_logger().info('=' * 55)

    # =====================================================
    # Subscribers
    # =====================================================
    def nav_state_callback(self, msg):
        # /nav_state format: "STATE | bat=X%"
        self.current_nav_state = msg.data.split(' ')[0]

    # =====================================================
    # Mission phase machine
    # =====================================================
    def tick(self):
        phase = self.mission_phase

        # ---- Wait for drone to be airborne ----
        if phase == MP.WAITING_FOR_TAKEOFF:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone ready. Starting deliveries.')
                self.set_phase(MP.GO_TO_PICKUP)
            else:
                self.get_logger().info(
                    f'Waiting for takeoff... state: {self.current_nav_state}',
                    throttle_duration_sec=2.0
                )

        # ---- Send goal: pickup location ----
        elif phase == MP.GO_TO_PICKUP:
            if self.delivery_index >= len(self.deliveries):
                self.get_logger().info('All deliveries complete.')
                self.set_phase(MP.SEND_RTH)
                return
            d = self.deliveries[self.delivery_index]
            self.send_goal(d['pickup'], f'PICKUP ({d["package_id"]})')
            self.send_attempt_time = time.time()
            self.set_phase(MP.WAIT_NAV_PICKUP)

        elif phase == MP.WAIT_NAV_PICKUP:
            if self.current_nav_state == 'NAVIGATING':
                self.set_phase(MP.WAIT_HOVER_PICKUP)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.get_logger().warn('Pickup goal not picked up - resending')
                self.set_phase(MP.GO_TO_PICKUP)

        elif phase == MP.WAIT_HOVER_PICKUP:
            if self.current_nav_state == 'HOVER':
                d = self.deliveries[self.delivery_index]
                self.get_logger().info(
                    f'Arrived at {d["pickup"][3]}. Picking up {d["package_id"]}...'
                )
                self.action_start_time = time.time()
                self.set_phase(MP.PICKING_UP)

        # ---- Simulate pickup ----
        elif phase == MP.PICKING_UP:
            elapsed = time.time() - self.action_start_time
            remaining = self.pickup_time - elapsed
            if remaining <= 0:
                d = self.deliveries[self.delivery_index]
                self.has_package = True
                self.get_logger().info(
                    f'[PICKED UP] {d["package_id"]} from {d["pickup"][3]} '
                    f'(cargo: LOADED)'
                )
                self.set_phase(MP.GO_TO_DROPOFF)
            else:
                self.get_logger().info(
                    f'Loading... {remaining:.1f}s', throttle_duration_sec=1.0
                )

        # ---- Send goal: dropoff location ----
        elif phase == MP.GO_TO_DROPOFF:
            d = self.deliveries[self.delivery_index]
            self.send_goal(d['dropoff'], f'DROPOFF ({d["package_id"]})')
            self.send_attempt_time = time.time()
            self.set_phase(MP.WAIT_NAV_DROPOFF)

        elif phase == MP.WAIT_NAV_DROPOFF:
            if self.current_nav_state == 'NAVIGATING':
                self.set_phase(MP.WAIT_HOVER_DROPOFF)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.get_logger().warn('Dropoff goal not picked up - resending')
                self.set_phase(MP.GO_TO_DROPOFF)

        elif phase == MP.WAIT_HOVER_DROPOFF:
            if self.current_nav_state == 'HOVER':
                d = self.deliveries[self.delivery_index]
                self.get_logger().info(
                    f'Arrived at {d["dropoff"][3]}. Delivering {d["package_id"]}...'
                )
                self.action_start_time = time.time()
                self.set_phase(MP.DROPPING_OFF)

        # ---- Simulate dropoff ----
        elif phase == MP.DROPPING_OFF:
            elapsed = time.time() - self.action_start_time
            remaining = self.dropoff_time - elapsed
            if remaining <= 0:
                d = self.deliveries[self.delivery_index]
                self.has_package = False
                self.log_entries.append({
                    'package_id': d['package_id'],
                    'from': d['pickup'][3],
                    'to': d['dropoff'][3],
                    'completed_at': datetime.now().isoformat(),
                })
                self.get_logger().info(
                    f'[DELIVERED] {d["package_id"]} to {d["dropoff"][3]} '
                    f'(cargo: EMPTY)'
                )
                self.set_phase(MP.NEXT_DELIVERY)
            else:
                self.get_logger().info(
                    f'Unloading... {remaining:.1f}s', throttle_duration_sec=1.0
                )

        # ---- Move to next delivery or finish ----
        elif phase == MP.NEXT_DELIVERY:
            self.delivery_index += 1
            self.get_logger().info(
                f'Completed {self.delivery_index}/{len(self.deliveries)} deliveries'
            )
            self.set_phase(MP.GO_TO_PICKUP)

        # ---- Mission complete: return home and land ----
        elif phase == MP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info('All packages delivered. Returning home.')
            self.set_phase(MP.WAIT_LANDED)

        elif phase == MP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.write_summary()
                self.set_phase(MP.DONE)
            else:
                self.get_logger().info(
                    f'Returning home... state: {self.current_nav_state}',
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
            cargo = 'LOADED' if self.has_package else 'EMPTY'
            self.get_logger().info(
                f'MISSION: {self.mission_phase} -> {new_phase}  [cargo: {cargo}]'
            )
            self.mission_phase = new_phase

    def send_goal(self, point, label):
        x, y, z, name = point
        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'>>> Going to {label}: {name} ({x}, {y}, {z})'
        )

    def write_summary(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('DELIVERY SUMMARY')
        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f'Completed: {len(self.log_entries)}/{len(self.deliveries)} deliveries'
        )
        for e in self.log_entries:
            self.get_logger().info(
                f'  {e["package_id"]}: {e["from"]} -> {e["to"]}'
            )

        try:
            with open(self.results_file, 'w') as f:
                f.write('Indoor Delivery Mission Report\n')
                f.write(f'Date: {datetime.now().isoformat()}\n')
                f.write(
                    f'Deliveries: {len(self.log_entries)}/'
                    f'{len(self.deliveries)}\n'
                )
                f.write('=' * 55 + '\n')
                for e in self.log_entries:
                    f.write(
                        f'{e["package_id"]} | {e["from"]:15s} -> '
                        f'{e["to"]:15s} | {e["completed_at"]}\n'
                    )
            self.get_logger().info(f'Report written to: {self.results_file}')
        except Exception as ex:
            self.get_logger().error(f'Failed to write report: {ex}')

        self.get_logger().info('=' * 55)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
