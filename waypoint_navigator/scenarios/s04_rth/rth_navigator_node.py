#!/usr/bin/env python3
"""
RTH Navigator - Goal navigation + Return-to-Home + Battery monitor.

Features:
  - Accepts goals from RViz '2D Goal Pose' tool
  - Records home position on first arm
  - Auto Return-to-Home when battery drops below threshold
  - Manual Return-to-Home via /rth_trigger topic (std_msgs/Bool)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, BatteryStatus
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

import math
from ...common.interfaces import declare_interface_params


class State:
    INIT = 'INIT'                  # waiting for position
    NORMAL = 'NORMAL'              # goal-from-RViz mode
    RTH = 'RTH'                    # returning home
    HOVER_HOME = 'HOVER_HOME'      # arrived home, brief hover before landing
    LANDING = 'LANDING'            # arm=false sent; C++ node handles AUTO_LAND


class RTHNavigator(Node):
    def __init__(self):
        super().__init__('rth_navigator')
        declare_interface_params(self)

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- Parameters ----
        self.tolerance = 0.5
        self.max_speed = 1.0
        self.default_altitude = 2.0
        self.battery_threshold = 0.20   # 20% -> trigger RTH
        self.battery_critical = 0.10    # 10% -> warn loudly
        self.hover_before_land_sec = 2.0  # brief hover above home before landing

        # ---- State ----
        self.state = State.INIT
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        # Home position (set on first arm)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = self.default_altitude
        self.home_recorded = False

        # Goal (from RViz)
        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude

        # Battery
        self.battery_remaining = 1.0
        self.battery_received = False
        self.low_battery_warned = False

        self.armed = False
        self.hover_home_start = None    # set when entering HOVER_HOME
        self.land_command_sent = False  # arm=false sent once on LANDING entry

        # ---- Subscribers ----
        # NOTE: PX4 v1.16+ uses /fmu/out/*_v1 versioned topics. The non-versioned
        # names are not published in this build.
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            self.topic('battery'),
            self.battery_callback,
            px4_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Manual RTH trigger
        self.rth_trigger_sub = self.create_subscription(
            Bool,
            '/rth_trigger',
            self.rth_trigger_callback,
            10
        )

        # ---- Publishers ----
        # NOTE: must match topics the C++ offboard_control_node (in package
        # px4_offboard_sim) subscribes to. See parameters "topics.cmd_vel" and
        # "topics.arm" in offboard_control_node.cpp.
        self.velocity_pub = self.create_publisher(
            Twist, self.topic('cmd_vel'), 10
        )
        self.arm_pub = self.create_publisher(
            Bool, self.topic('arm'), 10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RTH Navigator started.')
        self.get_logger().info(
            f'Battery threshold: {self.battery_threshold * 100:.0f}% | '
            f'Default altitude: {self.default_altitude}m'
        )

    # =====================================================
    # Callbacks
    # =====================================================
    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def battery_callback(self, msg):
        self.battery_remaining = msg.remaining
        self.battery_received = True

    def goal_callback(self, msg):
        # Ignore goals while RTH is active
        if self.state == State.RTH:
            self.get_logger().warn('Ignoring goal: RTH in progress.')
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = self.default_altitude
        self.has_goal = True
        self.state = State.NORMAL

        self.get_logger().info(
            f'Goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})'
        )

    def rth_trigger_callback(self, msg):
        if msg.data and self.state != State.RTH:
            self.get_logger().warn('Manual RTH triggered!')
            self.start_rth()

    # =====================================================
    # State transitions
    # =====================================================
    def start_rth(self):
        if not self.home_recorded:
            self.get_logger().error('Cannot RTH: home position not recorded yet.')
            return
        self.state = State.RTH
        self.get_logger().warn(
            f'>>> RETURNING HOME: ({self.home_x:.2f}, {self.home_y:.2f}, '
            f'{self.home_z:.2f}) <<<'
        )

    def check_battery(self):
        if not self.battery_received:
            return

        if self.battery_remaining < self.battery_critical:
            self.get_logger().error(
                f'CRITICAL BATTERY: {self.battery_remaining * 100:.0f}%',
                throttle_duration_sec=2.0
            )

        if (self.battery_remaining < self.battery_threshold
                and self.state != State.RTH):
            if not self.low_battery_warned:
                self.get_logger().warn(
                    f'Low battery ({self.battery_remaining * 100:.0f}%) - '
                    f'triggering auto RTH'
                )
                self.low_battery_warned = True
            self.start_rth()

    # =====================================================
    # Main control loop
    # =====================================================
    def control_loop(self):
        if not self.position_received:
            return

        # Arm once, record home position
        if not self.armed:
            arm_msg = Bool()
            arm_msg.data = True
            self.arm_pub.publish(arm_msg)
            self.armed = True
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.default_altitude
            self.home_recorded = True
            self.state = State.NORMAL
            self.get_logger().info(
                f'Armed. Home set to: ({self.home_x:.2f}, {self.home_y:.2f}, '
                f'{self.home_z:.2f})'
            )

        # Battery check (can change state to RTH)
        self.check_battery()

        # ---- Dispatch by state ----
        if self.state == State.NORMAL:
            self.handle_normal()
        elif self.state == State.RTH:
            self.handle_rth()
        elif self.state == State.HOVER_HOME:
            self.handle_hover_home()
        elif self.state == State.LANDING:
            self.handle_landing()
        else:
            self.publish_velocity(0.0, 0.0, 0.0)

    def handle_normal(self):
        if not self.has_goal:
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        self.navigate_to(self.goal_x, self.goal_y, self.goal_z, label='goal')

    def handle_rth(self):
        reached = self.navigate_to(
            self.home_x, self.home_y, self.home_z, label='HOME'
        )
        if reached:
            self.state = State.HOVER_HOME
            self.hover_home_start = self.get_clock().now()
            self.get_logger().info(
                f'Reached home. Hovering {self.hover_before_land_sec:.0f}s '
                f'before landing.'
            )

    def handle_hover_home(self):
        # Keep position over home, then transition to LANDING after a short hover.
        self.publish_velocity(0.0, 0.0, 0.0)
        elapsed = (self.get_clock().now() - self.hover_home_start).nanoseconds / 1e9
        if elapsed >= self.hover_before_land_sec:
            self.state = State.LANDING
            self.get_logger().warn('>>> LANDING (AUTO_LAND via offboard_control) <<<')

    def handle_landing(self):
        # Publish arm=false ONCE. The C++ offboard_control_node sees this,
        # switches PX4 to AUTO_LAND, waits for ground contact, then disarms.
        if not self.land_command_sent:
            disarm_msg = Bool()
            disarm_msg.data = False
            self.arm_pub.publish(disarm_msg)
            self.land_command_sent = True
            self.get_logger().info(
                'Sent arm=false to /px4_offboard_sim/offboard_control/arm. '
                'PX4 will descend and disarm on touchdown.'
            )
        # Stop sending velocity setpoints so PX4 auto-land takes full control.

    # =====================================================
    # Helpers
    # =====================================================
    def navigate_to(self, tx, ty, tz, label='target'):
        """Compute velocity toward (tx, ty, tz). Returns True if arrived."""
        dx = tx - self.current_x
        dy = ty - self.current_y
        dz = tz - self.current_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < self.tolerance:
            self.publish_velocity(0.0, 0.0, 0.0)
            return True

        vx = (dx / distance) * self.max_speed
        vy = (dy / distance) * self.max_speed
        vz = (dz / distance) * self.max_speed

        if distance < 2.0:
            scale = distance / 2.0
            vx *= scale
            vy *= scale
            vz *= scale

        self.publish_velocity(vx, vy, vz)

        self.get_logger().info(
            f'[{self.state}] -> {label}: dist={distance:.2f}m | '
            f'battery={self.battery_remaining * 100:.0f}%',
            throttle_duration_sec=1.0
        )
        return False

    def publish_velocity(self, vx, vy, vz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0
        self.velocity_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RTHNavigator()
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
