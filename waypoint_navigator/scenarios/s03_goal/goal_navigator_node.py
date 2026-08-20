#!/usr/bin/env python3
"""
Goal Navigator - Version 1
Reads goal from RViz's '2D Goal Pose' tool and navigates the drone there.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

import math
from ...common.interfaces import declare_interface_params


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        declare_interface_params(self)

        # QoS profile compatible with PX4
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- Parameters ----
        self.tolerance = 0.5         # meters — close enough to goal
        self.max_speed = 1.0         # m/s
        self.default_altitude = 2.0  # m — RViz 2D goal has z=0, so we use this

        # ---- State ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude
        self.goal_reached_logged = False

        self.armed = False

        # ---- Subscribers ----
        # NOTE: PX4 v1.16+ uses /fmu/out/vehicle_local_position_v1 (versioned topic).
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )

        # RViz '2D Goal Pose' tool publishes to /goal_pose by default
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # ---- Publishers ----
        # NOTE: these topics must match what offboard_control_node (C++ in
        # px4_offboard_sim) subscribes to. See offboard_control_node.cpp
        # parameters "topics.cmd_vel" and "topics.arm".
        self.velocity_pub = self.create_publisher(
            Twist,
            self.topic('cmd_vel'),
            10
        )

        self.arm_pub = self.create_publisher(
            Bool,
            self.topic('arm'),
            10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Goal Navigator started.')
        self.get_logger().info('Waiting for goal from RViz (2D Goal Pose tool)...')

    def position_callback(self, msg):
        # PX4 publishes NED (North, East, Down)
        # Convert to ENU (East, North, Up) for ROS convention
        self.current_x = msg.y       # East
        self.current_y = msg.x       # North
        self.current_z = -msg.z      # Up
        self.position_received = True

    def goal_callback(self, msg):
        # RViz publishes in its fixed frame (assumed to be ENU-aligned)
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        # RViz 2D Goal has z=0, so override with default altitude
        self.goal_z = self.default_altitude
        self.has_goal = True
        self.goal_reached_logged = False

        self.get_logger().info(
            f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})'
        )

    def control_loop(self):
        if not self.position_received:
            return

        # Arm once at startup
        if not self.armed:
            arm_msg = Bool()
            arm_msg.data = True
            self.arm_pub.publish(arm_msg)
            self.armed = True
            self.get_logger().info('Arm command sent.')

        # No goal yet → hover in place
        if not self.has_goal:
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        # Calculate distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dz = self.goal_z - self.current_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Reached → hover and wait for next goal
        if distance < self.tolerance:
            self.publish_velocity(0.0, 0.0, 0.0)
            if not self.goal_reached_logged:
                self.get_logger().info(
                    f'Goal reached: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
                    f'{self.goal_z:.2f}). Hovering.'
                )
                self.goal_reached_logged = True
            return

        # Move toward goal at max_speed
        vx = (dx / distance) * self.max_speed
        vy = (dy / distance) * self.max_speed
        vz = (dz / distance) * self.max_speed

        # Slow down within 2m of goal
        if distance < 2.0:
            scale = distance / 2.0
            vx *= scale
            vy *= scale
            vz *= scale

        self.publish_velocity(vx, vy, vz)

        self.get_logger().info(
            f'Distance: {distance:.2f}m | vel=({vx:.2f}, {vy:.2f}, {vz:.2f})',
            throttle_duration_sec=1.0
        )

    def publish_velocity(self, vx, vy, vz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0
        self.velocity_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
