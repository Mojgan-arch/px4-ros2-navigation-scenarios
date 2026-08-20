#!/usr/bin/env python3
"""
Obstacle Avoidance Navigator - Version 1 (Potential Field)
Combines goal-from-RViz navigation with LiDAR-based reactive obstacle avoidance.

Algorithm (Potential Field):
  - Attractive force pulls drone toward goal.
  - Repulsive force pushes drone away from nearby LiDAR points.
  - Net velocity = sum of both forces.

Limitations (this is v1):
  - Assumes drone yaw is constant (LiDAR body frame ~= world frame).
  - Can get stuck in local minima (e.g., U-shaped obstacles).
  - Reactive only (no planning ahead).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import math
from .interfaces import declare_interface_params


class ObstacleAvoidanceNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')
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

        # ---- Parameters (overridable via --ros-args -p name:=value) ----
        # Defaults are tuned for the 10x10 m walled_arena world. Override them
        # for a denser course, or for a real airframe (see docs/hardware.md).
        self.declare_parameter('tolerance', 0.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('default_altitude', 2.0)
        self.declare_parameter('safety_radius', 2.5)
        self.declare_parameter('min_lidar_range', 0.5)
        self.declare_parameter('goal_gain', 1.0)
        self.declare_parameter('obstacle_gain', 2.0)
        self.declare_parameter('tangential_gain', 2.0)

        self.tolerance        = self.get_parameter('tolerance').value
        self.max_speed        = self.get_parameter('max_speed').value
        self.default_altitude = self.get_parameter('default_altitude').value
        self.safety_radius    = self.get_parameter('safety_radius').value
        self.min_lidar_range  = self.get_parameter('min_lidar_range').value
        self.goal_gain        = self.get_parameter('goal_gain').value
        self.obstacle_gain    = self.get_parameter('obstacle_gain').value
        self.tangential_gain  = self.get_parameter('tangential_gain').value

        # ---- State ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude

        # List of (x, y, z) obstacle points in body frame
        self.obstacles = []

        self.armed = False

        # ---- Subscribers ----
        # NOTE: PX4 v1.16+ uses /fmu/out/*_v1 versioned topics.
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # LiDAR point cloud published by the sim bridge.
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.topic('pointcloud'),
            self.lidar_callback,
            lidar_qos
        )

        # ---- Publishers ----
        # NOTE: must match topics the C++ offboard_control_node (px4_offboard_sim)
        # subscribes to. See parameters "topics.cmd_vel" and "topics.arm".
        self.velocity_pub = self.create_publisher(
            Twist, self.topic('cmd_vel'), 10
        )
        self.arm_pub = self.create_publisher(
            Bool, self.topic('arm'), 10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Navigator started.')
        self.get_logger().info(
            f'safety_radius={self.safety_radius}m | goal_gain={self.goal_gain} | '
            f'obstacle_gain={self.obstacle_gain} | tangential_gain={self.tangential_gain}'
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

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = self.default_altitude
        self.has_goal = True
        self.get_logger().info(
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})'
        )

    def lidar_callback(self, msg):
        """Read LiDAR point cloud, keep nearby points as obstacle list."""
        new_obstacles = []
        try:
            points = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            for p in points:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                d = math.sqrt(x * x + y * y + z * z)
                # Skip points too close (drone body) or too far (ignore)
                if self.min_lidar_range < d < self.safety_radius:
                    new_obstacles.append((x, y, z, d))
        except Exception as e:
            self.get_logger().warn(f'LiDAR parse error: {e}', throttle_duration_sec=5.0)
            return

        self.obstacles = new_obstacles

    # =====================================================
    # Main control loop
    # =====================================================
    def control_loop(self):
        if not self.position_received:
            return

        if not self.armed:
            arm_msg = Bool()
            arm_msg.data = True
            self.arm_pub.publish(arm_msg)
            self.armed = True
            self.get_logger().info('Arm command sent.')

        if not self.has_goal:
            self.publish_velocity(0.0, 0.0, 0.0)
            return

        # --- Attractive force (toward goal, world frame) ---
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dz = self.goal_z - self.current_z
        distance_to_goal = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance_to_goal < self.tolerance:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info(
                'Goal reached. Hovering.', throttle_duration_sec=2.0
            )
            return

        # Normalize attractive force
        ax = (dx / distance_to_goal) * self.goal_gain
        ay = (dy / distance_to_goal) * self.goal_gain
        az = (dz / distance_to_goal) * self.goal_gain

        # --- Repulsive + tangential force (potential field with swirl) ---
        # NOTE: assumes body frame ~= world frame (yaw stable).
        #
        # Pure radial repulsion fails when the drone, obstacle and goal are
        # collinear: the push-back force is exactly opposite the goal pull, so
        # they cancel and the drone just oscillates. Adding a TANGENTIAL
        # ("swirl") component perpendicular to each obstacle direction makes the
        # drone slide around obstacles instead of bouncing straight off them.
        rx, ry, rz = 0.0, 0.0, 0.0
        num_obstacles = len(self.obstacles)
        for (ox, oy, oz, d) in self.obstacles:
            # Magnitude: stronger when closer (0 at safety_radius, 1 at contact)
            strength = ((self.safety_radius - d) / self.safety_radius) ** 2

            # Radial component: push directly away from the obstacle
            radial_mag = strength * self.obstacle_gain
            rx += -(ox / d) * radial_mag
            ry += -(oy / d) * radial_mag
            rz += -(oz / d) * radial_mag

            # Tangential component (XY plane only — keep altitude stable).
            # Rotate the obstacle direction +90 deg to get a perpendicular.
            tx = -(oy / d)
            ty = (ox / d)
            # Pick the side that points toward the goal so we go the short way.
            # When collinear (dot ~ 0) the fixed +90 deg choice still breaks the
            # symmetry, so the drone never gets stuck head-on.
            if (tx * ax + ty * ay) < 0.0:
                tx, ty = -tx, -ty
            tang_mag = strength * self.tangential_gain
            rx += tx * tang_mag
            ry += ty * tang_mag

        # --- Combine forces ---
        fx = ax + rx
        fy = ay + ry
        fz = az + rz

        # Cap to max_speed
        f_mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if f_mag > self.max_speed:
            fx = (fx / f_mag) * self.max_speed
            fy = (fy / f_mag) * self.max_speed
            fz = (fz / f_mag) * self.max_speed

        # Slow down near goal (proportional)
        if distance_to_goal < 2.0:
            scale = distance_to_goal / 2.0
            fx *= scale
            fy *= scale
            fz *= scale

        self.publish_velocity(fx, fy, fz)

        self.get_logger().info(
            f'goal_dist={distance_to_goal:.2f}m | '
            f'obs={num_obstacles} | '
            f'vel=({fx:.2f}, {fy:.2f}, {fz:.2f})',
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
    node = ObstacleAvoidanceNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
