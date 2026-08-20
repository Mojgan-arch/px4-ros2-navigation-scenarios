#!/usr/bin/env python3
"""
Full Navigation State Machine
Combines goal navigation, obstacle avoidance, battery monitoring,
RTH, takeoff/landing into a single coherent state machine.

States:
  IDLE        - waiting for first position
  ARMING      - sending arm command
  TAKEOFF     - ascending to default altitude
  HOVER       - holding position, waiting for goal
  NAVIGATING  - flying to goal (with obstacle avoidance)
  RTH         - returning to home
  LANDING     - descending toward ground
  LANDED      - on ground, motors off
  EMERGENCY   - critical fault, hold or descend immediately
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, BatteryStatus
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import math
from .interfaces import declare_interface_params


class S:
    """State constants."""
    IDLE = 'IDLE'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    HOVER = 'HOVER'
    NAVIGATING = 'NAVIGATING'
    RTH = 'RTH'
    LANDING = 'LANDING'
    LANDED = 'LANDED'
    EMERGENCY = 'EMERGENCY'


class NavigationStateMachine(Node):
    def __init__(self):
        super().__init__('navigation_state_machine')
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
        # The C++ offboard_control node holds the drone at ~2.5 m after takeoff
        # (observed in its POSITION hold logs). Match that here so the takeoff
        # transition actually fires and navigation goals are reachable.
        self.default_altitude = 2.5
        self.tolerance = 0.5
        # nav_tolerance is used for ordinary waypoint navigation. Can be made
        # tighter via parameter when a precise track is needed (e.g. path
        # planner mode) so the drone hugs the planned line instead of cutting
        # corners up to half a metre.
        self.declare_parameter('nav_tolerance', self.tolerance)
        self.nav_tolerance = self.get_parameter('nav_tolerance').value
        # Distance at which the drone starts decelerating toward a waypoint.
        # Defaults to 2.0 m, which is fine for a single goal but causes
        # accel/decel jitter when chaining many close waypoints. Path planner
        # mode lowers this (e.g. 0.5 m, smaller than its 1 m look-ahead) so
        # the drone cruises through intermediate waypoints at max_speed and
        # only slows for the final one.
        self.declare_parameter('slowdown_distance', 2.0)
        self.slowdown_distance = self.get_parameter('slowdown_distance').value
        self.max_speed = 2.0           # m/s — faster cruise so missions finish quicker
        self.takeoff_speed = 1.0       # m/s — faster climb
        self.landing_speed = 0.3

        # Obstacle avoidance
        # safety_radius is how close (m) an obstacle has to be before the
        # repulsion force starts acting. 2.5 m is good for reactive flight in
        # open space, but in narrow corridors (e.g. a 2 m doorway) it makes
        # both walls push on the drone at once, and near a waypoint close to
        # a wall it makes the drone circle around the goal instead of
        # settling. Override via parameter for planner-driven missions.
        self.declare_parameter('safety_radius', 2.5)
        self.safety_radius = self.get_parameter('safety_radius').value
        self.min_lidar_range = 0.5
        self.goal_gain = 1.0
        self.obstacle_gain = 2.0        # radial repulsive strength (push away)
        self.tangential_gain = 2.0      # swirl strength (go around, not bounce)

        # Live reactive avoidance toggle. Disable when a higher-level planner
        # (e.g. path_planner_node) already produces collision-free waypoints.
        # WARNING: fully disabling avoidance is dangerous — if the planner's
        # grid is even slightly off (LiDAR misses a face, inflation too low)
        # the drone flies straight into the wall. The avoidance_gain_scale
        # parameter (below) lets us keep a small safety reactive force.
        self.declare_parameter('reactive_avoidance', True)
        self.reactive_avoidance = self.get_parameter('reactive_avoidance').value
        if not self.reactive_avoidance:
            self.obstacle_gain = 0.0
            self.tangential_gain = 0.0

        # Independent multiplier on top of reactive_avoidance. Use a small value
        # (e.g. 0.3) when the drone is mostly following a pre-planned path but
        # you still want a soft "don't hit the wall" repulsion as a safety net.
        self.declare_parameter('avoidance_gain_scale', 1.0)
        scale = self.get_parameter('avoidance_gain_scale').value
        self.obstacle_gain *= scale
        self.tangential_gain *= scale

        # Battery thresholds
        self.battery_low = 0.20
        self.battery_critical = 0.10

        # Autonomous mission (no clicks needed). Run with mission_mode:=false to
        # fly interactively (goals from RViz) instead.
        self.declare_parameter('mission_mode', True)
        self.mission_mode = self.get_parameter('mission_mode').value
        # Preset waypoints (ENU x, y, z). Leg 1 passes the pillar at (2, 0) to
        # exercise obstacle avoidance. After the last waypoint the drone returns
        # home (arm position) and lands — fully automatic.
        self.mission_waypoints = [
            (4.0, 0.0, self.default_altitude),   # forward, past the pillar
            (4.0, 3.0, self.default_altitude),   # turn right
            (0.0, 3.0, self.default_altitude),   # come back across
        ]
        self.mission_start_delay = 3.0           # s to stabilise before starting

        # ========== State variables ==========
        self.state = S.IDLE
        self.prev_state = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False

        self.home_x = 0.0
        self.home_y = 0.0
        self.home_recorded = False

        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = self.default_altitude

        self.obstacles = []

        self.battery_remaining = 1.0
        self.battery_received = False

        # Mission progress
        self.mission_index = 0
        self.in_mission = False        # currently flying mission waypoints
        self.mission_started = False   # mission kicked off once (don't repeat)
        self.hover_start_time = None   # for the pre-mission stabilise delay

        # ========== Subscribers ==========
        # NOTE: PX4 v1.16+ uses /fmu/out/*_v1 versioned topics, and the sim
        # publishes LiDAR on /px4_offboard_sim/lidar/points.
        self.create_subscription(
            VehicleLocalPosition, self.topic('local_position'),
            self.position_callback, px4_qos
        )
        self.create_subscription(
            BatteryStatus, self.topic('battery'),
            self.battery_callback, px4_qos
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.create_subscription(
            PointCloud2, self.topic('pointcloud'), self.lidar_callback, lidar_qos
        )
        self.create_subscription(
            Bool, '/rth_trigger', self.rth_trigger_callback, 10
        )
        self.create_subscription(
            Bool, '/land_trigger', self.land_trigger_callback, 10
        )
        self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10
        )

        # ========== Publishers ==========
        # NOTE: must match topics the C++ offboard_control_node (px4_offboard_sim)
        # subscribes to — parameters "topics.cmd_vel" and "topics.arm".
        self.velocity_pub = self.create_publisher(
            Twist, self.topic('cmd_vel'), 10
        )
        self.arm_pub = self.create_publisher(
            Bool, self.topic('arm'), 10
        )
        # Publish current state for monitoring
        self.state_pub = self.create_publisher(
            String, '/nav_state', 10
        )

        # ========== Timer ==========
        self.timer = self.create_timer(0.1, self.tick)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Navigation State Machine started.')
        self.get_logger().info(f'Initial state: {self.state}')
        if self.mission_mode:
            self.get_logger().info(
                f'MISSION MODE: autonomous, {len(self.mission_waypoints)} waypoints '
                f'-> RTH -> land (no clicks needed)'
            )
        else:
            self.get_logger().info('INTERACTIVE MODE: waiting for RViz goals')
        self.get_logger().info(
            f'reactive_avoidance: {self.reactive_avoidance} '
            f'(obstacle_gain={self.obstacle_gain}, tangential_gain={self.tangential_gain})'
        )

    # =====================================================
    # Subscriber callbacks
    # =====================================================
    def position_callback(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def battery_callback(self, msg):
        self.battery_remaining = msg.remaining
        self.battery_received = True

    def goal_callback(self, msg):
        # A manual RViz goal overrides the autonomous mission.
        if self.state in (S.HOVER, S.NAVIGATING):
            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y
            self.goal_z = self.default_altitude
            self.has_goal = True
            self.in_mission = False        # manual control takes over
            self.mission_started = True    # don't auto-restart the mission
            self.transition(S.NAVIGATING)
            self.get_logger().info(
                f'Manual goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})'
            )
        else:
            self.get_logger().warn(f'Ignoring goal: state is {self.state}')

    def lidar_callback(self, msg):
        new_obs = []
        try:
            points = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
            for p in points:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                d = math.sqrt(x * x + y * y + z * z)
                if self.min_lidar_range < d < self.safety_radius:
                    new_obs.append((x, y, z, d))
        except Exception as e:
            self.get_logger().warn(f'LiDAR error: {e}', throttle_duration_sec=5.0)
            return
        self.obstacles = new_obs

    def rth_trigger_callback(self, msg):
        if msg.data and self.state in (S.HOVER, S.NAVIGATING):
            self.get_logger().warn('Manual RTH triggered.')
            self.transition(S.RTH)

    def land_trigger_callback(self, msg):
        if msg.data and self.state in (S.HOVER, S.NAVIGATING):
            self.get_logger().warn('Land triggered.')
            self.transition(S.LANDING)

    def emergency_callback(self, msg):
        if msg.data:
            self.get_logger().error('!!! EMERGENCY STOP !!!')
            self.transition(S.EMERGENCY)

    # =====================================================
    # State machine core
    # =====================================================
    def transition(self, new_state):
        if new_state == self.state:
            return
        self.prev_state = self.state
        self.state = new_state
        self.get_logger().info(f'STATE: {self.prev_state} -> {self.state}')

    def publish_state(self):
        msg = String()
        msg.data = f'{self.state} | bat={self.battery_remaining * 100:.0f}%'
        self.state_pub.publish(msg)

    def check_battery_and_global_events(self):
        """Run on every tick regardless of state."""
        if not self.battery_received:
            return
        # Critical battery -> emergency (only if airborne)
        if (self.battery_remaining < self.battery_critical
                and self.state not in (S.LANDED, S.IDLE, S.EMERGENCY)):
            self.get_logger().error(
                f'Battery CRITICAL ({self.battery_remaining * 100:.0f}%)'
            )
            self.transition(S.EMERGENCY)
            return
        # Low battery -> RTH (only if in active flight)
        if (self.battery_remaining < self.battery_low
                and self.state in (S.HOVER, S.NAVIGATING)):
            self.get_logger().warn(
                f'Battery low ({self.battery_remaining * 100:.0f}%) - RTH'
            )
            self.transition(S.RTH)

    def tick(self):
        if not self.position_received:
            return

        self.check_battery_and_global_events()

        # Dispatch to current state handler
        handler = {
            S.IDLE: self.handle_idle,
            S.ARMING: self.handle_arming,
            S.TAKEOFF: self.handle_takeoff,
            S.HOVER: self.handle_hover,
            S.NAVIGATING: self.handle_navigating,
            S.RTH: self.handle_rth,
            S.LANDING: self.handle_landing,
            S.LANDED: self.handle_landed,
            S.EMERGENCY: self.handle_emergency,
        }.get(self.state, self.handle_idle)
        handler()

    # =====================================================
    # State handlers
    # =====================================================
    def handle_idle(self):
        self.publish_velocity(0.0, 0.0, 0.0)
        # Auto-start: go to ARMING once position is known
        self.transition(S.ARMING)

    def handle_arming(self):
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)
        # Record home, go to TAKEOFF
        self.home_x = self.current_x
        self.home_y = self.current_y
        self.home_recorded = True
        self.get_logger().info(
            f'Home set: ({self.home_x:.2f}, {self.home_y:.2f})'
        )
        self.transition(S.TAKEOFF)

    def handle_takeoff(self):
        # The C++ offboard node performs the real climb. We just wait until the
        # drone is at (or above) the target altitude, then hand over to HOVER.
        # One-sided test (>=) so we don't get stuck oscillating on the boundary
        # when the C++ node settles right at default_altitude - tolerance.
        if self.current_z >= self.default_altitude - self.tolerance:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.transition(S.HOVER)
            return
        self.publish_velocity(0.0, 0.0, self.takeoff_speed)
        self.get_logger().info(
            f'Takeoff: z={self.current_z:.2f}m -> {self.default_altitude}m',
            throttle_duration_sec=1.0
        )

    def handle_hover(self):
        self.publish_velocity(0.0, 0.0, 0.0)
        # In mission mode, auto-start the waypoint mission once, after a short
        # stabilising hover. Otherwise wait for an RViz goal / RTH / land.
        if not self.mission_mode or self.mission_started:
            return
        if self.hover_start_time is None:
            self.hover_start_time = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.mission_start_delay:
            self.start_mission()

    def start_mission(self):
        self.mission_started = True
        self.in_mission = True
        self.mission_index = 0
        self.get_logger().info(
            f'>>> AUTONOMOUS MISSION START: {len(self.mission_waypoints)} waypoints <<<'
        )
        self.load_mission_waypoint()

    def load_mission_waypoint(self):
        wp = self.mission_waypoints[self.mission_index]
        self.goal_x, self.goal_y, self.goal_z = wp
        self.has_goal = True
        self.transition(S.NAVIGATING)
        self.get_logger().info(
            f'Mission waypoint {self.mission_index + 1}/{len(self.mission_waypoints)}: '
            f'({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})'
        )

    def handle_navigating(self):
        if not self.has_goal:
            self.transition(S.HOVER)
            return
        label = f'WP{self.mission_index + 1}' if self.in_mission else 'goal'
        reached = self.fly_to_with_avoidance(
            self.goal_x, self.goal_y, self.goal_z, label=label
        )
        if not reached:
            return

        if self.in_mission:
            # Advance to the next mission waypoint, or RTH when finished.
            self.mission_index += 1
            if self.mission_index < len(self.mission_waypoints):
                self.load_mission_waypoint()
            else:
                self.get_logger().info('Mission waypoints complete -> RTH')
                self.in_mission = False
                self.has_goal = False
                self.transition(S.RTH)
        else:
            self.get_logger().info('Goal reached.')
            self.has_goal = False
            self.transition(S.HOVER)

    def handle_rth(self):
        # Tight tolerance so the drone centres precisely over home before
        # descending — it should land right on the X marker.
        reached = self.fly_to_with_avoidance(
            self.home_x, self.home_y, self.default_altitude, label='HOME', tol=0.2
        )
        if reached:
            self.get_logger().info('Home reached. Landing.')
            self.transition(S.LANDING)

    def handle_landing(self):
        # Descend at landing_speed, when near ground -> LANDED
        if self.current_z < 0.3:
            self.publish_velocity(0.0, 0.0, 0.0)
            # Disarm
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            self.transition(S.LANDED)
            return
        # Keep correcting horizontal drift toward home while descending, so the
        # drone lands centred on the X instead of drifting off during descent.
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        vx = max(-0.3, min(0.3, dx))
        vy = max(-0.3, min(0.3, dy))
        self.publish_velocity(vx, vy, -self.landing_speed)
        self.get_logger().info(
            f'Landing: z={self.current_z:.2f}m | xy_err={math.hypot(dx, dy):.2f}m',
            throttle_duration_sec=1.0
        )

    def handle_landed(self):
        self.publish_velocity(0.0, 0.0, 0.0)
        self.get_logger().info('Landed. Mission complete.', throttle_duration_sec=5.0)

    def handle_emergency(self):
        # Slow descent in place
        if self.current_z < 0.3:
            self.publish_velocity(0.0, 0.0, 0.0)
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            return
        self.publish_velocity(0.0, 0.0, -self.landing_speed)
        self.get_logger().error(
            'EMERGENCY: descending in place', throttle_duration_sec=2.0
        )

    # =====================================================
    # Shared helpers (used by NAVIGATING and RTH)
    # =====================================================
    def fly_to_with_avoidance(self, tx, ty, tz, label='target', tol=None):
        """Potential field navigation. Returns True if arrived.

        tol overrides the arrival tolerance (e.g. tighter for a precise RTH).
        Arrival requires BOTH horizontal (xy) and vertical (z) errors to be
        within tol — using only 3D Euclidean distance lets the drone declare
        "reached" when it's right above the waypoint with the wrong altitude,
        so it never actually follows the path's z variation.
        """
        if tol is None:
            tol = self.nav_tolerance
        dx = tx - self.current_x
        dy = ty - self.current_y
        dz = tz - self.current_z
        xy_dist = math.sqrt(dx * dx + dy * dy)
        z_dist = abs(dz)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if xy_dist < tol and z_dist < tol:
            self.publish_velocity(0.0, 0.0, 0.0)
            return True

        # Attractive
        ax = (dx / dist) * self.goal_gain
        ay = (dy / dist) * self.goal_gain
        az = (dz / dist) * self.goal_gain

        # Repulsive + tangential (swirl) — body frame ~= world frame assumption.
        # Pure radial repulsion oscillates when drone/obstacle/goal are collinear
        # (forces cancel). The tangential term makes the drone slide around the
        # obstacle instead of bouncing straight off it.
        rx, ry, rz = 0.0, 0.0, 0.0
        for (ox, oy, oz, d) in self.obstacles:
            strength = ((self.safety_radius - d) / self.safety_radius) ** 2

            # Radial: push directly away from the obstacle
            radial_mag = strength * self.obstacle_gain
            rx += -(ox / d) * radial_mag
            ry += -(oy / d) * radial_mag
            rz += -(oz / d) * radial_mag

            # Tangential (XY only): rotate obstacle direction +90 deg, pick the
            # side that points toward the goal. Breaks the head-on symmetry.
            tx = -(oy / d)
            ty = (ox / d)
            if (tx * ax + ty * ay) < 0.0:
                tx, ty = -tx, -ty
            tang_mag = strength * self.tangential_gain
            rx += tx * tang_mag
            ry += ty * tang_mag

        fx, fy, fz = ax + rx, ay + ry, az + rz

        f_mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if f_mag > self.max_speed:
            fx = (fx / f_mag) * self.max_speed
            fy = (fy / f_mag) * self.max_speed
            fz = (fz / f_mag) * self.max_speed

        if dist < self.slowdown_distance:
            scale = dist / self.slowdown_distance
            fx *= scale
            fy *= scale
            fz *= scale

        self.publish_velocity(fx, fy, fz)
        self.get_logger().info(
            f'[{self.state}] -> {label}: dist={dist:.2f}m | '
            f'obs={len(self.obstacles)} | bat={self.battery_remaining * 100:.0f}%',
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
    node = NavigationStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
