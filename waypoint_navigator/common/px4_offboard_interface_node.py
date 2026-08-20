#!/usr/bin/env python3
"""PX4 offboard interface — the vehicle abstraction layer of this stack.

This node is the ONLY component that talks to the flight controller. Every
other node in this package is platform-agnostic: it publishes a velocity
command and an arm request, and knows nothing about PX4, uORB or NED.

    navigation nodes  --( geometry_msgs/Twist, std_msgs/Bool )-->  THIS NODE
    THIS NODE         --( px4_msgs, Micro XRCE-DDS )-->            PX4

Because the boundary is a plain Twist, the same navigation stack runs against
PX4 SITL today and against a real PX4 airframe later with no code change --
and against a non-PX4 platform by replacing this one file.

Coordinate frames
-----------------
The navigation nodes work in ENU (x=East, y=North, z=Up), which is the ROS
convention. PX4 works in NED (x=North, y=East, z=Down). This node performs
the conversion in one place:

    ned.x =  enu.y        ned.y =  enu.x        ned.z = -enu.z

Yaw rate is likewise negated, since ENU yaw is counter-clockwise about Up
while NED yaw is clockwise about Down.

Flight sequence
---------------
PX4 will only accept offboard mode once it is already receiving a stream of
setpoints, so this node publishes continuously from startup:

    DISARMED -> (arm request) -> ARMING -> TAKEOFF -> ACTIVE
                                                       |
                                     (disarm request)  v
                                                    LANDING -> DISARMED

While ACTIVE the node forwards cmd_vel. If cmd_vel goes stale for longer than
`cmd_timeout` the node commands zero velocity, so a crashed or stopped
navigation node leaves the aircraft hovering rather than continuing on its
last command.

Message definitions used here are the public px4_msgs interface definitions
(github.com/PX4/px4_msgs). No third-party implementation was consulted.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1 (custom mode enabled),
# param2 = 6 (PX4 custom main mode: OFFBOARD).
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

STATE_DISARMED = 'DISARMED'
STATE_ARMING = 'ARMING'
STATE_TAKEOFF = 'TAKEOFF'
STATE_ACTIVE = 'ACTIVE'
STATE_LANDING = 'LANDING'


class PX4OffboardInterface(Node):
    """Bridges Twist/Bool navigation commands onto the PX4 offboard interface."""

    def __init__(self):
        super().__init__('px4_offboard_interface')

        # -- Interface topics (override these to retarget the whole stack) ----
        self.declare_parameter('cmd_vel_topic', '/nav/cmd_vel')
        self.declare_parameter('arm_topic', '/nav/arm')
        self.declare_parameter('status_topic', '/nav/vehicle_state')

        # -- PX4 side ---------------------------------------------------------
        # PX4 v1.15+ publishes versioned output topics (…_v1). Older builds use
        # the unversioned names — override these two parameters if `ros2 topic
        # list` shows /fmu/out/vehicle_local_position without the suffix.
        self.declare_parameter('px4_namespace', '/fmu')
        self.declare_parameter('local_position_topic', 'out/vehicle_local_position_v1')
        self.declare_parameter('vehicle_status_topic', 'out/vehicle_status_v1')
        self.declare_parameter('target_system', 1)

        # -- Flight envelope --------------------------------------------------
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('takeoff_altitude', 2.5)
        self.declare_parameter('takeoff_speed', 1.0)
        self.declare_parameter('landing_speed', 0.7)
        self.declare_parameter('altitude_tolerance', 0.25)
        self.declare_parameter('max_horizontal_speed', 3.0)
        self.declare_parameter('max_vertical_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('setpoints_before_offboard', 20)

        gp = self.get_parameter
        cmd_vel_topic = gp('cmd_vel_topic').value
        arm_topic = gp('arm_topic').value
        status_topic = gp('status_topic').value
        ns = gp('px4_namespace').value.rstrip('/')

        self.target_system = int(gp('target_system').value)
        self.control_rate = float(gp('control_rate').value)
        self.takeoff_altitude = float(gp('takeoff_altitude').value)
        self.takeoff_speed = float(gp('takeoff_speed').value)
        self.landing_speed = float(gp('landing_speed').value)
        self.altitude_tolerance = float(gp('altitude_tolerance').value)
        self.max_h_speed = float(gp('max_horizontal_speed').value)
        self.max_v_speed = float(gp('max_vertical_speed').value)
        self.max_yaw_rate = float(gp('max_yaw_rate').value)
        self.cmd_timeout = float(gp('cmd_timeout').value)
        self.setpoints_before_offboard = int(gp('setpoints_before_offboard').value)

        # PX4 publishes with BEST_EFFORT; a RELIABLE subscriber will not match.
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # -- Publishers to PX4 -------------------------------------------------
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/in/vehicle_command', 10)

        # -- Subscribers from PX4 ---------------------------------------------
        pos_topic = f"{ns}/{gp('local_position_topic').value.lstrip('/')}"
        status_topic_px4 = f"{ns}/{gp('vehicle_status_topic').value.lstrip('/')}"
        self.create_subscription(
            VehicleLocalPosition, pos_topic,
            self.local_position_callback, px4_qos)
        self.create_subscription(
            VehicleStatus, status_topic_px4,
            self.vehicle_status_callback, px4_qos)

        # -- Subscribers from the navigation stack ----------------------------
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Bool, arm_topic, self.arm_callback, 10)

        self.status_pub = self.create_publisher(String, status_topic, 10)

        # -- Internal state ----------------------------------------------------
        self.state = STATE_DISARMED
        self.setpoint_counter = 0
        self.offboard_requested = False

        self.cmd = Twist()
        self.last_cmd_time = None

        self.altitude = 0.0          # metres up, ENU
        self.position_valid = False
        self.px4_armed = False

        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f'PX4 offboard interface ready.\n'
            f'  commands in : {cmd_vel_topic} (Twist, ENU)\n'
            f'  arm requests: {arm_topic} (Bool)\n'
            f'  PX4 topics  : {ns}/in/*, {ns}/out/*\n'
            f'  takeoff alt : {self.takeoff_altitude:.1f} m'
        )

    # ------------------------------------------------------------------ input

    def local_position_callback(self, msg):
        self.altitude = -msg.z          # NED down -> ENU up
        self.position_valid = bool(msg.z_valid)

    def vehicle_status_callback(self, msg):
        self.px4_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def cmd_vel_callback(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def arm_callback(self, msg):
        if msg.data and self.state == STATE_DISARMED:
            self.get_logger().info('Arm requested — starting offboard sequence.')
            self.state = STATE_ARMING
            self.setpoint_counter = 0
            self.offboard_requested = False
        elif not msg.data and self.state in (STATE_TAKEOFF, STATE_ACTIVE):
            self.get_logger().info('Disarm requested — landing.')
            self.state = STATE_LANDING

    # ------------------------------------------------------------- main cycle

    def control_loop(self):
        # PX4 requires an uninterrupted setpoint stream to hold offboard mode,
        # so this is published on every tick regardless of state.
        self.publish_offboard_control_mode()

        if self.state == STATE_DISARMED:
            self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)

        elif self.state == STATE_ARMING:
            self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)
            self.setpoint_counter += 1
            if (self.setpoint_counter >= self.setpoints_before_offboard
                    and not self.offboard_requested):
                self.engage_offboard_mode()
                self.arm()
                self.offboard_requested = True
                self.state = STATE_TAKEOFF
                self.get_logger().info(
                    f'Offboard engaged, arming. Climbing to '
                    f'{self.takeoff_altitude:.1f} m.')

        elif self.state == STATE_TAKEOFF:
            if not self.position_valid:
                self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)
            elif self.altitude < self.takeoff_altitude - self.altitude_tolerance:
                # ENU up is positive; NED down is negative.
                self.publish_velocity_ned(0.0, 0.0, -self.takeoff_speed, 0.0)
            else:
                self.state = STATE_ACTIVE
                self.get_logger().info(
                    f'Takeoff complete at {self.altitude:.2f} m — '
                    f'now following cmd_vel.')

        elif self.state == STATE_ACTIVE:
            self.publish_active_command()

        elif self.state == STATE_LANDING:
            if self.position_valid and self.altitude > 0.3:
                self.publish_velocity_ned(0.0, 0.0, self.landing_speed, 0.0)
            else:
                self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)
                self.disarm()
                self.state = STATE_DISARMED
                self.get_logger().info('Landed and disarmed.')

        self.publish_state()

    def publish_active_command(self):
        """Forward cmd_vel, converted ENU->NED, clamped, and timed out."""
        if self.last_cmd_time is None:
            self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            # Stale command: hold position rather than continuing blindly.
            self.publish_velocity_ned(0.0, 0.0, 0.0, 0.0)
            return

        vx_enu = self.clamp(self.cmd.linear.x, self.max_h_speed)
        vy_enu = self.clamp(self.cmd.linear.y, self.max_h_speed)
        vz_enu = self.clamp(self.cmd.linear.z, self.max_v_speed)
        yaw_rate_enu = self.clamp(self.cmd.angular.z, self.max_yaw_rate)

        # ENU -> NED
        self.publish_velocity_ned(
            north=vy_enu,
            east=vx_enu,
            down=-vz_enu,
            yaw_rate=-yaw_rate_enu,
        )

    # -------------------------------------------------------------- PX4 output

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def publish_velocity_ned(self, north, east, down, yaw_rate):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        # NaN position tells PX4 that position is not being controlled.
        nan = float('nan')
        msg.position = [nan, nan, nan]
        msg.velocity = [float(north), float(east), float(down)]
        msg.acceleration = [nan, nan, nan]
        msg.yaw = nan
        msg.yawspeed = float(yaw_rate)
        self.setpoint_pub.publish(msg)

    def engage_offboard_mode(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )

    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    # ------------------------------------------------------------------ helpers

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    @staticmethod
    def clamp(value, limit):
        if math.isnan(value):
            return 0.0
        return max(-limit, min(limit, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardInterface()
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
