#!/usr/bin/env python3
"""
Mission File Executor (Stage 17).

Reads a YAML mission file and executes it through the state machine.

YAML format (see missions/sample_mission.yaml):
  mission:
    name: <str>
    default_altitude: <float>
    default_hover_time: <float>
    return_home_at_end: <bool>
  waypoints:
    - name: <str>
      x: <float>
      y: <float>
      z: <float>           # optional, defaults to default_altitude
      hover_time: <float>  # optional, defaults to default_hover_time
      action: <str>        # capture | scan | pass

Trigger:
  ros2 topic pub --once /mission_start std_msgs/Bool "data: true"
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

import os
import sys
import time
import yaml
from datetime import datetime
from ...common.interfaces import declare_interface_params


def _default_mission_path():
    """Locate the bundled sample mission inside the installed package."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('waypoint_navigator'),
            'missions', 'sample_mission.yaml')
    except Exception:
        return os.path.join(os.getcwd(), 'missions', 'sample_mission.yaml')



class MP:
    """Mission execution phases."""
    LOADING = 'LOADING'
    WAITING_FOR_HOVER = 'WAITING_FOR_HOVER'
    READY = 'READY'
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    HOVERING = 'HOVERING'
    ACTION = 'ACTION'
    NEXT_WP = 'NEXT_WP'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'
    FAILED = 'FAILED'


class MissionFileExecutor(Node):
    def __init__(self):
        super().__init__('mission_file_executor')
        declare_interface_params(self)

        # ========== Parameters ==========
        default_path = os.path.expanduser(
            _default_mission_path()
        )
        self.mission_file = self.declare_parameter(
            'mission_file', default_path
        ).value
        self.goal_send_timeout = 3.0

        # ========== State ==========
        self.mission_data = None
        self.waypoints = []
        self.mission_name = ''
        self.default_altitude = 2.0
        self.default_hover_time = 2.0
        self.return_home = True

        self.current_nav_state = 'UNKNOWN'
        self.phase = MP.LOADING
        self.current_index = 0
        self.hover_start_time = None
        self.send_attempt_time = None
        self.mission_started = False
        self.log_entries = []

        # ========== Subscribers ==========
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_start', self.start_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Load mission file
        self.load_mission()

    # =====================================================
    # YAML loading
    # =====================================================
    def load_mission(self):
        if not os.path.exists(self.mission_file):
            self.get_logger().error(
                f'Mission file not found: {self.mission_file}'
            )
            self.set_phase(MP.FAILED)
            return

        try:
            with open(self.mission_file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'YAML parse error: {e}')
            self.set_phase(MP.FAILED)
            return

        if not isinstance(data, dict):
            self.get_logger().error('Invalid YAML structure')
            self.set_phase(MP.FAILED)
            return

        mission_cfg = data.get('mission', {})
        self.mission_name = mission_cfg.get('name', '(unnamed)')
        self.default_altitude = float(mission_cfg.get('default_altitude', 2.0))
        self.default_hover_time = float(mission_cfg.get('default_hover_time', 2.0))
        self.return_home = bool(mission_cfg.get('return_home_at_end', True))

        raw_wps = data.get('waypoints', [])
        if not raw_wps:
            self.get_logger().error('No waypoints in mission file')
            self.set_phase(MP.FAILED)
            return

        self.waypoints = []
        for i, wp in enumerate(raw_wps):
            try:
                x = float(wp['x'])
                y = float(wp['y'])
            except (KeyError, ValueError) as e:
                self.get_logger().error(
                    f'Waypoint {i + 1}: invalid x/y ({e})'
                )
                self.set_phase(MP.FAILED)
                return
            z = float(wp.get('z', self.default_altitude))
            name = wp.get('name', f'WP{i + 1}')
            hover_time = float(wp.get('hover_time', self.default_hover_time))
            action = wp.get('action', 'pass')
            self.waypoints.append({
                'name': name,
                'x': x, 'y': y, 'z': z,
                'hover_time': hover_time,
                'action': action,
            })

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'MISSION: {self.mission_name}')
        self.get_logger().info(f'  File:      {self.mission_file}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Altitude:  {self.default_altitude}m (default)')
        self.get_logger().info(f'  Hover:     {self.default_hover_time}s (default)')
        self.get_logger().info(f'  RTH end:   {self.return_home}')
        self.get_logger().info('-' * 60)
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f'  {i + 1:2d}. {wp["name"]:20s} '
                f'({wp["x"]:6.2f}, {wp["y"]:6.2f}, {wp["z"]:5.2f}) '
                f'hover={wp["hover_time"]:.1f}s '
                f'action={wp["action"]}'
            )
        self.get_logger().info('=' * 60)
        self.get_logger().info('Trigger with:')
        self.get_logger().info(
            '  ros2 topic pub --once /mission_start '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 60)
        self.set_phase(MP.WAITING_FOR_HOVER)

    # =====================================================
    # Subscribers
    # =====================================================
    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    def start_callback(self, msg):
        if not msg.data:
            return
        if self.phase != MP.READY:
            self.get_logger().warn(
                f'Cannot start (phase={self.phase}). '
                'Wait until drone is HOVERing.'
            )
            return
        self.mission_started = True
        self.get_logger().info('>>> MISSION STARTED <<<')
        self.set_phase(MP.SEND_WP)

    # =====================================================
    # Phase tick
    # =====================================================
    def tick(self):
        p = self.phase

        if p == MP.LOADING:
            return

        elif p == MP.WAITING_FOR_HOVER:
            if self.current_nav_state == 'HOVER':
                self.get_logger().info('Drone hovering. Ready to start.')
                self.set_phase(MP.READY)
            else:
                self.get_logger().info(
                    f'Waiting for HOVER... state: {self.current_nav_state}',
                    throttle_duration_sec=2.0
                )

        elif p == MP.READY:
            pass  # waiting for /mission_start

        elif p == MP.SEND_WP:
            if self.current_index >= len(self.waypoints):
                if self.return_home:
                    self.set_phase(MP.SEND_RTH)
                else:
                    self.get_logger().info('Mission done (no RTH).')
                    self.write_summary()
                    self.set_phase(MP.DONE)
                return
            wp = self.waypoints[self.current_index]
            self.send_goal(wp['x'], wp['y'], wp['z'], wp['name'])
            self.send_attempt_time = time.time()
            self.set_phase(MP.WAIT_NAV)

        elif p == MP.WAIT_NAV:
            if self.current_nav_state == 'NAVIGATING':
                self.set_phase(MP.WAIT_HOVER)
            elif time.time() - self.send_attempt_time > self.goal_send_timeout:
                self.get_logger().warn('Goal not picked up - resending')
                self.set_phase(MP.SEND_WP)

        elif p == MP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                wp = self.waypoints[self.current_index]
                self.get_logger().info(
                    f'Arrived at {wp["name"]}. Hovering {wp["hover_time"]}s...'
                )
                self.hover_start_time = time.time()
                self.set_phase(MP.HOVERING)

        elif p == MP.HOVERING:
            wp = self.waypoints[self.current_index]
            elapsed = time.time() - self.hover_start_time
            remaining = wp['hover_time'] - elapsed
            if remaining <= 0:
                self.set_phase(MP.ACTION)
            else:
                self.get_logger().info(
                    f'  Hovering at {wp["name"]}: {remaining:.1f}s',
                    throttle_duration_sec=1.0
                )

        elif p == MP.ACTION:
            self.do_action()
            self.current_index += 1
            self.set_phase(MP.SEND_WP)

        elif p == MP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().info('All waypoints done. RTH triggered.')
            self.set_phase(MP.WAIT_LANDED)

        elif p == MP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.write_summary()
                self.set_phase(MP.DONE)

        elif p == MP.DONE:
            self.get_logger().info(
                'Mission complete. Press Ctrl+C to exit.',
                throttle_duration_sec=5.0
            )

        elif p == MP.FAILED:
            self.get_logger().error(
                'Mission FAILED at load time. Fix YAML and restart node.',
                throttle_duration_sec=5.0
            )

    def set_phase(self, new_phase):
        if new_phase != self.phase:
            self.get_logger().info(f'MISSION: {self.phase} -> {new_phase}')
            self.phase = new_phase

    # =====================================================
    # Actions
    # =====================================================
    def do_action(self):
        wp = self.waypoints[self.current_index]
        action = wp['action']
        entry = {
            'index': self.current_index + 1,
            'waypoint': wp['name'],
            'action': action,
            'x': wp['x'], 'y': wp['y'], 'z': wp['z'],
            'timestamp': datetime.now().isoformat(),
        }
        if action == 'capture':
            self.get_logger().info(
                f'  [CAPTURE] {wp["name"]} - photo logged'
            )
        elif action == 'scan':
            self.get_logger().info(
                f'  [SCAN] {wp["name"]} - extended hover'
            )
        elif action == 'pass':
            self.get_logger().info(
                f'  [PASS] {wp["name"]} - continuing'
            )
        else:
            self.get_logger().warn(
                f'  [UNKNOWN ACTION: {action}] - treating as pass'
            )
        self.log_entries.append(entry)

    # =====================================================
    # Helpers
    # =====================================================
    def send_goal(self, x, y, z, name=''):
        goal = PoseStamped()
        goal.header.frame_id = self.frame('map')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'>>> WP {self.current_index + 1}/{len(self.waypoints)}: '
            f'{name} ({x:.2f}, {y:.2f}, {z:.2f})'
        )

    def write_summary(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = f'/tmp/mission_{ts}.log'
        try:
            with open(path, 'w') as f:
                f.write(f'Mission Report: {self.mission_name}\n')
                f.write(f'Source: {self.mission_file}\n')
                f.write(f'Date:   {datetime.now().isoformat()}\n')
                f.write(f'Waypoints completed: {len(self.log_entries)}\n')
                f.write('=' * 60 + '\n')
                for e in self.log_entries:
                    f.write(
                        f'#{e["index"]:2d} | {e["waypoint"]:20s} | '
                        f'{e["action"]:8s} | '
                        f'({e["x"]:.2f}, {e["y"]:.2f}, {e["z"]:.2f}) | '
                        f'{e["timestamp"]}\n'
                    )
            self.get_logger().info(f'Report saved: {path}')
        except Exception as e:
            self.get_logger().error(f'Report write failed: {e}')

    def publish_status(self):
        msg = String()
        if self.phase in (MP.SEND_WP, MP.WAIT_NAV, MP.WAIT_HOVER,
                          MP.HOVERING, MP.ACTION):
            wp_name = ''
            if self.current_index < len(self.waypoints):
                wp_name = self.waypoints[self.current_index]['name']
            msg.data = (
                f'{self.phase} | '
                f'wp={self.current_index + 1}/{len(self.waypoints)} | '
                f'{wp_name}'
            )
        else:
            msg.data = f'{self.phase} | mission={self.mission_name}'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionFileExecutor()
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
