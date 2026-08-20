#!/usr/bin/env python3
"""
Mission File Executor with Resumption (Stage 18).

Extends the YAML mission executor (stage 17) with:
  - State persistence (each completed waypoint saved to disk)
  - Auto-resume on node restart (if saved state matches mission file)
  - /mission_pause   - triggers RTH and saves state
  - /mission_resume  - explicitly continue from saved state
  - /mission_clear_state - delete saved state, start fresh

Files:
  - missions/sample_mission.yaml  (the mission definition)
  - /tmp/mission_state.json       (current progress, auto-managed)

Workflow (typical):
  1. Run -> reads YAML, detects no saved state -> WAITING_FOR_HOVER
  2. /mission_start -> begins executing
  3. After each waypoint -> saves state
  4. /mission_pause or battery low -> RTH + state saved
  5. Restart node -> detects saved state -> WAITING_RESUME_DECISION
  6. /mission_resume -> continues from next un-done waypoint
  7. Mission complete -> deletes state file
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

import json
import math
import os
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
    """Mission phases."""
    LOADING = 'LOADING'
    WAITING_FOR_HOVER = 'WAITING_FOR_HOVER'
    READY = 'READY'
    WAITING_RESUME_DECISION = 'WAITING_RESUME_DECISION'
    SEND_WP = 'SEND_WP'
    WAIT_NAV = 'WAIT_NAV'
    WAIT_HOVER = 'WAIT_HOVER'
    HOVERING = 'HOVERING'
    ACTION = 'ACTION'
    PAUSE_REQUESTED = 'PAUSE_REQUESTED'
    PAUSED = 'PAUSED'
    SEND_RTH = 'SEND_RTH'
    WAIT_LANDED = 'WAIT_LANDED'
    DONE = 'DONE'
    FAILED = 'FAILED'


class MissionResumable(Node):
    def __init__(self):
        super().__init__('mission_resumable')
        declare_interface_params(self)

        # ========== Parameters ==========
        default_path = os.path.expanduser(
            _default_mission_path()
        )
        self.mission_file = self.declare_parameter(
            'mission_file', default_path
        ).value
        self.state_file = '/tmp/mission_state.json'
        self.goal_send_timeout = 3.0

        # When false, suppress per-waypoint / HOME text labels in RViz.
        self.declare_parameter('show_text_labels', True)
        self.show_text_labels = self.get_parameter('show_text_labels').value

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
        self.log_entries = []
        self.has_saved_state = False
        self.saved_state_info = {}

        # Drone position (ENU) for path / odometry visualisation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.position_received = False
        self._path_msg = Path()
        self._path_msg.header.frame_id = self.frame('map')
        self._last_path_xy = None

        # ========== Subscribers ==========
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.topic('local_position'),
            self.position_callback,
            px4_qos,
        )
        self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_start', self.start_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_pause', self.pause_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_resume', self.resume_callback, 10
        )
        self.create_subscription(
            Bool, '/mission_clear_state', self.clear_state_callback, 10
        )

        # ========== Publishers ==========
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.rth_pub = self.create_publisher(Bool, '/rth_trigger', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        # Visualisation
        self.marker_pub = self.create_publisher(
            MarkerArray, '/mission_markers', 10
        )
        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone_odom', 10)

        # ========== Timers ==========
        self.tick_timer = self.create_timer(0.5, self.tick)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.marker_timer = self.create_timer(0.5, self.publish_markers)
        self.path_timer = self.create_timer(0.1, self.publish_path)
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)

        # Load mission file + check for saved state
        self.load_mission()
        if self.phase != MP.FAILED:
            self.check_for_saved_state()

    # =====================================================
    # Loading
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

        for i, wp in enumerate(raw_wps):
            try:
                x = float(wp['x'])
                y = float(wp['y'])
            except (KeyError, ValueError) as e:
                self.get_logger().error(f'Waypoint {i + 1}: invalid x/y ({e})')
                self.set_phase(MP.FAILED)
                return
            self.waypoints.append({
                'name': wp.get('name', f'WP{i + 1}'),
                'x': x, 'y': y,
                'z': float(wp.get('z', self.default_altitude)),
                'hover_time': float(wp.get('hover_time', self.default_hover_time)),
                'action': wp.get('action', 'pass'),
            })

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'MISSION: {self.mission_name}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info('=' * 60)

    def check_for_saved_state(self):
        if not os.path.exists(self.state_file):
            self.get_logger().info('No saved state - clean start')
            self.set_phase(MP.WAITING_FOR_HOVER)
            return

        try:
            with open(self.state_file, 'r') as f:
                saved = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not read state file: {e}')
            self.set_phase(MP.WAITING_FOR_HOVER)
            return

        # Validate state matches current mission
        if saved.get('mission_file') != self.mission_file:
            self.get_logger().warn(
                'Saved state is for a different mission file - ignoring'
            )
            self.set_phase(MP.WAITING_FOR_HOVER)
            return
        if saved.get('mission_name') != self.mission_name:
            self.get_logger().warn(
                'Saved state mission name differs - ignoring'
            )
            self.set_phase(MP.WAITING_FOR_HOVER)
            return

        idx = int(saved.get('current_index', 0))
        if idx >= len(self.waypoints):
            self.get_logger().warn('Saved state is past end - ignoring')
            self.set_phase(MP.WAITING_FOR_HOVER)
            return

        self.has_saved_state = True
        self.saved_state_info = saved
        self.log_entries = saved.get('completed_waypoints', [])
        self.set_phase(MP.WAITING_FOR_HOVER)

    # =====================================================
    # State persistence
    # =====================================================
    def save_state(self):
        try:
            state = {
                'mission_file': self.mission_file,
                'mission_name': self.mission_name,
                'current_index': self.current_index,
                'completed_waypoints': self.log_entries,
                'saved_at': datetime.now().isoformat(),
                'phase': self.phase,
            }
            with open(self.state_file, 'w') as f:
                json.dump(state, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Save state failed: {e}')

    def delete_state(self):
        if os.path.exists(self.state_file):
            try:
                os.remove(self.state_file)
                self.get_logger().info('State file deleted.')
            except Exception as e:
                self.get_logger().error(f'Delete state failed: {e}')

    # =====================================================
    # Subscribers
    # =====================================================
    def nav_state_callback(self, msg):
        self.current_nav_state = msg.data.split(' ')[0]

    def position_callback(self, msg):
        # PX4 NED -> ENU
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.position_received = True

    def start_callback(self, msg):
        if not msg.data:
            return
        if self.phase != MP.READY:
            self.get_logger().warn(f'Cannot start (phase={self.phase})')
            return
        if self.has_saved_state:
            self.get_logger().warn(
                'A saved state exists. Use /mission_resume or '
                '/mission_clear_state.'
            )
            return
        self.get_logger().info('>>> MISSION STARTED (fresh) <<<')
        self.set_phase(MP.SEND_WP)

    def pause_callback(self, msg):
        if not msg.data:
            return
        if self.phase not in (MP.SEND_WP, MP.WAIT_NAV, MP.WAIT_HOVER,
                              MP.HOVERING, MP.ACTION):
            self.get_logger().warn(
                f'Cannot pause now (phase={self.phase})'
            )
            return
        self.get_logger().warn('>>> PAUSE REQUESTED <<<')
        self.save_state()
        self.set_phase(MP.PAUSE_REQUESTED)

    def resume_callback(self, msg):
        if not msg.data:
            return
        if self.phase != MP.WAITING_RESUME_DECISION:
            self.get_logger().warn(
                f'No resume needed (phase={self.phase})'
            )
            return
        self.current_index = int(self.saved_state_info.get('current_index', 0))
        self.get_logger().info(
            f'>>> RESUMING from WP {self.current_index + 1}/'
            f'{len(self.waypoints)} <<<'
        )
        self.has_saved_state = False
        self.set_phase(MP.SEND_WP)

    def clear_state_callback(self, msg):
        if not msg.data:
            return
        self.delete_state()
        self.has_saved_state = False
        self.saved_state_info = {}
        self.log_entries = []
        self.current_index = 0
        if self.phase == MP.WAITING_RESUME_DECISION:
            self.set_phase(MP.READY)

    # =====================================================
    # Phase tick
    # =====================================================
    def tick(self):
        p = self.phase

        if p == MP.LOADING or p == MP.FAILED:
            return

        elif p == MP.WAITING_FOR_HOVER:
            if self.current_nav_state == 'HOVER':
                if self.has_saved_state:
                    self.announce_resume_decision()
                    self.set_phase(MP.WAITING_RESUME_DECISION)
                else:
                    self.get_logger().info('Drone hovering. Ready.')
                    self.set_phase(MP.READY)

        elif p == MP.READY:
            pass

        elif p == MP.WAITING_RESUME_DECISION:
            pass  # waiting for /mission_resume or /mission_clear_state

        elif p == MP.SEND_WP:
            if self.current_index >= len(self.waypoints):
                self.get_logger().info('All waypoints done.')
                self.delete_state()
                if self.return_home:
                    self.set_phase(MP.SEND_RTH)
                else:
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
                self.set_phase(MP.SEND_WP)

        elif p == MP.WAIT_HOVER:
            if self.current_nav_state == 'HOVER':
                wp = self.waypoints[self.current_index]
                self.get_logger().info(
                    f'Arrived at {wp["name"]}. Hovering {wp["hover_time"]}s'
                )
                self.hover_start_time = time.time()
                self.set_phase(MP.HOVERING)

        elif p == MP.HOVERING:
            wp = self.waypoints[self.current_index]
            elapsed = time.time() - self.hover_start_time
            if elapsed >= wp['hover_time']:
                self.set_phase(MP.ACTION)
            else:
                self.get_logger().info(
                    f'  Hovering at {wp["name"]}: {wp["hover_time"] - elapsed:.1f}s',
                    throttle_duration_sec=1.0
                )

        elif p == MP.ACTION:
            self.do_action()
            self.current_index += 1
            self.save_state()   # <-- persist progress after each waypoint
            self.set_phase(MP.SEND_WP)

        elif p == MP.PAUSE_REQUESTED:
            # Trigger RTH so drone safely returns
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.get_logger().warn('RTH triggered for pause.')
            self.set_phase(MP.PAUSED)

        elif p == MP.PAUSED:
            if self.current_nav_state == 'LANDED':
                self.get_logger().warn('=' * 60)
                self.get_logger().warn('MISSION PAUSED. State saved.')
                self.get_logger().warn(
                    f'Progress: {self.current_index}/{len(self.waypoints)} '
                    f'waypoints done'
                )
                self.get_logger().warn(
                    'Restart this node to resume the mission.'
                )
                self.get_logger().warn('=' * 60)
                self.set_phase(MP.DONE)

        elif p == MP.SEND_RTH:
            msg = Bool()
            msg.data = True
            self.rth_pub.publish(msg)
            self.set_phase(MP.WAIT_LANDED)

        elif p == MP.WAIT_LANDED:
            if self.current_nav_state == 'LANDED':
                self.write_summary()
                self.delete_state()
                self.set_phase(MP.DONE)

        elif p == MP.DONE:
            self.get_logger().info(
                'Done. Press Ctrl+C to exit.',
                throttle_duration_sec=5.0
            )

    def set_phase(self, new_phase):
        if new_phase != self.phase:
            self.get_logger().info(f'MISSION: {self.phase} -> {new_phase}')
            self.phase = new_phase

    # =====================================================
    # Announce resume decision
    # =====================================================
    def announce_resume_decision(self):
        idx = self.saved_state_info.get('current_index', 0)
        saved_at = self.saved_state_info.get('saved_at', 'unknown')
        self.get_logger().info('=' * 60)
        self.get_logger().info('SAVED STATE FOUND')
        self.get_logger().info(f'  Mission:  {self.mission_name}')
        self.get_logger().info(
            f'  Progress: {idx}/{len(self.waypoints)} waypoints done'
        )
        self.get_logger().info(f'  Saved at: {saved_at}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('To RESUME from saved state:')
        self.get_logger().info(
            '  ros2 topic pub --once /mission_resume '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('To CLEAR and start fresh:')
        self.get_logger().info(
            '  ros2 topic pub --once /mission_clear_state '
            'std_msgs/Bool "data: true"'
        )
        self.get_logger().info('=' * 60)

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
            self.get_logger().info(f'  [CAPTURE] {wp["name"]}')
        elif action == 'scan':
            self.get_logger().info(f'  [SCAN] {wp["name"]}')
        else:
            self.get_logger().info(f'  [{action.upper()}] {wp["name"]}')
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
        path = f'/tmp/mission_resumable_{ts}.log'
        try:
            with open(path, 'w') as f:
                f.write(f'Mission Report: {self.mission_name}\n')
                f.write(f'Source: {self.mission_file}\n')
                f.write(f'Date:   {datetime.now().isoformat()}\n')
                f.write(f'Completed: {len(self.log_entries)} waypoints\n')
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
            self.get_logger().error(f'Report failed: {e}')

    # =====================================================
    # Visualisation
    # =====================================================
    def publish_markers(self):
        """Spheres + ground disks + vertical pillars + path line + home."""
        if not self.waypoints:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints):
            x, y, z = wp['x'], wp['y'], wp['z']

            # Sphere at waypoint
            sph = Marker()
            sph.header.frame_id = self.frame('map')
            sph.header.stamp = now
            sph.ns = 'wps'
            sph.id = i
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = x
            sph.pose.position.y = y
            sph.pose.position.z = z
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.5
            if i < self.current_index:
                sph.color.r, sph.color.g, sph.color.b = 0.2, 0.9, 0.2  # green (done)
            elif (i == self.current_index
                  and self.phase in (MP.SEND_WP, MP.WAIT_NAV,
                                     MP.WAIT_HOVER, MP.HOVERING, MP.ACTION)):
                sph.color.r, sph.color.g, sph.color.b = 1.0, 0.5, 0.0  # orange (current)
            else:
                sph.color.r, sph.color.g, sph.color.b = 0.5, 0.5, 0.5  # gray (todo)
            sph.color.a = 0.9
            ma.markers.append(sph)

            # Disk on the ground
            disk = Marker()
            disk.header.frame_id = self.frame('map')
            disk.header.stamp = now
            disk.ns = 'disks'
            disk.id = i
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position.x = x
            disk.pose.position.y = y
            disk.pose.position.z = 0.05
            disk.pose.orientation.w = 1.0
            disk.scale.x = 0.8
            disk.scale.y = 0.8
            disk.scale.z = 0.1
            disk.color.r = sph.color.r
            disk.color.g = sph.color.g
            disk.color.b = sph.color.b
            disk.color.a = 0.6
            ma.markers.append(disk)

            # Vertical pillar
            pillar = Marker()
            pillar.header.frame_id = self.frame('map')
            pillar.header.stamp = now
            pillar.ns = 'pillars'
            pillar.id = i
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.pose.position.x = x
            pillar.pose.position.y = y
            pillar.pose.position.z = z / 2.0
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = 0.05
            pillar.scale.y = 0.05
            pillar.scale.z = z
            pillar.color.r = sph.color.r
            pillar.color.g = sph.color.g
            pillar.color.b = sph.color.b
            pillar.color.a = 0.4
            ma.markers.append(pillar)

            # Text label (skip when show_text_labels:=false)
            if self.show_text_labels:
                t = Marker()
                t.header.frame_id = self.frame('map')
                t.header.stamp = now
                t.ns = 'labels'
                t.id = i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = x
                t.pose.position.y = y
                t.pose.position.z = z + 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.4
                t.color.r = t.color.g = t.color.b = 1.0
                t.color.a = 1.0
                t.text = f"{i + 1}: {wp['name']}"
                ma.markers.append(t)

        # Line strip connecting waypoints
        line = Marker()
        line.header.frame_id = self.frame('map')
        line.header.stamp = now
        line.ns = 'mission_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.r, line.color.g, line.color.b = 0.0, 0.8, 1.0
        line.color.a = 0.7
        # Start from home (0,0,altitude) so the takeoff leg is shown
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = self.default_altitude
        line.points.append(p0)
        for wp in self.waypoints:
            p = Point()
            p.x = wp['x']
            p.y = wp['y']
            p.z = wp['z']
            line.points.append(p)
        ma.markers.append(line)

        # Home marker (red disk on the ground)
        home = Marker()
        home.header.frame_id = self.frame('map')
        home.header.stamp = now
        home.ns = 'home'
        home.id = 0
        home.type = Marker.CYLINDER
        home.action = Marker.ADD
        home.pose.position.x = 0.0
        home.pose.position.y = 0.0
        home.pose.position.z = 0.1
        home.pose.orientation.w = 1.0
        home.scale.x = 1.0
        home.scale.y = 1.0
        home.scale.z = 0.1
        home.color.r = 1.0
        home.color.g = 0.0
        home.color.b = 0.0
        home.color.a = 0.8
        ma.markers.append(home)

        if self.show_text_labels:
            home_t = Marker()
            home_t.header.frame_id = self.frame('map')
            home_t.header.stamp = now
            home_t.ns = 'home_label'
            home_t.id = 0
            home_t.type = Marker.TEXT_VIEW_FACING
            home_t.action = Marker.ADD
            home_t.pose.position.x = 0.0
            home_t.pose.position.y = 0.0
            home_t.pose.position.z = 0.7
            home_t.pose.orientation.w = 1.0
            home_t.scale.z = 0.5
            home_t.color.r = home_t.color.g = home_t.color.b = 1.0
            home_t.color.a = 1.0
            home_t.text = 'HOME'
            ma.markers.append(home_t)

        self.marker_pub.publish(ma)

    def publish_path(self):
        if not self.position_received:
            return
        if self._last_path_xy is not None:
            ldx = self.current_x - self._last_path_xy[0]
            ldy = self.current_y - self._last_path_xy[1]
            if ldx * ldx + ldy * ldy < 0.01:
                self._path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self._path_msg)
                return
        ps = PoseStamped()
        ps.header.frame_id = self.frame('map')
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.current_x
        ps.pose.position.y = self.current_y
        ps.pose.position.z = self.current_z
        ps.pose.orientation.w = 1.0
        self._path_msg.poses.append(ps)
        if len(self._path_msg.poses) > 2000:
            self._path_msg.poses = self._path_msg.poses[-2000:]
        self._last_path_xy = (self.current_x, self.current_y)
        self._path_msg.header.stamp = ps.header.stamp
        self.path_pub.publish(self._path_msg)

    def publish_odometry(self):
        if not self.position_received:
            return
        odom = Odometry()
        odom.header.frame_id = self.frame('map')
        odom.child_frame_id = self.frame('base_link')
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = self.current_z
        yaw = 0.0
        if len(self._path_msg.poses) >= 2:
            a = self._path_msg.poses[-2].pose.position
            b = self._path_msg.poses[-1].pose.position
            yaw = math.atan2(b.y - a.y, b.x - a.x)
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

    def publish_status(self):
        msg = String()
        msg.data = (
            f'{self.phase} | wp={self.current_index}/{len(self.waypoints)} | '
            f'saved={self.has_saved_state}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionResumable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
