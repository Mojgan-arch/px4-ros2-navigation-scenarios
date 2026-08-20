"""Shared interface configuration for every node in this package.

This module is the single place where the stack's connection to the outside
world is defined: which topics it speaks on, and which TF frames it works in.
No node hard-codes a topic name or a frame id; they all read from here, and
every value is a ROS parameter that can be overridden at launch.

That is what makes the same navigation code run against simulation today and
real hardware later. To retarget the stack you change a YAML file, not source:

    ros2 launch waypoint_navigator waypoint.launch.py \\
        interface_config:=/path/to/my_robot_topics.yaml

Two ready-made profiles ship in `config/`:

    topics_sim.yaml       PX4 SITL + Gazebo, using this package's own
                          px4_offboard_interface node (the default)
    topics_hardware.yaml  a real PX4 airframe with an external LiDAR and,
                          optionally, an external SLAM package supplying pose

Frame convention
----------------
All navigation nodes work in ENU (x=East, y=North, z=Up), the ROS standard.
Conversion to PX4's NED happens only in px4_offboard_interface_node.
"""

# Interface defaults. These are deliberately platform-neutral names: nothing
# here mentions a simulator, a vendor, or a specific LiDAR. The simulation
# profile in config/topics_sim.yaml maps them onto whatever the sim provides.
DEFAULT_TOPICS = {
    # --- commands out to the vehicle -------------------------------------
    'cmd_vel': '/nav/cmd_vel',
    'arm': '/nav/arm',

    # --- state in from the vehicle ---------------------------------------
    'local_position': '/fmu/out/vehicle_local_position_v1',
    'battery': '/fmu/out/battery_status_v1',

    # --- perception in ----------------------------------------------------
    'pointcloud': '/nav/points',

    # --- navigation interface ---------------------------------------------
    'goal_pose': '/goal_pose',
    'clicked_point': '/clicked_point',
    'nav_state': '/nav_state',
}

DEFAULT_FRAMES = {
    'map': 'map',
    'odom': 'odom',
    'base_link': 'base_link',
}


def declare_interface_params(node):
    """Declare the shared topic and frame parameters on `node`.

    Call once, immediately after ``super().__init__()``. Afterwards the node
    can use ``node.topic('cmd_vel')`` and ``node.frame('map')``.

    Parameters are declared as ``topics.<name>`` and ``frames.<name>`` so a
    YAML override file stays readable:

        /**:
          ros__parameters:
            topics:
              cmd_vel: /mavros/setpoint_velocity/cmd_vel_unstamped
            frames:
              map: odom
    """
    for name, default in DEFAULT_TOPICS.items():
        param = f'topics.{name}'
        if not node.has_parameter(param):
            node.declare_parameter(param, default)

    for name, default in DEFAULT_FRAMES.items():
        param = f'frames.{name}'
        if not node.has_parameter(param):
            node.declare_parameter(param, default)

    # Bind the accessors onto the node instance.
    node.topic = lambda key: _get(node, 'topics', key, DEFAULT_TOPICS)
    node.frame = lambda key: _get(node, 'frames', key, DEFAULT_FRAMES)


def _get(node, group, key, defaults):
    param = f'{group}.{key}'
    if node.has_parameter(param):
        return node.get_parameter(param).value
    return defaults[key]
