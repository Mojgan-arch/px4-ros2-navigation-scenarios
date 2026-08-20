"""Shared bring-up for every scenario in this package.

This launch file starts the parts that are common to all scenarios: the
vehicle interface, and optionally RViz. Individual scenario launch files
include it and then add their own behaviour nodes.

Nothing here is simulation-specific. The only thing that changes between
simulation and hardware is which interface profile is loaded:

    # simulation (default)
    ros2 launch waypoint_navigator bringup.launch.py

    # real vehicle
    ros2 launch waypoint_navigator bringup.launch.py \\
        interface_config:=/path/to/my_vehicle.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'waypoint_navigator'


def default_interface_config():
    return os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'topics_sim.yaml')


def default_rviz_config():
    return os.path.join(
        get_package_share_directory(PACKAGE), 'rviz', 'nav_scenarios.rviz')


def declare_common_arguments():
    """Launch arguments shared by bringup and every scenario launch file."""
    return [
        DeclareLaunchArgument(
            'interface_config',
            default_value=default_interface_config(),
            description=(
                'YAML file mapping the stack onto a platform. Defaults to the '
                'PX4 SITL profile; pass a hardware profile to retarget.'),
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with this package\'s display config.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config(),
            description='RViz configuration file.'),
        DeclareLaunchArgument(
            'use_vehicle_interface', default_value='true',
            description=(
                'Start px4_offboard_interface_node. Set false if another '
                'component already provides the cmd_vel/arm interface.')),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='2.5',
            description='Altitude in metres the vehicle climbs to after arming.'),
    ]


def generate_launch_description():
    interface_config = LaunchConfiguration('interface_config')

    vehicle_interface = Node(
        package=PACKAGE,
        executable='px4_offboard_interface_node',
        name='px4_offboard_interface',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_vehicle_interface')),
        parameters=[
            interface_config,
            {'takeoff_altitude': LaunchConfiguration('takeoff_altitude')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(
        declare_common_arguments() + [vehicle_interface, rviz])
