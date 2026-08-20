"""Scenario 02: Fly a preplanned route

Fly a predefined list of waypoints, then return home.

When you need it: Survey lines, patrol circuits, repeatable test flights.

    ros2 launch waypoint_navigator waypoint.launch.py

On real hardware, point it at your own platform profile:

    ros2 launch waypoint_navigator waypoint.launch.py \\
        interface_config:=/path/to/my_vehicle.yaml

This scenario runs:
  - waypoint_navigator_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'waypoint_navigator'

# Executables this scenario starts, in order.
SCENARIO_NODES = ['waypoint_navigator_node']


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    interface_config = LaunchConfiguration('interface_config')

    arguments = [
        DeclareLaunchArgument(
            'interface_config',
            default_value=os.path.join(share, 'config', 'topics_sim.yaml'),
            description='Platform interface profile (topics and frames).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz alongside the scenario.'),
    ]

    # Shared bring-up: the vehicle interface, and optionally RViz.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'interface_config': interface_config,
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    nodes = [
        Node(
            package=PACKAGE,
            executable=executable,
            name=executable.removesuffix('_node'),
            output='screen',
            parameters=[interface_config],
        )
        for executable in SCENARIO_NODES
    ]

    return LaunchDescription(arguments + [bringup] + nodes)
