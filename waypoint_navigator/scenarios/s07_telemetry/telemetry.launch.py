"""Scenario 07: Record the flight for later analysis

Record position, velocity and battery to CSV during a flight.

When you need it: Tuning parameters, proving a flight happened, reviewing an incident.

    ros2 launch waypoint_navigator telemetry.launch.py

On real hardware, point it at your own platform profile:

    ros2 launch waypoint_navigator telemetry.launch.py \\
        interface_config:=/path/to/my_vehicle.yaml

This scenario runs:
  - state_machine_node
  - geofence_monitor_node
  - visualization_node
  - telemetry_logger_node
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
SCENARIO_NODES = ['state_machine_node', 'geofence_monitor_node', 'visualization_node', 'telemetry_logger_node']


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
