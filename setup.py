"""Package definition for px4-ros2-navigation-scenarios.

The package is laid out by scenario rather than by layer: everything that
belongs to one scenario -- its node, its launch file, its README and its demo
script -- lives in a single folder under waypoint_navigator/scenarios/.

Code shared by more than one scenario lives in waypoint_navigator/common/,
so that a fix is made once rather than repeated across folders.
"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'waypoint_navigator'


def scenario_launch_files():
    """Collect every scenario launch file into share/<pkg>/launch/.

    Launch files live beside the node they start, but ros2 launch expects to
    find them in one place, so they are gathered at install time.
    """
    files = glob('waypoint_navigator/common/*.launch.py')
    files += glob('waypoint_navigator/scenarios/*/*.launch.py')
    return files


setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), scenario_launch_files()),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description=(
        'A catalog of autonomous navigation scenarios for PX4 drones on ROS 2. '
        'Mission logic is platform-agnostic; all contact with the flight '
        'controller is isolated in px4_offboard_interface_node.'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- shared: vehicle interface and reusable behaviours ---
            'px4_offboard_interface_node = waypoint_navigator.common.px4_offboard_interface_node:main',
            'state_machine_node = waypoint_navigator.common.state_machine_node:main',
            'geofence_monitor_node = waypoint_navigator.common.geofence_monitor_node:main',
            'exploration_node = waypoint_navigator.common.exploration_node:main',
            'obstacle_avoidance_node = waypoint_navigator.common.obstacle_avoidance_node:main',
            'visualization_node = waypoint_navigator.common.visualization_node:main',

            # --- scenario-specific behaviours ---
            # 02 Waypoint mission
            'waypoint_navigator_node = waypoint_navigator.scenarios.s02_waypoint.waypoint_navigator_node:main',
            # 03 Click-to-goal
            'goal_navigator_node = waypoint_navigator.scenarios.s03_goal.goal_navigator_node:main',
            # 04 Return to home
            'rth_navigator_node = waypoint_navigator.scenarios.s04_rth.rth_navigator_node:main',
            # 07 Telemetry logging
            'telemetry_logger_node = waypoint_navigator.scenarios.s07_telemetry.telemetry_logger_node:main',
            # 09 Obstacle course traversal
            'obstacle_course_mission_node = waypoint_navigator.scenarios.s09_obstacle_course.obstacle_course_mission_node:main',
            # 10 Grid path planning
            'path_planner_node = waypoint_navigator.scenarios.s10_path_planner.path_planner_node:main',
            # 11 Path preview and confirm
            'path_planner_preview_node = waypoint_navigator.scenarios.s11_path_preview.path_planner_preview_node:main',
            # 12 Area coverage
            'coverage_planner_node = waypoint_navigator.scenarios.s12_coverage.coverage_planner_node:main',
            # 14 Persistent occupancy map
            'persistent_map_node = waypoint_navigator.scenarios.s14_persistent_map.persistent_map_node:main',
            # 15 Inspection mission
            'inspection_mission_node = waypoint_navigator.scenarios.s15_inspection.inspection_mission_node:main',
            # 16 Delivery mission
            'delivery_mission_node = waypoint_navigator.scenarios.s16_delivery.delivery_mission_node:main',
            # 17 Target following
            'target_simulator_node = waypoint_navigator.scenarios.s17_target_follow.target_simulator_node:main',
            'target_follower_node = waypoint_navigator.scenarios.s17_target_follow.target_follower_node:main',
            # 18 YAML mission execution
            'mission_file_executor_node = waypoint_navigator.scenarios.s18_mission_file.mission_file_executor_node:main',
            # 19 Interactive mission editor
            'mission_editor_node = waypoint_navigator.scenarios.s19_mission_editor.mission_editor_node:main',
            # 20 Resumable mission
            'mission_resumable_node = waypoint_navigator.scenarios.s20_mission_resumable.mission_resumable_node:main',
        ],
    },
)
