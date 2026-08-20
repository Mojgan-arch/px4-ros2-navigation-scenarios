#!/usr/bin/env bash
# Scenario 10: Plan a route around known obstacles
#
# Plan a path around obstacles on an occupancy grid, then follow it.
#
# When you need it: The obstacles are already mapped and a straight line will not work.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator path_planner.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-obstacle_course}"

run_scenario path_planner "$@"
