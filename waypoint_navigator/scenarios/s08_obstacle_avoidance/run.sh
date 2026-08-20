#!/usr/bin/env bash
# Scenario 08: Avoid obstacles nobody mapped
#
# Steer around obstacles seen in the LiDAR cloud while holding a heading.
#
# When you need it: Flying where the map is wrong, stale, or missing entirely.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator obstacle_avoidance.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-obstacle_course}"

run_scenario obstacle_avoidance "$@"
