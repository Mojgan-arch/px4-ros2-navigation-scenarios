#!/usr/bin/env bash
# Scenario 13: Map an unknown space by itself
#
# Explore unknown space by flying to the nearest frontier between known-free and unknown cells.
#
# When you need it: Entering a building, tunnel, or disaster site with no prior map.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator exploration.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-obstacle_course}"

run_scenario exploration "$@"
