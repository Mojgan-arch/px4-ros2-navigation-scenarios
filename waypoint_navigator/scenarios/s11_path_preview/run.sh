#!/usr/bin/env bash
# Scenario 11: Let a human approve the route first
#
# Plan a path, display it, and wait for an operator to confirm or reject.
#
# When you need it: Supervised autonomy: flying near people or expensive equipment.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator path_preview.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-obstacle_course}"

run_scenario path_preview "$@"
