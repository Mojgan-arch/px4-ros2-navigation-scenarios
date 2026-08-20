#!/usr/bin/env bash
# Scenario 14: Reuse the map from last time
#
# Explore, save the occupancy grid to disk, and reload it on the next run.
#
# When you need it: Repeat visits to the same site, without re-exploring it every flight.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator persistent_map.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-obstacle_course}"

run_scenario persistent_map "$@"
