#!/usr/bin/env bash
# Scenario 06: See what the drone is doing
#
# Publish RViz markers for vehicle state, geofence and flown path.
#
# When you need it: Debugging a mission, or showing someone else what is happening in real time.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator visualization.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario visualization "$@"
