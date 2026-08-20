#!/usr/bin/env bash
# Scenario 04: Bring it home when the battery runs low
#
# Return to the launch point on command, or when the battery gets low.
#
# When you need it: Any flight far enough away that you cannot simply walk over and pick it up.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator rth.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario rth "$@"
