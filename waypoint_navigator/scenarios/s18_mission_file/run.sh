#!/usr/bin/env bash
# Scenario 18: Run a mission written in a file
#
# Execute a mission described in a YAML file rather than in code.
#
# When you need it: Repeatable missions you keep in version control instead of in code.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator mission_file.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario mission_file "$@"
