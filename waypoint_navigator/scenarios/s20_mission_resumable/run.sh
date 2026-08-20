#!/usr/bin/env bash
# Scenario 20: Resume after an interruption
#
# Pause, resume and recover a mission, including across a node restart.
#
# When you need it: Long missions, battery swaps, recovering from a crash or a restart.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator mission_resumable.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario mission_resumable "$@"
