#!/usr/bin/env bash
# Scenario 15: Stop at each point and capture
#
# Visit a set of inspection points, hovering at each for a capture interval.
#
# When you need it: Structure inspection: towers, solar panels, facades, storage tanks.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator inspection.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario inspection "$@"
