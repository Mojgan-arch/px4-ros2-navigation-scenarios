#!/usr/bin/env bash
# Scenario 03: Send it somewhere on the map
#
# Fly to a goal published on /goal_pose - in RViz, the 2D Goal Pose tool.
#
# When you need it: An operator picks a destination mid-flight, with no route planned in advance.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator goal.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario goal "$@"
