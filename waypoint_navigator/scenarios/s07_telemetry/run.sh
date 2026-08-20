#!/usr/bin/env bash
# Scenario 07: Record the flight for later analysis
#
# Record position, velocity and battery to CSV during a flight.
#
# When you need it: Tuning parameters, proving a flight happened, reviewing an incident.
#
# Runs the scenario in simulation: PX4 SITL, the DDS agent, the Gazebo bridge
# and the scenario launch file, in a multi-pane terminal.
#
# On real hardware do not use this script. Run the launch file directly:
#
#     ros2 launch waypoint_navigator telemetry.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/scripts/common.sh"

WORLD="${WORLD:-walled_arena}"

run_scenario telemetry "$@"
