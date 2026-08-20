#!/usr/bin/env bash
# Shared configuration and helpers for the scenario demo scripts.
#
# These scripts are a convenience layer for running a scenario in simulation:
# they start PX4, the DDS agent, the Gazebo bridge and the scenario itself in
# a multi-pane terminal so you can watch all of it at once.
#
# They are NOT the deployment mechanism. On real hardware you run the launch
# file directly:
#
#     ros2 launch waypoint_navigator waypoint.launch.py \
#         interface_config:=/path/to/my_vehicle.yaml
#
# Everything below can be overridden from the environment, e.g.
#
#     ROS_DISTRO=humble WORLD=obstacle_course \
#         ./waypoint_navigator/scenarios/s02_waypoint/run.sh

set -euo pipefail

# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------

# Repository root, resolved from this file's location — no hard-coded paths.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# The colcon workspace this package was built in. If you cloned into
# ~/ros2_ws/src/px4-ros2-navigation-scenarios, this resolves to ~/ros2_ws.
WS_DIR="${WS_DIR:-$(cd "$REPO_DIR/../.." 2>/dev/null && pwd || echo "$REPO_DIR")}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# PX4-Autopilot checkout, used to start SITL.
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

# Optional. Left unset, QGroundControl simply is not started.
QGC_PATH="${QGC_PATH:-}"

# --------------------------------------------------------------------------
# Simulation bring-up
# --------------------------------------------------------------------------

# Which world to load. Worlds ship in the repository's worlds/ directory.
WORLD="${WORLD:-walled_arena}"

# Airframe model. x500_depth carries a depth sensor; x500 does not.
PX4_MODEL="${PX4_MODEL:-gz_x500_depth}"

# The command that brings up the simulator. Override this to use a different
# simulation stack entirely — the scenarios do not care what starts Gazebo,
# only that PX4 is reachable over Micro XRCE-DDS afterwards.
SIM_LAUNCH_CMD="${SIM_LAUNCH_CMD:-}"

default_sim_launch_cmd() {
    cat <<EOF
cd "$PX4_DIR" && \
PX4_GZ_WORLD=$WORLD \
GZ_SIM_RESOURCE_PATH="\$GZ_SIM_RESOURCE_PATH:$REPO_DIR/worlds" \
make px4_sitl $PX4_MODEL
EOF
}

sim_launch_cmd() {
    if [ -n "$SIM_LAUNCH_CMD" ]; then
        echo "$SIM_LAUNCH_CMD"
    else
        default_sim_launch_cmd
    fi
}

# PX4 resolves PX4_GZ_WORLD against its OWN worlds directory and does not
# consult GZ_SIM_RESOURCE_PATH for it, so a world that lives in this repository
# is invisible to it until it appears there. Symlink ours in — a link rather
# than a copy, so editing worlds/*.sdf here takes effect immediately, and
# nothing is duplicated inside the PX4 checkout.
link_worlds_into_px4() {
    local dest="$PX4_DIR/Tools/simulation/gz/worlds"

    if [ ! -d "$dest" ]; then
        echo "WARNING: PX4 worlds directory not found at $dest." >&2
        echo "         '$WORLD' will not be found unless you place it there." >&2
        return 0
    fi

    local w name
    for w in "$REPO_DIR"/worlds/*.sdf; do
        [ -e "$w" ] || continue
        name="$(basename "$w")"

        if [ -L "$dest/$name" ] && [ "$(readlink -f "$dest/$name")" = "$(readlink -f "$w")" ]; then
            continue
        fi

        if [ -e "$dest/$name" ] && [ ! -L "$dest/$name" ]; then
            echo "NOTE: $dest/$name already exists and was left untouched." >&2
            continue
        fi

        if ln -sfn "$w" "$dest/$name" 2>/dev/null; then
            echo "Linked $name into the PX4 worlds directory."
        else
            echo "WARNING: could not link $name into $dest (not writable?)." >&2
            echo "         Copy it there by hand, or set SIM_LAUNCH_CMD." >&2
        fi
    done
}

# Micro XRCE-DDS agent: the PX4 <-> ROS 2 transport.
DDS_AGENT_CMD="${DDS_AGENT_CMD:-MicroXRCEAgent udp4 -p 8888}"

# Bridges the Gazebo LiDAR onto the topic the navigation stack expects.
# Adjust the Gazebo-side topic to match your world and airframe.
GZ_LIDAR_TOPIC="${GZ_LIDAR_TOPIC:-/world/$WORLD/model/x500_depth_0/link/lidar_sensor_link/sensor/lidar/scan/points}"
BRIDGE_CMD="${BRIDGE_CMD:-ros2 run ros_gz_bridge parameter_bridge $GZ_LIDAR_TOPIC@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r $GZ_LIDAR_TOPIC:=/nav/points}"

# --------------------------------------------------------------------------
# Environment
# --------------------------------------------------------------------------

ros_env() {
    cat <<EOF
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
EOF
}

check_prerequisites() {
    local missing=0
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ERROR: ros2 not found. Source /opt/ros/$ROS_DISTRO/setup.bash first." >&2
        missing=1
    fi
    if [ ! -f "$WS_DIR/install/setup.bash" ]; then
        echo "WARNING: no build found at $WS_DIR/install." >&2
        echo "         Run: cd $WS_DIR && colcon build --packages-select waypoint_navigator" >&2
    fi
    if [ -z "$SIM_LAUNCH_CMD" ] && [ ! -d "$PX4_DIR" ]; then
        echo "ERROR: PX4-Autopilot not found at $PX4_DIR." >&2
        echo "       Set PX4_DIR, or set SIM_LAUNCH_CMD to your own bring-up command." >&2
        missing=1
    fi
    if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
        echo "WARNING: MicroXRCEAgent not on PATH; PX4 will not reach ROS 2." >&2
    fi
    return $missing
}

# --------------------------------------------------------------------------
# Terminal multiplexing — Terminator if present, otherwise tmux
# --------------------------------------------------------------------------

detect_terminal() {
    if [ -n "${NAV_TERMINAL:-}" ]; then
        echo "$NAV_TERMINAL"
    elif command -v terminator >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
        echo terminator
    elif command -v tmux >/dev/null 2>&1; then
        echo tmux
    else
        echo none
    fi
}

# run_panes <session-name> <title1> <cmd1> [<title2> <cmd2> ...]
#
# Starts each command in its own pane. Under Terminator this opens a grid in a
# new window; under tmux it creates a detached session you attach to. Neither
# path touches the user's own terminator or tmux configuration.
run_panes() {
    local session="$1"; shift
    local term; term="$(detect_terminal)"

    local titles=() cmds=()
    while [ "$#" -gt 0 ]; do
        titles+=("$1"); cmds+=("$2"); shift 2
    done

    case "$term" in
        terminator) run_panes_terminator "$session" ;;
        tmux)       run_panes_tmux "$session" ;;
        none)
            echo "ERROR: neither terminator nor tmux is installed." >&2
            echo "       Install one of them, or run the commands below by hand:" >&2
            local i
            for i in "${!titles[@]}"; do
                echo >&2
                echo "  # ${titles[$i]}" >&2
                echo "  ${cmds[$i]}" >&2
            done
            return 1
            ;;
    esac
}

# Both implementations read the arrays set by run_panes.
run_panes_terminator() {
    local session="$1"
    local workdir; workdir="$(mktemp -d "/tmp/${session}.XXXXXX")"
    local config="$workdir/terminator_config"

    local i child_entries=""
    for i in "${!titles[@]}"; do
        local runner="$workdir/pane_$i.sh"
        {
            echo '#!/usr/bin/env bash'
            echo "echo '=== ${titles[$i]} ==='"
            ros_env
            echo "${cmds[$i]}"
            echo 'exec bash'
        } > "$runner"
        chmod +x "$runner"
    done

    # A dedicated config file in a temp dir: the user's own
    # ~/.config/terminator/config is never read or written.
    #
    # Terminator's layout is a binary tree: a Window accepts exactly ONE child,
    # and every split is a Paned node with exactly two children.
    local n=${#titles[@]}
    {
        echo '[global_config]'
        echo '[keybindings]'
        echo '[profiles]'
        echo '  [[default]]'
        echo '    scrollback_infinite = True'
        echo '[layouts]'
        echo '  [[default]]'
        echo '    [[[window0]]]'
        echo '      type = Window'
        echo '      parent = ""'
        echo '      order = 0'
        if [ "$n" -eq 1 ]; then
            echo '    [[[terminal0]]]'
            echo '      type = Terminal'
            echo '      parent = window0'
            echo '      order = 0'
            echo "      command = \"$workdir/pane_0.sh\""
            echo "      title = \"${titles[0]}\""
        else
            local parent ptype
            for ((i = 0; i < n - 1; i++)); do
                if [ "$i" -eq 0 ]; then parent="window0"; else parent="paned$((i - 1))"; fi
                if [ $((i % 2)) -eq 0 ]; then ptype="HPaned"; else ptype="VPaned"; fi
                echo "    [[[paned$i]]]"
                echo "      type = $ptype"
                echo "      parent = $parent"
                if [ "$i" -eq 0 ]; then echo '      order = 0'; else echo '      order = 1'; fi
                echo "    [[[terminal$i]]]"
                echo '      type = Terminal'
                echo "      parent = paned$i"
                echo '      order = 0'
                echo "      command = \"$workdir/pane_$i.sh\""
                echo "      title = \"${titles[$i]}\""
            done
            echo "    [[[terminal$((n - 1))]]]"
            echo '      type = Terminal'
            echo "      parent = paned$((n - 2))"
            echo '      order = 1'
            echo "      command = \"$workdir/pane_$((n - 1)).sh\""
            echo "      title = \"${titles[$((n - 1))]}\""
        fi
        echo '[plugins]'
    } > "$config"

    echo "Starting ${#titles[@]} panes in Terminator (config: $config)"
    terminator -g "$config" -l default &
}

run_panes_tmux() {
    local session="$1"
    tmux kill-session -t "$session" 2>/dev/null || true

    local envsetup; envsetup="$(ros_env)"
    tmux new-session -d -s "$session" -n main
    local i
    for i in "${!titles[@]}"; do
        if [ "$i" -gt 0 ]; then
            tmux split-window -t "$session:main"
            tmux select-layout -t "$session:main" tiled >/dev/null
        fi
        tmux send-keys -t "$session:main.$i" \
            "clear; echo '=== ${titles[$i]} ==='; $envsetup; ${cmds[$i]}" C-m
    done
    tmux select-layout -t "$session:main" tiled >/dev/null

    echo "Started tmux session '$session' with ${#titles[@]} panes."
    echo "Attach with:  tmux attach -t $session"
    echo "Stop with:    tmux kill-session -t $session"
    if [ -z "${NAV_NO_ATTACH:-}" ] && [ -t 1 ]; then
        tmux attach -t "$session"
    fi
}

# --------------------------------------------------------------------------
# Scenario entry point
# --------------------------------------------------------------------------

# run_scenario <launch-file-basename> [extra launch args...]
run_scenario() {
    local scenario="$1"; shift
    local extra_args="$*"

    check_prerequisites || return 1

    # Only meaningful for the default PX4 bring-up; a custom SIM_LAUNCH_CMD is
    # responsible for finding its own world.
    if [ -z "$SIM_LAUNCH_CMD" ]; then
        link_worlds_into_px4
    fi

    echo "Scenario : $scenario"
    echo "World    : $WORLD"
    echo "Workspace: $WS_DIR"
    echo "ROS      : $ROS_DISTRO"
    echo

    local panes=(
        "PX4 SITL + Gazebo"  "$(sim_launch_cmd)"
        "Micro XRCE-DDS"     "sleep 8; $DDS_AGENT_CMD"
        "Gazebo bridge"      "sleep 12; $BRIDGE_CMD"
        "Scenario"           "sleep 16; ros2 launch waypoint_navigator ${scenario}.launch.py $extra_args"
    )

    if [ -n "$QGC_PATH" ] && [ -x "$QGC_PATH" ]; then
        panes+=("QGroundControl" "sleep 20; $QGC_PATH")
    fi

    run_panes "nav_$scenario" "${panes[@]}"
}
