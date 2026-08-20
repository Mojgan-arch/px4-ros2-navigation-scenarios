# 06 — See what the drone is doing

Publish RViz markers for vehicle state, geofence and flown path.

**When you need it:** Debugging a mission, or showing someone else what is happening in real time.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s06_visualization/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator visualization.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `state_machine_node.py` | `waypoint_navigator/common/` |
| `geofence_monitor_node.py` | `waypoint_navigator/common/` |
| `visualization_node.py` | `waypoint_navigator/common/` |

This scenario has no code of its own: it is a specific combination of shared nodes from `waypoint_navigator/common/`. The launch file here is what defines it.

## In this folder

- `__init__.py`
- `visualization.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Markers go to `/viz_markers`, the flown path to `/drone_path`, odometry to `/drone_odom`.

## On real hardware

Marker frames follow `frames.map`, so this reflects whatever frame your pose source defines.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator visualization.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
