# 10 — Plan a route around known obstacles

Plan a path around obstacles on an occupancy grid, then follow it.

**When you need it:** The obstacles are already mapped and a straight line will not work.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s10_path_planner/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator path_planner.launch.py
```

World: `worlds/obstacle_course.sdf`

## What runs

| Node | Location |
|---|---|
| `path_planner_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `path_planner_node.py`  - the behaviour itself
- `path_planner.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

The occupancy grid is built from accumulated point clouds. Planning is A* over grid cells; the result is published on `/planned_path`.

## On real hardware

Grid resolution and extent are the parameters that matter most - both trade memory against fidelity.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator path_planner.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
