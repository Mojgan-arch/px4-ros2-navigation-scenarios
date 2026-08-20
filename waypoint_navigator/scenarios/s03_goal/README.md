# 03 — Send it somewhere on the map

Fly to a goal published on /goal_pose - in RViz, the 2D Goal Pose tool.

**When you need it:** An operator picks a destination mid-flight, with no route planned in advance.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s03_goal/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator goal.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `goal_navigator_node.py` | in this folder |

This folder contains all the code unique to this scenario. The only other thing it needs is the vehicle interface in `waypoint_navigator/common/`, which every scenario uses.

## In this folder

- `__init__.py`
- `goal_navigator_node.py`  - the behaviour itself
- `goal.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

RViz publishes a `PoseStamped` with z=0, so the node substitutes its own cruise altitude.

## On real hardware

Works unchanged. The goal can come from anything that publishes `PoseStamped` - a GCS, a script, another planner.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator goal.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
