# 13 — Map an unknown space by itself

Explore unknown space by flying to the nearest frontier between known-free and unknown cells.

**When you need it:** Entering a building, tunnel, or disaster site with no prior map.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s13_exploration/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator exploration.launch.py
```

World: `worlds/obstacle_course.sdf`

## What runs

| Node | Location |
|---|---|
| `state_machine_node.py` | `waypoint_navigator/common/` |
| `exploration_node.py` | `waypoint_navigator/common/` |

This scenario has no code of its own: it is a specific combination of shared nodes from `waypoint_navigator/common/`. The launch file here is what defines it.

## In this folder

- `__init__.py`
- `exploration.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Frontiers are clustered, and a cluster is chosen by distance and size. Start and stop with `/start_exploration` and `/stop_exploration`.

## On real hardware

The most demanding scenario on perception. Confirm your point cloud is dense and correctly framed before trying it.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator exploration.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
