# 08 — Avoid obstacles nobody mapped

Steer around obstacles seen in the LiDAR cloud while holding a heading.

**When you need it:** Flying where the map is wrong, stale, or missing entirely.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s08_obstacle_avoidance/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator obstacle_avoidance.launch.py
```

World: `worlds/obstacle_course.sdf`

## What runs

| Node | Location |
|---|---|
| `obstacle_avoidance_node.py` | `waypoint_navigator/common/` |

This scenario has no code of its own: it is a specific combination of shared nodes from `waypoint_navigator/common/`. The launch file here is what defines it.

## In this folder

- `__init__.py`
- `obstacle_avoidance.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Purely reactive: no map is built and no path is planned. Sectors of the point cloud are scored and the least obstructed direction is chosen.

## On real hardware

Start with a large safety radius and a low speed. Reactive avoidance has no lookahead and can be trapped by concave obstacles.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator obstacle_avoidance.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
