# 02 — Fly a preplanned route

Fly a predefined list of waypoints, then return home.

**When you need it:** Survey lines, patrol circuits, repeatable test flights.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s02_waypoint/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator waypoint.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `waypoint_navigator_node.py` | in this folder |

This folder contains all the code unique to this scenario. The only other thing it needs is the vehicle interface in `waypoint_navigator/common/`, which every scenario uses.

## In this folder

- `__init__.py`
- `waypoint_navigator_node.py`  - the behaviour itself
- `waypoint.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

The waypoint list is defined in the node. A waypoint counts as reached when the drone is within a tolerance radius, then the next one is taken.

## On real hardware

Adjust tolerance and cruise speed for your airframe. The defaults assume a small quadrotor in a 10 m arena.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator waypoint.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
