# 14 — Reuse the map from last time

Explore, save the occupancy grid to disk, and reload it on the next run.

**When you need it:** Repeat visits to the same site, without re-exploring it every flight.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s14_persistent_map/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator persistent_map.launch.py
```

World: `worlds/obstacle_course.sdf`

## What runs

| Node | Location |
|---|---|
| `persistent_map_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |
| `exploration_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `persistent_map_node.py`  - the behaviour itself
- `persistent_map.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Save with `/save_map`, load with `/load_map`. The map is stored as JSON.

## On real hardware

The saved grid is tied to whatever frame `frames.map` names. Reloading it only means something if that frame is repeatable between flights - which, without SLAM, it generally is not.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator persistent_map.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
