# 05 — Keep it inside a permitted area

Enforce a boundary and react when the drone approaches or crosses it.

**When you need it:** Flying near property lines, restricted airspace, or people.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s05_geofence/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator geofence.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `state_machine_node.py` | `waypoint_navigator/common/` |
| `geofence_monitor_node.py` | `waypoint_navigator/common/` |

This scenario has no code of its own: it is a specific combination of shared nodes from `waypoint_navigator/common/`. The launch file here is what defines it.

## In this folder

- `__init__.py`
- `geofence.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Status is published on `/geofence_status`. The fence is a rectangular prism defined by parameters.

## On real hardware

**Do not rely on this for containment.** It runs on the companion computer, so it stops enforcing anything the moment that computer or the network fails. Configure PX4's own geofence as well.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator geofence.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
