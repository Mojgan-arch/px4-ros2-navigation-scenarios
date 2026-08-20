# 01 — Take off, hover, and land safely

The arm - takeoff - navigate - land cycle on its own, with no mission on top.

**When you need it:** Your first flight with any new airframe: it validates the whole control chain before you add autonomy on top.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s01_state_machine/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator state_machine.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `state_machine_node.py` | `waypoint_navigator/common/` |

This scenario has no code of its own: it is a specific combination of shared nodes from `waypoint_navigator/common/`. The launch file here is what defines it.

## In this folder

- `__init__.py`
- `state_machine.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Publishes its current state on `/nav_state`. Fifteen of the twenty scenarios run this node underneath their own mission logic, which is why it lives in `common/` rather than here.

## On real hardware

This is the right first scenario to fly. It exercises the vehicle interface, the topic wiring and your failsafes without any autonomy on top.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator state_machine.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
