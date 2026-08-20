# 16 — Pick something up and drop it off

Fly to a pick-up point, then a drop-off point, with a hold at each.

**When you need it:** Payload transport between two known points.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s16_delivery/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator delivery.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `delivery_mission_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `delivery_mission_node.py`  - the behaviour itself
- `delivery.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

A two-leg mission with state reporting on `/mission_status`.

## On real hardware

Payload actuation is not implemented - the node holds position where a release would occur.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator delivery.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
