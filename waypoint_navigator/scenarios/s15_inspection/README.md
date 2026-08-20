# 15 — Stop at each point and capture

Visit a set of inspection points, hovering at each for a capture interval.

**When you need it:** Structure inspection: towers, solar panels, facades, storage tanks.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s15_inspection/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator inspection.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `inspection_mission_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `inspection_mission_node.py`  - the behaviour itself
- `inspection.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Results are written to a log file. Hover duration per point is configurable.

## On real hardware

Substitute a real camera trigger for the simulated capture.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator inspection.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
