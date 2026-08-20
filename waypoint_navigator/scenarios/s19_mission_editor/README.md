# 19 — Plan the mission in the field

Build a mission by clicking points in RViz, then fly it.

**When you need it:** Deciding where to fly once you are on site, without editing files.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s19_mission_editor/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator mission_editor.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `mission_editor_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `mission_editor_node.py`  - the behaviour itself
- `mission_editor.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Add points with RViz's **Publish Point** tool, clear with `/mission_clear`, start with `/mission_start`.

## On real hardware

Useful for quick field missions without editing files.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator mission_editor.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
