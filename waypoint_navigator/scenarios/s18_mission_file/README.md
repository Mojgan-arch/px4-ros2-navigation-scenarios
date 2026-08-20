# 18 — Run a mission written in a file

Execute a mission described in a YAML file rather than in code.

**When you need it:** Repeatable missions you keep in version control instead of in code.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s18_mission_file/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator mission_file.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `mission_file_executor_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `mission_file_executor_node.py`  - the behaviour itself
- `mission_file.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

The bundled sample is `missions/sample_mission.yaml`. Waypoints support per-point altitude, hover time and an action.

## On real hardware

The mission file path is a parameter; point it anywhere.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator mission_file.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
