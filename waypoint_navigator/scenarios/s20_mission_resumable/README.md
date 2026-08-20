# 20 — Resume after an interruption

Pause, resume and recover a mission, including across a node restart.

**When you need it:** Long missions, battery swaps, recovering from a crash or a restart.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s20_mission_resumable/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator mission_resumable.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `mission_resumable_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `mission_resumable_node.py`  - the behaviour itself
- `mission_resumable.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Progress is checkpointed to a JSON state file. Control with `/mission_pause`, `/mission_resume` and `/mission_clear_state`.

## On real hardware

The most operationally useful scenario for long missions. Make sure the state file lands somewhere writable and persistent.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator mission_resumable.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
