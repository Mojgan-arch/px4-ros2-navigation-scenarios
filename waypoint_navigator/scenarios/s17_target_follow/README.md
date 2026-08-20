# 17 — Track a moving subject

Follow a moving target while maintaining a standoff distance.

**When you need it:** Following a vehicle, a person, or livestock.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s17_target_follow/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator target_follow.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `target_simulator_node.py` | in this folder |
| `target_follower_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `target_follower_node.py`  - the behaviour itself
- `target_simulator_node.py`  - the behaviour itself
- `target_follow.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

`target_simulator_node` publishes a synthetic moving target on `/target_position`.

## On real hardware

Drop `target_simulator_node` and publish `/target_position` from your own vision pipeline. Nothing else changes.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator target_follow.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
