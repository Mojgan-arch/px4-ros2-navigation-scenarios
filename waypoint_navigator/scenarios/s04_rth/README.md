# 04 — Bring it home when the battery runs low

Return to the launch point on command, or when the battery gets low.

**When you need it:** Any flight far enough away that you cannot simply walk over and pick it up.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s04_rth/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator rth.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `rth_navigator_node.py` | in this folder |

This folder contains all the code unique to this scenario. The only other thing it needs is the vehicle interface in `waypoint_navigator/common/`, which every scenario uses.

## In this folder

- `__init__.py`
- `rth_navigator_node.py`  - the behaviour itself
- `rth.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Trigger manually with `ros2 topic pub -1 /rth_trigger std_msgs/Bool "{data: true}"`. The automatic trigger reads the battery topic.

## On real hardware

Verify your battery telemetry is real and calibrated before trusting the automatic trigger.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator rth.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
