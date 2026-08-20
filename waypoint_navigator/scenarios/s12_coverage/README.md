# 12 — Cover an entire area

Sweep a rectangular area in a boustrophedon (lawnmower) pattern.

**When you need it:** Field survey, search sweep, photogrammetry, spraying.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s12_coverage/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator coverage.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `coverage_planner_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `coverage_planner_node.py`  - the behaviour itself
- `coverage.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

The pattern is published as markers on `/coverage_pattern` before flight, so you can inspect the sweep before it starts.

## On real hardware

Line spacing should follow from your sensor's ground footprint, not from the defaults here.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator coverage.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
