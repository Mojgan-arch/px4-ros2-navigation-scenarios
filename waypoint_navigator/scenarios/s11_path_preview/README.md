# 11 — Let a human approve the route first

Plan a path, display it, and wait for an operator to confirm or reject.

**When you need it:** Supervised autonomy: flying near people or expensive equipment.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s11_path_preview/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator path_preview.launch.py
```

World: `worlds/obstacle_course.sdf`

## What runs

| Node | Location |
|---|---|
| `path_planner_preview_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `path_planner_preview_node.py`  - the behaviour itself
- `path_preview.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Confirm with `ros2 topic pub -1 /confirm_path std_msgs/Bool "{data: true}"`, reject on `/reject_path`.

## On real hardware

A useful pattern for supervised autonomy: the operator stays in the loop without having to fly manually.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator path_preview.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
