# 07 — Record the flight for later analysis

Record position, velocity and battery to CSV during a flight.

**When you need it:** Tuning parameters, proving a flight happened, reviewing an incident.

## Run it

```bash
# simulation, one command
./waypoint_navigator/scenarios/s07_telemetry/run.sh

# or launch it directly (this is what runs on hardware)
ros2 launch waypoint_navigator telemetry.launch.py
```

World: `worlds/walled_arena.sdf`

## What runs

| Node | Location |
|---|---|
| `telemetry_logger_node.py` | in this folder |
| `state_machine_node.py` | `waypoint_navigator/common/` |
| `geofence_monitor_node.py` | `waypoint_navigator/common/` |
| `visualization_node.py` | `waypoint_navigator/common/` |

This folder contains the code that is unique to this scenario. It also uses shared nodes from `waypoint_navigator/common/` - see the table above - and the vehicle interface (`common/px4_offboard_interface_node.py`), which every scenario needs.

## In this folder

- `__init__.py`
- `telemetry_logger_node.py`  - the behaviour itself
- `telemetry.launch.py`  - how the scenario is launched
- `run.sh`  - one-command simulation demo

## How it works

Writes a CSV and a summary text file per flight.

## On real hardware

Point the output directory at persistent storage; the default is a temporary location that will not survive a reboot.

Then read [the hardware checklist](../../../docs/hardware.md) before flying.

## Configuration

This scenario reads its topics and frames from the interface profile, like
every other one. Nothing here is hard-coded:

```bash
ros2 launch waypoint_navigator telemetry.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

---

[← all scenarios](../../../README.md)
