# px4-ros2-navigation-scenarios

**Twenty autonomous flight scenarios for PX4 drones on ROS 2 — each one a self-contained folder you can read, run and lift out on its own.**

Most PX4 + ROS 2 examples show you one thing: how to send an offboard setpoint. This repository is the layer above that — a catalog of *behaviours*: return-to-home, frontier exploration, coverage sweeps, obstacle slaloms, resumable missions, target following.

Every scenario lives in its own folder with its code, its launch file, its README and a one-command demo script. If you only care about scenario 15, you read one folder.

Simulation is the primary runnable environment. The navigation logic contains no simulation-specific assumptions, so the same stack points at a real airframe by swapping a YAML profile rather than editing code.

> **Status:** simulation-tested. The hardware path is designed for but has not been flown. Read [docs/hardware.md](docs/hardware.md) first.

<!-- Add a GIF here: scenario 13 (exploration) is the most striking one. -->

---

## The scenarios

Each row is a situation a drone ends up in, and the behaviour that handles it. Ordered from simplest to most demanding — which is also the order recommended for bringing this up on a real vehicle.

| # | What it does for you | When you need it | World |
|---|---|---|---|
| **01** | [Take off, hover, and land safely](waypoint_navigator/scenarios/s01_state_machine/) | Your first flight with any new airframe: it validates the whole control chain before you add autonomy on top | `walled_arena` |
| **02** | [Fly a preplanned route](waypoint_navigator/scenarios/s02_waypoint/) | Survey lines, patrol circuits, repeatable test flights | `walled_arena` |
| **03** | [Send it somewhere on the map](waypoint_navigator/scenarios/s03_goal/) | An operator picks a destination mid-flight, with no route planned in advance | `walled_arena` |
| **04** | [Bring it home when the battery runs low](waypoint_navigator/scenarios/s04_rth/) | Any flight far enough away that you cannot simply walk over and pick it up | `walled_arena` |
| **05** | [Keep it inside a permitted area](waypoint_navigator/scenarios/s05_geofence/) | Flying near property lines, restricted airspace, or people | `walled_arena` |
| **06** | [See what the drone is doing](waypoint_navigator/scenarios/s06_visualization/) | Debugging a mission, or showing someone else what is happening in real time | `walled_arena` |
| **07** | [Record the flight for later analysis](waypoint_navigator/scenarios/s07_telemetry/) | Tuning parameters, proving a flight happened, reviewing an incident | `walled_arena` |
| **08** | [Avoid obstacles nobody mapped](waypoint_navigator/scenarios/s08_obstacle_avoidance/) | Flying where the map is wrong, stale, or missing entirely | `obstacle_course` |
| **09** | [Get through cluttered space](waypoint_navigator/scenarios/s09_obstacle_course/) | Warehouses, forests, construction sites, urban canyons | `obstacle_course` |
| **10** | [Plan a route around known obstacles](waypoint_navigator/scenarios/s10_path_planner/) | The obstacles are already mapped and a straight line will not work | `obstacle_course` |
| **11** | [Let a human approve the route first](waypoint_navigator/scenarios/s11_path_preview/) | Supervised autonomy: flying near people or expensive equipment | `obstacle_course` |
| **12** | [Cover an entire area](waypoint_navigator/scenarios/s12_coverage/) | Field survey, search sweep, photogrammetry, spraying | `walled_arena` |
| **13** | [Map an unknown space by itself](waypoint_navigator/scenarios/s13_exploration/) | Entering a building, tunnel, or disaster site with no prior map | `obstacle_course` |
| **14** | [Reuse the map from last time](waypoint_navigator/scenarios/s14_persistent_map/) | Repeat visits to the same site, without re-exploring it every flight | `obstacle_course` |
| **15** | [Stop at each point and capture](waypoint_navigator/scenarios/s15_inspection/) | Structure inspection: towers, solar panels, facades, storage tanks | `walled_arena` |
| **16** | [Pick something up and drop it off](waypoint_navigator/scenarios/s16_delivery/) | Payload transport between two known points | `walled_arena` |
| **17** | [Track a moving subject](waypoint_navigator/scenarios/s17_target_follow/) | Following a vehicle, a person, or livestock | `walled_arena` |
| **18** | [Run a mission written in a file](waypoint_navigator/scenarios/s18_mission_file/) | Repeatable missions you keep in version control instead of in code | `walled_arena` |
| **19** | [Plan the mission in the field](waypoint_navigator/scenarios/s19_mission_editor/) | Deciding where to fly once you are on site, without editing files | `walled_arena` |
| **20** | [Resume after an interruption](waypoint_navigator/scenarios/s20_mission_resumable/) | Long missions, battery swaps, recovering from a crash or a restart | `walled_arena` |

Each link goes to a folder containing everything for that scenario.

---

## Quick start

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/px4-ros2-navigation-scenarios.git
git clone https://github.com/PX4/px4_msgs.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs waypoint_navigator
source install/setup.bash

# run scenario 01
./src/px4-ros2-navigation-scenarios/waypoint_navigator/scenarios/s01_state_machine/run.sh
```

Then arm it:

```bash
ros2 topic pub -1 /nav/arm std_msgs/Bool "{data: true}"
```

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| PX4-Autopilot | v1.15+ |
| Gazebo | Harmonic |
| Micro XRCE-DDS Agent | latest |
| `px4_msgs` | matching your PX4 |
| Terminator *or* tmux | either |

---

## How the repository is organised

```
px4-ros2-navigation-scenarios/
├── waypoint_navigator/
│   ├── common/                     code used by more than one scenario
│   │   ├── px4_offboard_interface_node.py   ← the only PX4-aware file
│   │   ├── interfaces.py                    ← topic and frame config
│   │   ├── state_machine_node.py            ← used by 15 scenarios
│   │   ├── geofence_monitor_node.py         ← used by 3
│   │   ├── exploration_node.py              ← used by 2
│   │   ├── obstacle_avoidance_node.py       ← used by 2
│   │   ├── visualization_node.py            ← used by 2
│   │   └── bringup.launch.py
│   └── scenarios/
│       ├── s01_state_machine/      README + launch + run.sh
│       ├── s02_waypoint/           README + node + launch + run.sh
│       ├── …
│       └── s20_mission_resumable/
├── config/     interface profiles: simulation and hardware
├── worlds/     walled_arena.sdf, obstacle_course.sdf
├── rviz/       display configuration
├── missions/   sample mission YAML
├── scripts/    common.sh — shared shell config, Terminator/tmux detection
└── docs/       hardware.md — the real-vehicle checklist
```

**Why `common/` exists.** Sixteen of the twenty-one nodes belong to exactly one scenario, and those live in their scenario's folder. Five are genuinely shared — `state_machine_node` alone is used by fifteen scenarios. Copying it into fifteen folders would mean fixing every bug fifteen times, so shared code is kept in one place and each scenario's README says which shared nodes it uses.

---

## Architecture

The single design rule is that **only one file knows what a PX4 is**.

```
┌──────────────────────────────────────────────────────────────┐
│  Scenario behaviours          waypoint_navigator/scenarios/  │
│  waypoints · goals · RTH · avoidance · planning · coverage   │
│  exploration · inspection · delivery · geofence · following  │
│                                                              │
│  Platform-agnostic. Works in ENU. Knows nothing about PX4.   │
└───────────────┬──────────────────────────────────────────────┘
                │  geometry_msgs/Twist   (velocity command, ENU)
                │  std_msgs/Bool         (arm request)
                ▼
┌──────────────────────────────────────────────────────────────┐
│  Vehicle interface   common/px4_offboard_interface_node.py   │
│  ENU→NED · offboard mode · arm/disarm · takeoff · landing    │
│  command timeout · velocity clamping                         │
│                                                              │
│  The ONLY file that imports px4_msgs.                        │
└───────────────┬──────────────────────────────────────────────┘
                │  px4_msgs over Micro XRCE-DDS
                ▼
    simulation: PX4 SITL + Gazebo   │   hardware: real PX4
```

Two consequences, and they are the whole point:

1. **No scenario node contains a hard-coded topic name or frame id.** Every one is a ROS parameter, declared in [`common/interfaces.py`](waypoint_navigator/common/interfaces.py) and supplied by a YAML profile at launch.
2. **Retargeting is a configuration change.** Simulation and hardware differ by which profile you load, not by which code runs.

Porting to a non-PX4 platform means replacing one file and leaving the other twenty alone.

---

## A note on localization and SLAM

Worth being precise about, because the distinction is often blurred.

The scenarios take vehicle state from PX4's onboard EKF, via `vehicle_local_position` over Micro XRCE-DDS. They consume LiDAR point clouds to build a **local occupancy grid** for obstacle avoidance and planning, and scenario 14 can save and reload that grid.

That is occupancy mapping against a known pose. It is **not** SLAM: nothing here estimates the vehicle's pose from sensor data, and nothing performs loop closure or global map optimisation.

For true SLAM, run a package that provides it — [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [RTAB-Map](https://github.com/introlab/rtabmap_ros), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — and feed its pose to PX4 as an external vision estimate. This stack then runs on top unchanged. The integration point is documented in [docs/hardware.md](docs/hardware.md).

---

## Running on real hardware

**Read [docs/hardware.md](docs/hardware.md) in full before flying.** In short:

| Concern | Simulation | Hardware |
|---|---|---|
| Point cloud | Gazebo LiDAR bridged to `/nav/points` | your driver — set `topics.pointcloud` |
| Vehicle state | PX4 SITL over UDP | real PX4 over serial or Ethernet |
| Localization | PX4 EKF | PX4 EKF, plus an external pose source indoors |
| `map` frame | PX4 origin | whatever your pose source publishes |
| Flight envelope | permissive defaults | set speed and altitude limits for your airframe |
| Failsafes | a command timeout only | configure PX4's own failsafes properly |

The `run.sh` scripts are for simulation. On hardware you run the launch file:

```bash
ros2 launch waypoint_navigator waypoint.launch.py \
    interface_config:=/path/to/my_vehicle.yaml
```

Start from [`config/topics_hardware.yaml`](config/topics_hardware.yaml), which is commented for exactly this.

---

## Contributing

Issues and pull requests welcome, particularly:

- Hardware bring-up reports — which airframe, what needed changing
- Interface profiles for other platforms (MAVROS, ArduPilot)
- Scenarios that expose a gap in the abstraction

A new scenario is a new folder under `waypoint_navigator/scenarios/` with a node, a launch file, a README and a `run.sh`. Copy the closest existing one.

## License

_Not yet chosen. Until a license file is added, no permissions are granted._

## Acknowledgements

Developed on top of an internal research workspace. The navigation and mission logic here is original work; PX4, ROS 2 and Gazebo are external dependencies.
