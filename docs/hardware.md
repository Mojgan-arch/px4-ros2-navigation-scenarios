# Running on real hardware

This document describes what has to change to fly these scenarios on a real
aircraft, and what to verify before you do.

> **This stack has not been flown.** It is designed so that the hardware path
> is a configuration change rather than a rewrite, and the abstraction has
> been built with that in mind — but no part of it has been validated in the
> air. Treat everything here as a starting point for your own bring-up, not
> as a tested procedure. Fly in a safe area, with a pilot on the sticks and a
> finger on the mode switch.

---

## 1. The architecture, and why porting is small

Every navigation node in this package is platform-agnostic. They:

- work in **ENU** (x East, y North, z Up), the ROS convention
- publish a `geometry_msgs/Twist` velocity command and a `std_msgs/Bool` arm request
- consume a `sensor_msgs/PointCloud2` and the vehicle's local position
- read **every** topic name and frame id from ROS parameters

A single node, `px4_offboard_interface_node`, converts that into PX4's
protocol: ENU→NED, offboard mode, arming, takeoff, landing, velocity clamping
and command timeouts. It is the only file that imports `px4_msgs`.

That node is unchanged between simulation and hardware. It speaks the same
uORB topics to SITL and to a real autopilot; only the transport differs.

---

## 2. What you need

| Component | Notes |
|---|---|
| PX4 airframe | v1.15+ recommended, so topic names match the defaults |
| Companion computer | Anything that runs ROS 2 Jazzy. Raspberry Pi 5, Jetson Orin Nano, Intel NUC. |
| Serial or Ethernet link | Between the companion computer and the flight controller |
| Micro XRCE-DDS Agent | Running on the companion computer |
| A depth sensor | LiDAR or stereo/depth camera, publishing `PointCloud2` |
| A pose source | See §5. Outdoors GPS may suffice; indoors it will not. |

---

## 3. Configuration

Copy `config/topics_hardware.yaml`, edit it, and pass it
to any scenario:

```bash
ros2 launch waypoint_navigator waypoint.launch.py \
    interface_config:=/home/pi/my_vehicle.yaml
```

### 3.1 Confirm the PX4 topic names

PX4 v1.15 introduced versioned output topics. Check what yours publishes:

```bash
ros2 topic list | grep /fmu/out
```

If you see `vehicle_local_position` without the `_v1` suffix, set both
`topics.local_position` and the interface node's `local_position_topic`
accordingly.

### 3.2 Point the stack at your sensor

Set `topics.pointcloud` to your driver's output — `/livox/lidar`,
`/ouster/points`, `/camera/depth/color/points`, whatever it is. Confirm the
cloud is actually arriving and is in a sensible frame:

```bash
ros2 topic hz /livox/lidar
ros2 run tf2_ros tf2_echo base_link livox_frame
```

### 3.3 Set the flight envelope

The defaults are permissive because simulation is forgiving. Real airframes
are not. At minimum, set these on `px4_offboard_interface_node`:

```yaml
px4_offboard_interface:
  ros__parameters:
    takeoff_altitude: 1.5        # start low
    max_horizontal_speed: 1.0    # start slow
    max_vertical_speed: 0.5
    max_yaw_rate: 0.5
    cmd_timeout: 0.3
```

Raise them only after each scenario behaves as you expect.

---

## 4. Frames

The navigation nodes publish and expect data in `frames.map`. In simulation
that frame coincides with the PX4 local origin and drifts negligibly.

On hardware you have three honest options:

1. **`frames.map: odom`** — no external pose source. The stack works, but
   position drifts with the EKF. Acceptable for short outdoor flights with
   GPS; not acceptable for indoor missions that depend on returning to a
   previously visited point.

2. **`frames.map: map`, with a SLAM package publishing `map`** — the correct
   setup for indoor or GPS-denied flight. See §5.

3. **`frames.map` set to a motion-capture frame** — the easiest way to get a
   trustworthy pose in a lab.

Do not leave `frames.map: map` configured if nothing publishes a `map` frame.
TF lookups will fail and the visualization will be empty or wrong.

---

## 5. Localization and SLAM

**This package does not implement SLAM.** It builds a local occupancy grid
from point clouds against a pose it is given; it does not estimate that pose,
and it performs no loop closure or global optimisation.

For GPS-denied flight you need a real SLAM or odometry package. The usual
choices with a LiDAR are [FAST-LIO](https://github.com/hku-mars/FAST_LIO),
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) or
[RTAB-Map](https://github.com/introlab/rtabmap_ros); with a stereo camera,
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) or
RTAB-Map again.

The integration has two halves, and both matter:

**Feed the pose to PX4.** Publish the SLAM pose to PX4's external vision
interface (`VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`) and set
PX4's `EKF2_EV_CTRL` parameters so the EKF fuses it. This is what makes PX4's
own position estimate trustworthy indoors — and every scenario here reads its
position from PX4, so this step is not optional.

**Point the stack at the right frame.** Set `frames.map` to whatever your
SLAM package publishes as its map frame.

Getting external vision into PX4 correctly is its own project, with its own
failure modes around timestamps, frame conventions and covariances. Do that
first, verify it in `ros2 topic echo` and in flight logs, and only then bring
up these scenarios on top.

---

## 6. Safety

Read this section before the first flight.

- **Configure PX4's own failsafes.** RC loss, data-link loss, low battery,
  geofence. These are enforced by the flight controller and are the only ones
  that survive a companion-computer crash.

- **`geofence_monitor_node` is not containment.** It runs on the companion
  computer. If that computer or the network fails, it stops enforcing
  anything. Configure PX4's geofence as well, always.

- **`cmd_timeout` is not a safety system.** It makes the aircraft hover when
  commands stop arriving, which is better than continuing on a stale command.
  It does nothing if the aircraft is already commanded into a wall.

- **Keep a pilot on the sticks.** Offboard mode is exited by switching modes
  on the transmitter. Make sure the pilot knows which switch and has tested it
  on the ground.

- **Test the abort path first.** Before any autonomous scenario, verify in the
  air that flipping out of offboard mode returns manual control immediately.

- **Bring the envelope up slowly.** Start at 1 m altitude and 0.5 m/s in a
  large open space. Every scenario in this repository was tuned in a 10×10 m
  simulated arena; the numbers are not automatically right for your airframe.

---

## 7. Suggested bring-up order

Work through the scenarios in increasing order of autonomy, verifying each
before moving on:

1. **Scenario 01, take off / hover / land** (`s01_state_machine`) — nothing
   else. This validates the interface node, the topic wiring and your failsafes.
2. **Scenario 02, fly a preplanned route** (`s02_waypoint`) — a short square at
   low altitude. Validates position control and the ENU/NED conversion.
3. **Scenario 04, bring it home on low battery** (`s04_rth`) — validates
   return-to-home and the battery trigger.
4. **Scenario 05, keep it inside a permitted area** (`s05_geofence`) — with
   PX4's own geofence set slightly wider as a backstop.
5. **Scenario 08, avoid obstacles nobody mapped** (`s08_obstacle_avoidance`) —
   the first scenario that depends on perception. Test against a single large
   static obstacle before anything cluttered.
6. **Scenarios 10, 12 and 13** (plan a route, cover an area, map an unknown
   space) — only once everything above is reliable.

If a scenario misbehaves, check in this order: is the point cloud arriving and
in the right frame; is the pose sane; is `cmd_vel` what you expect
(`ros2 topic echo /nav/cmd_vel`); is the interface node in the state you think
(`ros2 topic echo /nav/vehicle_state`).

---

## 8. Porting to a non-PX4 platform

Replace `px4_offboard_interface_node.py` with a node that:

- subscribes to `topics.cmd_vel` (`geometry_msgs/Twist`, ENU)
- subscribes to `topics.arm` (`std_msgs/Bool`)
- publishes `topics.local_position` in the form the navigation nodes expect,
  or adapts them via a small shim
- translates all of that into your platform's protocol

The other twenty-one nodes need no changes. A MAVROS variant, for instance,
would map `cmd_vel` onto `/mavros/setpoint_velocity/cmd_vel_unstamped` and
arming onto the `/mavros/cmd/arming` service.
