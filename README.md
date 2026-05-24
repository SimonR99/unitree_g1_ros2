# unitree_g1_ros2

ROS 2 Foxy bridge that exposes the Unitree G1 humanoid through **standard ROS 2
interfaces** — `sensor_msgs/Imu`, `sensor_msgs/JointState`, `nav_msgs/Odometry`,
`geometry_msgs/Twist`, `sensor_msgs/Image` — so RViz, Nav2,
`teleop_twist_keyboard`, `robot_localization`, etc. work out of the box.

The package is **standalone**: it ships the G1 29-DOF URDF and meshes, runs
`robot_state_publisher` for you, and emits the static TFs the URDF doesn't
declare (camera optical frames, `base_link → pelvis`).

## What it publishes / subscribes

| Direction | Topic                            | Type                       | Source / Sink                                            |
|-----------|----------------------------------|----------------------------|----------------------------------------------------------|
| pub       | `/joint_states`                  | `sensor_msgs/JointState`   | `rt/lowstate` motor positions/velocities/effort          |
| pub       | `/imu/data`                      | `sensor_msgs/Imu`          | `rt/lowstate` IMU                                        |
| pub       | `/odom`                          | `nav_msgs/Odometry`        | `rt/sportmodestate` position/velocity                    |
| pub       | `/tf`                            | TF                         | `odom→base_link`, `pelvis→imu_link`                      |
| pub       | `/tf_static`, `/robot_description` | TF + URDF                | `robot_state_publisher` over the bundled `g1_29dof.urdf` |
| pub       | `/camera/color/image_raw` + info | `sensor_msgs/Image`        | on-board RealSense (optional)                            |
| pub       | `/camera/depth/image_raw` + info | `sensor_msgs/Image`        | on-board RealSense (optional)                            |
| sub       | `/cmd_vel`                       | `geometry_msgs/Twist`      | → `LocoClient.Move(vx, vy, vyaw)`                        |
| sub       | `/g1/enable`                     | `std_msgs/Bool`            | safety latch for `/cmd_vel` (optional)                   |
| sub       | `/g1/loco/set_fsm_id`            | `std_msgs/Int32`           | → `LocoClient.SetFsmId(...)`                             |
| sub       | `/g1/loco/set_balance_mode`      | `std_msgs/Int32`           | → `LocoClient.SetBalanceMode(...)`                       |
| sub       | `/g1/loco/set_stand_height`      | `std_msgs/Float32`         | → `LocoClient.SetStandHeight(...)`                       |
| srv       | `/g1/loco/{start,damp,sit,zero_torque,squat_to_stand,lie_to_stand,stand_to_squat,high_stand,low_stand,stop_move}` | `std_srvs/srv/Trigger` | named FSM transitions (see below) |

The Unitree firmware already publishes the LiDAR point cloud on
`/utlidar/cloud_livox_mid360` (frame `livox_frame`); the bundled URDF declares
that frame, so RViz can render the cloud as soon as the bridge is up.

## Prerequisites

- ROS 2 Foxy
- The Unitree DDS stack set up per `../unitree_ros2/README.md` (cyclonedds, etc.)
- `unitree_sdk2_python` installed and importable (already present in
  `../unitree_sdk2_python` — `pip install -e ../unitree_sdk2_python` if needed)
- Optional for cameras: `pip install --user pyrealsense2`

## Build

```bash
cd ~/workspaces/unitree_g1_ros2
source /opt/ros/foxy/setup.bash
# Source cyclonedds env if your Unitree setup needs it:
# source ~/workspaces/unitree_ros2/setup.sh
colcon build --symlink-install --packages-select g1_ros2_bridge
source install/setup.bash
```

Tell the SDK which interface the robot is on:

```bash
export G1_INTERFACE=eth0   # or eno2, enp3s0, etc.
```

## Run

### On the robot — bridges + URDF + TF (recommended)

```bash
ros2 launch g1_ros2_bridge robot.launch.py
# with the on-board RealSense:
ros2 launch g1_ros2_bridge robot.launch.py enable_realsense:=true
# safe dry-run (logs Twist but does not move the robot):
ros2 launch g1_ros2_bridge robot.launch.py dry_run:=true
```

This brings up the three bridges (`state`, `odom`, `cmd_vel`),
`robot_state_publisher` (with the bundled `g1_29dof.urdf`), and the static TFs
that aren't expressed in the URDF (camera optical frames, `base_link → pelvis`).
Anything else on the same `ROS_DOMAIN_ID` immediately sees a complete TF tree.

### On a laptop — visualization only

The laptop just sees the topics over the network (same `ROS_DOMAIN_ID`):

```bash
ros2 launch g1_ros2_bridge laptop.launch.py
```

This launches RViz with the in-tree config. `/robot_description` and
`/tf_static` come from the robot, so no local URDF is needed. If for some
reason the robot is started with `enable_description:=false`, pass
`enable_description:=true` here to publish the URDF locally.

### Single host (robot + viz on the same machine)

```bash
ros2 launch g1_ros2_bridge all_in_one.launch.py
```

## Bringing up the FSM (`/g1/loco/...`)

The robot only accepts `/cmd_vel` once it's in **FSM 500** ("Start" / balance).
The `loco_bridge` (started by `robot.launch.py` by default) wraps the SDK's
`LocoClient` and exposes every FSM transition as a ROS 2 service or topic, so
you don't need the Unitree app:

```bash
# Stand up from a lying pose:
ros2 service call /g1/loco/lie_to_stand std_srvs/srv/Trigger

# Or from a squat:
ros2 service call /g1/loco/squat_to_stand std_srvs/srv/Trigger

# Enter balance / locomotion mode (required for /cmd_vel to move the robot):
ros2 service call /g1/loco/start std_srvs/srv/Trigger

# Sit / damp / zero-torque if you want to power down gracefully:
ros2 service call /g1/loco/sit std_srvs/srv/Trigger
ros2 service call /g1/loco/damp std_srvs/srv/Trigger
ros2 service call /g1/loco/zero_torque std_srvs/srv/Trigger
```

Standing height is a topic (continuous parameter):

```bash
ros2 topic pub --once /g1/loco/set_stand_height std_msgs/msg/Float32 "{data: 0.78}"
ros2 service call /g1/loco/high_stand std_srvs/srv/Trigger
ros2 service call /g1/loco/low_stand  std_srvs/srv/Trigger
```

For an arbitrary FSM ID not exposed as a named service:

```bash
ros2 topic pub --once /g1/loco/set_fsm_id std_msgs/msg/Int32 "{data: 500}"
```

The full mapping is:

| Service                         | SDK call                | FSM ID |
|---------------------------------|-------------------------|--------|
| `/g1/loco/zero_torque`          | `ZeroTorque()`          | 0      |
| `/g1/loco/damp`                 | `Damp()`                | 1      |
| `/g1/loco/sit`                  | `Sit()`                 | 3      |
| `/g1/loco/start`                | `Start()` (balance)     | 500    |
| `/g1/loco/lie_to_stand`         | `Lie2StandUp()`         | 702    |
| `/g1/loco/squat_to_stand`       | `Squat2StandUp()`       | 706    |
| `/g1/loco/stand_to_squat`       | `StandUp2Squat()`       | 706    |
| `/g1/loco/high_stand`           | `HighStand()`           | —      |
| `/g1/loco/low_stand`            | `LowStand()`            | —      |
| `/g1/loco/stop_move`            | `StopMove()`            | —      |

Every successful call is also echoed on `/g1/loco/last_command`
(`std_msgs/String`) for dashboards / logging.

## Driving the robot

Once the robot is in FSM 500 (see above):

```bash
# Arm the cmd_vel bridge:
ros2 topic pub --once /g1/enable std_msgs/msg/Bool "{data: true}"

# Drive it (keyboard or RViz):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To disable: `data: false`, or stop publishing `/cmd_vel` (the 500 ms watchdog
will issue a `StopMove`).

If you want raw behavior with no enable latch (e.g. Nav2 pipeline):

```bash
ros2 launch g1_ros2_bridge robot.launch.py require_enable:=false
```

## TF tree

After `robot.launch.py` is up the published frames are:

```
odom                              ← odom_bridge
└── base_link                     ← odom_bridge
    └── pelvis                    ← description.launch.py (static)
        ├── imu_link              ← state_bridge (per-tick)
        ├── imu_in_pelvis         ← URDF
        ├── torso_link            ← URDF
        │   ├── d435_link         ← URDF
        │   │   ├── camera_color_optical_frame  ← description.launch.py (static)
        │   │   └── camera_depth_optical_frame  ← description.launch.py (static)
        │   └── mid360_link       ← URDF
        │       └── livox_frame   ← URDF
        └── (every leg / arm / hand link)       ← URDF
```

## File layout

```
src/g1_ros2_bridge/
├── g1_ros2_bridge/
│   ├── joint_names.py          # 29-DOF index → URDF joint name
│   ├── state_bridge.py         # /joint_states + /imu/data
│   ├── odom_bridge.py          # /odom + odom→base_link TF
│   ├── cmd_vel_bridge.py       # /cmd_vel → LocoClient.Move (with watchdog)
│   └── realsense_publisher.py  # RealSense → /camera/{color,depth}/*
├── description/
│   ├── urdf/g1_29dof.urdf      # bundled URDF (mesh refs use package://g1_ros2_bridge)
│   └── meshes/*.STL            # 35 meshes referenced by the URDF
├── launch/
│   ├── description.launch.py   # robot_state_publisher + missing static TFs
│   ├── robot.launch.py         # bridges + description (default on-robot launch)
│   ├── laptop.launch.py        # rviz2 only by default
│   └── all_in_one.launch.py    # both at once
├── rviz/g1.rviz
└── ...
```

## Caveats

- The G1 LowState IMU `accelerometer` field is in m/s² and `gyroscope` is in
  rad/s per the Unitree SDK; the bridge passes them through unchanged.
- `nav_msgs/Odometry` has zero covariance — the robot doesn't expose any. If
  you fuse with `robot_localization`, set explicit `pose0_covariance`.
- `odom_bridge` publishes the `odom → base_link` TF. The G1's onboard pose
  drifts over time and resets on power cycle — treat it as relative odometry.
- `cmd_vel_bridge` calls `LocoClient.Move(..., continous_move=True)`, which
  sets a long velocity duration. The 500 ms watchdog issues `StopMove` if
  `/cmd_vel` goes silent, so a dead publisher won't leave the robot walking.

## Troubleshooting

### `ImportError: cannot import name 'core' from partially initialized module 'numpy'`

`~/.local/lib/python3.8/site-packages/numpy` is partially installed and shadows
the system numpy that ROS 2 Foxy was built against. Remove the user-local copy:

```bash
python3 -m pip uninstall -y numpy
# the system numpy at /usr/lib/python3/dist-packages/numpy/ will be used instead
```

### `ModuleNotFoundError: No module named 'unitree_sdk2py'`

```bash
python3 -m pip install -e ~/workspaces/unitree_sdk2_python
```

### Bridge starts but no `/joint_states` or `/imu/data` appear

The bridge needs to be on the same DDS network as the robot. Verify:
1. `G1_INTERFACE` is set to the interface actually on the robot's subnet.
2. The Unitree DDS env is sourced (typically `~/workspaces/unitree_ros2/setup.sh`).
3. `ros2 topic list` from a third terminal shows native topics like `rt/lowstate`.

### RViz: `Message Filter dropping message: frame 'livox_frame' for reason 'Unknown'`

RViz received a point cloud (typically `/utlidar/cloud_livox_mid360`) but
couldn't transform `livox_frame` to the Fixed Frame. The TF chain
`livox_frame → mid360_link → torso_link → … → pelvis → base_link → odom` is
only complete when **all** of the following are running:

1. `state_bridge` (publishes `pelvis → imu_link` and `/joint_states`).
2. `odom_bridge` (publishes `odom → base_link`).
3. `robot_state_publisher` with the bundled URDF (publishes
   `pelvis → torso_link → … → mid360_link → livox_frame` on `/tf_static`).
4. The static TFs in `description.launch.py` (`base_link → pelvis`,
   camera optical frames).

`robot.launch.py` runs all four by default. If you started the bridges with
`enable_description:=false`, either flip it back on or run
`description.launch.py` somewhere else on the same DDS network.

### RViz: image topic visible but display is black / dropping

RealSense images use `camera_color_optical_frame` / `camera_depth_optical_frame`,
which are **not** in the URDF — they're added as static TFs by
`description.launch.py`. If you bypassed that launch you'll see the topic in
`ros2 topic list` but RViz will silently drop every frame.

If you only want to spot-check the stream without TF, use `rqt_image_view` —
it doesn't require TF to render.

## Related repos in this workspace

- `../unitree_ros2` — Unitree's DDS / msg packages (`unitree_go`, `unitree_hg`,
  `unitree_api`). This bridge uses `unitree_sdk2_python` directly instead.
- `../unitree_sdk2_python` — Python SDK we depend on at runtime.
