# Simulation World Change Log (Bilingual)

> This document records simulation-related changes in execution order.  
> 本文档按**实际执行顺序**记录仿真相关变更。

## Logging Rules / 记录规则

- This file is append-only by default.  
  默认以追加方式维护本文件。
- Each step includes Chinese + English explanation.  
  每一步都包含中文和英文说明。
- Include: modified files, new files, downloaded files, and verification.  
  包含：修改文件、新增文件、下载文件、以及验证结果。

---

## Session: 2026-02-25

### Step 1 - Fix launch parameter forwarding for SLAM/Nav2

**Chinese**  
修复了 `turtlebot4_ignition.launch.py` 参数透传不完整的问题。之前使用 `slam:=true` 时，参数没有传到底层 `turtlebot4_spawn.launch.py`，导致 `slam_toolbox` 没有真正启动。

**English**  
Fixed incomplete launch argument forwarding in `turtlebot4_ignition.launch.py`. Previously, `slam:=true` was not propagated to `turtlebot4_spawn.launch.py`, so `slam_toolbox` did not actually start.

**Modified file(s)**
- `src/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py`

**What changed**
- Added launch arguments:
  - `use_sim_time`
  - `localization`
  - `slam`
  - `nav2`
- Forwarded arguments to child launches:
  - To `ignition.launch.py`: `world`, `model`, `use_sim_time`
  - To `turtlebot4_spawn.launch.py`: `namespace`, `use_sim_time`, `rviz`, `model`, `localization`, `slam`, `nav2`, `x`, `y`, `z`, `yaw`

**Verification**
- `slam_toolbox`, `robot_state_publisher`, `/tf_static` appeared during runtime.

---

### Step 2 - Forward `world` argument across spawn/bridge chain

**Chinese**  
修复了切换 world 时桥接层不跟随的问题。此前 `world` 只传到了 Gazebo 层，`ros_ign_bridge` 仍可能使用默认 world，导致话题路径不一致。

**English**  
Fixed world-switch inconsistency where `world` reached Gazebo but not the bridge layer. `ros_ign_bridge` could still use a default world, causing topic path mismatch.

**Modified file(s)**
- `src/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py`
- `src/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py`

**What changed**
- In `turtlebot4_ignition.launch.py`:
  - Forwarded `('world', LaunchConfiguration('world'))` to spawn launch.
- In `turtlebot4_spawn.launch.py`:
  - Added `DeclareLaunchArgument('world', default_value='warehouse', ...)`
  - Passed `('world', world)` to `ros_ign_bridge.launch.py`

**Verification**
- `world:=depot` launch showed healthy `/scan`, `/tf`, `/tf_static`, and `slam_toolbox`.

---

### Step 3 - External world discovery and downloads

**Chinese**  
尝试筛选并下载外部室内 world（房间/家具类），用于当前 Humble + Ignition 环境验证。

**English**  
Searched and downloaded external indoor worlds (room/furniture style) for validation in current Humble + Ignition setup.

**Downloaded file(s)**
- `~/.ignition/fuel/fuel.gazebosim.org/yohatad/worlds/living_room/1/living_room.sdf`  
  (Downloaded successfully via `ign fuel download`)

**Attempted download(s)**
- `https://fuel.gazebosim.org/1.0/watermelon123/worlds/bedroom`  
  (Command returned without a clear local SDF confirmation in this workspace context)

**New file(s) in workspace**
- None created in workspace at this step.

---

### Step 4 - Fix missing AWS small-house model resource path

**Chinese**  
`small_house.sdf` 启动时出现 `Unable to find uri[model://aws_robomaker_residential_*]`。根因是 Ignition 资源路径没有包含 `aws-robomaker-small-house-world` 的 `models/worlds`。

**English**  
`small_house.sdf` initially failed with `Unable to find uri[model://aws_robomaker_residential_*]`. Root cause: Ignition resource path did not include `aws-robomaker-small-house-world` `models/worlds`.

**Modified file(s)**
- `src/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/ignition.launch.py`

**What changed**
- Extended `IGN_GAZEBO_RESOURCE_PATH` with:
  - `.../src/aws-robomaker-small-house-world/models`
  - `.../src/aws-robomaker-small-house-world/worlds`
- Added existence checks before appending paths.

**Verification**
- Missing-URI errors disappeared in subsequent runs.

---

### Step 5 - Fix invalid inertia in AWS models (small_house)

**Chinese**  
`small_house` 仍报 `A link named link has invalid inertia`。定位并修复了两个模型中的惯量定义错误。

**English**  
`small_house` still reported `A link named link has invalid inertia`. Located and fixed invalid inertia definitions in two AWS models.

**Modified file(s)**
- `src/aws-robomaker-small-house-world/models/aws_robomaker_residential_Dumbbell_01/model.sdf`
- `src/aws-robomaker-small-house-world/models/aws_robomaker_residential_ShoeRack_01/model.sdf`

**What changed**
- `Dumbbell_01`: changed `<iyy>0</iyy>` to a positive value (`14.601`).
- `ShoeRack_01`: fixed duplicated `<ixx>` entry; replaced second one with `<izz>0.02</izz>`.

**Verification**
- Programmatic inertia validation over all AWS model SDFs returned `bad_count 0`.
- Runtime check no longer showed:
  - `invalid inertia`
  - `Failed to load a world`
  - `Unable to find uri`
- Key runtime components healthy:
  - Nodes: `/controller_manager`, `/slam_toolbox`, `/robot_state_publisher`, `/lidar_bridge`, `/clock_bridge`
  - Topics: `/scan`, `/tf`, `/tf_static`, `/clock`

---

## Current Recommended Launch / 当前推荐启动命令

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:=small_house slam:=true rviz:=false
```

Fallback stable world:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite world:=depot slam:=true rviz:=false
```

---

## New Files Summary / 新增文件汇总

- `SIM_WORLD_CHANGELOG_BILINGUAL.md` (this file / 本文件)

---

### Step 6 - Increase robot speed limits (Nav2 + safety override)

**Chinese**  
针对“机器人转动和移动速度很慢”的问题，提升了导航层速度上限，并把 Create3 仿真默认安全模式切换为 `full`，避免 `motion_control` 将速度压回安全上限（约 0.306 m/s）。

**English**  
To fix slow robot motion/rotation, increased Nav2 velocity limits and switched Create3 simulation default safety mode to `full`, preventing `motion_control` from clamping speed to the safety-on limit (~0.306 m/s).

**Modified file(s)**
- `src/turtlebot4/turtlebot4_navigation/config/nav2.yaml`
- `src/create3_sim/irobot_create_common/irobot_create_common_bringup/launch/create3_nodes.launch.py`

**What changed**
- In `nav2.yaml`:
  - `FollowPath.max_vel_x`: `0.26 -> 0.35`
  - `FollowPath.max_vel_theta`: `1.0 -> 1.6`
  - `FollowPath.max_speed_xy`: `0.26 -> 0.35`
  - `velocity_smoother.max_velocity`: `[0.26, 0.0, 1.0] -> [0.35, 0.0, 1.6]`
  - `velocity_smoother.min_velocity`: `[-0.26, 0.0, -1.0] -> [-0.35, 0.0, -1.6]`
- In `create3_nodes.launch.py`:
  - Added `{'safety_override': 'full'}` to `motion_control` node parameters.

**Verification**
- Lint check: no linter errors in modified files.
- Build check: both packages built successfully:
  - `turtlebot4_navigation`
  - `irobot_create_common_bringup`

**Notes / 说明**
- This is a conservative speed increase.  
  这是一次保守提速，兼顾稳定性。
- If needed, we can further tune acceleration and angular limits after runtime testing.  
  如有需要，可在运行测试后继续上调加速度和角速度上限。

---

### Step 7 - Second speed increase + smoother command rate

**Chinese**  
根据“还想再快一点，同时有点卡顿”的反馈，进行了第二档提速，并提高控制与速度平滑频率，让速度响应更快、轨迹更连贯。

**English**  
Based on feedback (“faster, but currently a bit choppy”), applied a second speed increase and raised controller/smoother frequencies for quicker response and smoother motion.

**Modified file(s)**
- `src/turtlebot4/turtlebot4_navigation/config/nav2.yaml`

**What changed**
- `controller_server.controller_frequency`: `20.0 -> 25.0`
- `FollowPath.max_vel_x`: `0.35 -> 0.45`
- `FollowPath.max_vel_theta`: `1.6 -> 2.0`
- `FollowPath.max_speed_xy`: `0.35 -> 0.45`
- `FollowPath.acc_lim_x`: `2.5 -> 3.0`
- `FollowPath.acc_lim_theta`: `3.2 -> 4.0`
- `FollowPath.decel_lim_x`: `-2.5 -> -3.0`
- `FollowPath.decel_lim_theta`: `-3.2 -> -4.0`
- `velocity_smoother.smoothing_frequency`: `20.0 -> 25.0`
- `velocity_smoother.max_velocity`: `[0.35, 0.0, 1.6] -> [0.45, 0.0, 2.0]`
- `velocity_smoother.min_velocity`: `[-0.35, 0.0, -1.6] -> [-0.45, 0.0, -2.0]`
- `velocity_smoother.max_accel`: `[2.5, 0.0, 3.2] -> [3.0, 0.0, 4.0]`
- `velocity_smoother.max_decel`: `[-2.5, 0.0, -3.2] -> [-3.0, 0.0, -4.0]`

**Verification**
- Lint check: no issues.
- Build check: `turtlebot4_navigation` built successfully.

**Runtime note / 运行说明**
- Ensure `motion_control` is still in full safety override mode at runtime:  
  `ros2 param get /motion_control safety_override` should be `full`.

---

### Step 8 - Third speed increase (raise hard caps + planner caps)

**Chinese**  
按“车速再快一点”的要求，进行了第三档提速。这次不仅提高 Nav2 参数，还同步提高底层 `diffdrive_controller` 与 `motion_control` 的硬上限，避免出现“上层给了更高速度，但底层仍限速”的情况。

**English**  
Applied a third speed increase per request. This time, both Nav2 limits and lower-level hard caps (`diffdrive_controller` + `motion_control`) were raised to avoid upper-layer commands being clamped by lower layers.

**Modified file(s)**
- `src/turtlebot4/turtlebot4_navigation/config/nav2.yaml`
- `src/create3_sim/irobot_create_common/irobot_create_control/config/control.yaml`
- `src/create3_sim/irobot_create_common/irobot_create_nodes/include/irobot_create_nodes/motion_control_node.hpp`

**What changed**
- In `nav2.yaml`:
  - `controller_frequency`: `25.0 -> 30.0`
  - `FollowPath.max_vel_x`: `0.45 -> 0.55`
  - `FollowPath.max_vel_theta`: `2.0 -> 2.4`
  - `FollowPath.max_speed_xy`: `0.45 -> 0.55`
  - `FollowPath.acc_lim_x`: `3.0 -> 3.5`
  - `FollowPath.acc_lim_theta`: `4.0 -> 5.0`
  - `FollowPath.decel_lim_x`: `-3.0 -> -3.5`
  - `FollowPath.decel_lim_theta`: `-4.0 -> -5.0`
  - `FollowPath.trans_stopped_velocity`: `0.25 -> 0.1` (reduce start/stop choppy feel)
  - `velocity_smoother.smoothing_frequency`: `25.0 -> 30.0`
  - `velocity_smoother.max_velocity`: `[0.45, 0.0, 2.0] -> [0.55, 0.0, 2.4]`
  - `velocity_smoother.min_velocity`: `[-0.45, 0.0, -2.0] -> [-0.55, 0.0, -2.4]`
  - `velocity_smoother.max_accel`: `[3.0, 0.0, 4.0] -> [3.5, 0.0, 5.0]`
  - `velocity_smoother.max_decel`: `[-3.0, 0.0, -4.0] -> [-3.5, 0.0, -5.0]`
- In `control.yaml`:
  - `linear.x.max_velocity`: `0.46 -> 0.60`
  - `linear.x.min_velocity`: `-0.46 -> -0.60`
  - `linear.x.max_acceleration`: `0.9 -> 1.2`
  - `angular.z.max_velocity`: `1.9 -> 2.4`
  - `angular.z.min_velocity`: `-1.9 -> -2.4`
- In `motion_control_node.hpp`:
  - `GYRO_MAX_ROTATE_SPEED_RAD_S`: `1.9 -> 2.4`
  - `SAFETY_OFF_MAX_SPEED`: `0.46 -> 0.60`

**Verification**
- Lint check: no issues in edited files.
- Build check: success for
  - `turtlebot4_navigation`
  - `irobot_create_control`
  - `irobot_create_nodes`
  - `irobot_create_common_bringup`

## Step 8 - Third speed increase + Maze map output status

**中文**  
根据你“再快一点”的要求，这一步把速度上限再提高一档，并同步抬高了底层 Create3 控制上限（不仅是 Nav2）。  
同时尝试直接在 `maze` 世界自动建图导出，但本次运行被系统环境中的 DDS SHM 锁冲突阻塞（`RTPS_TRANSPORT_SHM ... open_and_lock_file failed`），导致 `/odom` 订阅不稳定，自动扫描导图流程未能在本次会话内稳定完成。  
另外已确认工作区中已有可直接使用的 `maze` 地图文件：`maps/maze_map.yaml` + `maps/maze_map.pgm`。

**English**  
Per your request (“make it faster”), this step increases speed limits one more level and also raises low-level Create3 limits (not just Nav2).  
I also attempted to auto-scan and export a `maze` map directly, but this run was blocked by an environment-level DDS SHM lock conflict (`RTPS_TRANSPORT_SHM ... open_and_lock_file failed`), which made `/odom` subscription unstable, so the automatic scan-export pipeline could not complete reliably in this session.  
Verified that usable maze map files already exist in the workspace: `maps/maze_map.yaml` + `maps/maze_map.pgm`.

**Modified file(s)**
- `src/turtlebot4/turtlebot4_navigation/config/nav2.yaml`
- `src/create3_sim/irobot_create_common/irobot_create_nodes/include/irobot_create_nodes/motion_control_node.hpp`
- `src/create3_sim/irobot_create_common/irobot_create_control/config/control.yaml`

**What changed**
- `FollowPath.max_vel_x`: `0.45 -> 0.55`
- `FollowPath.max_vel_theta`: `2.0 -> 2.3`
- `FollowPath.max_speed_xy`: `0.45 -> 0.55`
- `FollowPath.acc_lim_x`: `3.0 -> 3.6`
- `FollowPath.acc_lim_theta`: `4.0 -> 4.6`
- `FollowPath.decel_lim_x`: `-3.0 -> -3.6`
- `FollowPath.decel_lim_theta`: `-4.0 -> -4.6`
- `velocity_smoother.max_velocity`: `[0.45, 0.0, 2.0] -> [0.55, 0.0, 2.3]`
- `velocity_smoother.min_velocity`: `[-0.45, 0.0, -2.0] -> [-0.55, 0.0, -2.3]`
- `velocity_smoother.max_accel`: `[3.0, 0.0, 4.0] -> [3.6, 0.0, 4.6]`
- `velocity_smoother.max_decel`: `[-3.0, 0.0, -4.0] -> [-3.6, 0.0, -4.6]`
- `motion_control_node.hpp`:
  - `SAFETY_OFF_MAX_SPEED`: `0.46 -> 0.60`
  - `GYRO_MAX_ROTATE_SPEED_RAD_S`: `1.9 -> 2.3`
- `control.yaml`:
  - `linear.x.max_velocity/min_velocity`: `±0.46 -> ±0.60`
  - `angular.z.max_velocity/min_velocity`: `±1.9 -> ±2.3`

**Verification**
- Lint check: no issues.
- Build check: packages built successfully:
  - `turtlebot4_navigation`
  - `irobot_create_control`
  - `irobot_create_nodes`
  - `irobot_create_common_bringup`
- Existing maze map files verified:
  - `maps/maze_map.yaml`
  - `maps/maze_map.pgm`

