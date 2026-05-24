# slam_walking_robot

A 2D-LiDAR SLAM and autonomous-navigation stack for a small **walking robot** built around a Jetson Orin Nano and an Arduino-driven motor base. The robot uses a mechanical walking mechanism (not a wheeled chassis) and exposes itself to ROS 2 as a differential-drive platform so that off-the-shelf SLAM and Nav2 components work out of the box.

The repository contains:

- **`src/slamwalker_bridge/`** — Python ROS 2 package: `serial_bridge_node` (cmd_vel ⇄ Arduino, encoder odometry) and a keyboard `teleop`.
- **`src/slamwalker_bringup/`** — launch files, RViz configs, slam_toolbox/Nav2 parameters, URDF/Xacro.
- **`src/ldlidar_ros2/`** — vendored LDRobot LD06/LD14/LD19 driver (with a submodule for the C++ SDK).
- **`arduino/`** — UNO firmware (`test/test.ino`), upload script, and several diagnostic/calibration sketches.
- **`scripts/calibrate.py`** — interactive walking-mechanism odometry calibrator.
- **`maps/`** — a saved `my_map.{pgm,yaml}` ready to use for navigation.
- **`simulation/`** — TurtleBot4 + Gazebo reference simulation (used for tuning, kept as a separate workspace).

---

## Hardware

| Item | Spec |
|------|------|
| SBC | NVIDIA Jetson Orin Nano Developer Kit 8 GB, JetPack 6.2 on a 32 GB microSD |
| MCU | Arduino UNO (ATmega328P) |
| Motor driver | HW-095 (L298N) breakout |
| Motors | 2× Pololu #4754 — 37D 70:1 metal gearmotor with 64 CPR Hall encoder |
| LiDAR | LDRobot **LD19** (CP210x, 230400 baud) |
| Mechanism | Custom walking linkage; effective ratio measured at **21333 ticks / metre** (≈ 2.14× extra reduction beyond the 70:1 gearmotor) |
| Body | 0.52 m × 0.356 m × 0.08 m |

The Arduino exposes a USB-CDC serial port (`/dev/ttyACM0`, symlinked as `/dev/arduino` via udev) at 115200 baud. The LiDAR appears as `/dev/ttyUSB0` (symlinked as `/dev/ldlidar`).

### TF tree

```
map ──► odom ──► base_footprint ──► base_link ──► base_laser
                                              ├── left_wheel
                                              └── right_wheel
```

`serial_bridge_node` publishes `odom → base_footprint`. `robot_state_publisher` (from the URDF) publishes everything below `base_link`. `slam_toolbox` (mapping) or `amcl` (localisation) publishes `map → odom`.

### Arduino pin map (`arduino/test/test.ino`)

```
D2  Left  encoder A   (INT0)
D3  Right encoder A   (INT1)
D4  Left  encoder B
D5  Right ENB (PWM)
D6  Right IN3
D7  Right encoder B
D8  Right IN4
D9  Left  IN2
D11 Left  ENA (PWM)
D12 Left  IN1
```

> Note: pins D10 and the original D12 IN3 had stopped working on this particular UNO; the firmware was re-mapped accordingly. If your board is fresh, the layout in `NAV_BRIDGE_GUIDE.md` (the pre-rewire version) is the recommended baseline.

### Serial protocol

| Direction | Frame | Meaning |
|-----------|-------|---------|
| ROS → MCU | `V<linear_x>,<angular_z>#\n` | Velocity command, m/s and rad/s |
| MCU → ROS | `O<dTickL>,<dTickR>,<dt_ms>\n` | Encoder delta, 20 Hz |
| MCU → ROS | `D ...` (ASCII debug line) | 1 Hz debug — PWM, encoder totals, pin states |
| MCU → ROS | `READY\n` | Boot banner |

Hard-coded safety: if no `V` frame is received for **500 ms**, motors stop.

The walking mechanism has a real PWM dead-zone of around **180 / 255**, so non-zero velocity commands are remapped into the **[180, 255]** range; below `MAX_SPEED * 0.01` the output is forced to 0.

---

## Software requirements

- **Ubuntu 22.04** on the Jetson with **ROS 2 Humble** (the JetPack 6.2 default works).
- ROS 2 packages: `slam_toolbox`, `nav2_bringup`, `nav2_map_server`, `nav2_rviz_plugins`, `robot_state_publisher`, `xacro`, `rviz2`.
- Python: `pyserial` (≥ 3.5).
- For Arduino: `arduino-cli` and `avrdude`.

```bash
sudo apt update && sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-rviz-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  python3-serial \
  avrdude
# arduino-cli: see https://arduino.github.io/arduino-cli/
```

---

## Build

Clone the workspace and its submodule (the LD-LiDAR vendor SDK):

```bash
git clone --recursive https://github.com/KrisWonka/slam_walking_robot.git ~/walker_ws
cd ~/walker_ws
# udev rule for the LiDAR (one-time)
bash src/ldlidar_ros2/scripts/create_udev_rules.sh
# build
colcon build --symlink-install
source install/setup.bash
```

If you forgot `--recursive`:

```bash
cd ~/walker_ws/src/ldlidar_ros2 && git submodule update --init --recursive
```

A udev rule for the Arduino is also recommended so that the launch files can use `/dev/arduino` instead of `/dev/ttyACM0`. A minimal rule:

```udev
# /etc/udev/rules.d/99-arduino.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arduino", MODE="0666", GROUP="dialout"
```

Add your user to `dialout`:

```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

---

## Flash the Arduino

```bash
arduino-cli compile --fqbn arduino:avr:uno ~/walker_ws/arduino/test
cd ~/walker_ws/arduino && bash upload.sh         # press RESET when prompted
```

`upload.sh` finds the most recent `test.ino.hex` from the `arduino-cli` cache and flashes it with `avrdude`.

Other sketches in `arduino/` are diagnostic helpers, not part of the runtime stack:

- `motor_diag/` — print the four encoder pin levels + cumulative ticks at 1 Hz.
- `motor_test/` — sweep PWM and report whether each side moves (used to discover the dead-zone).
- `rotation_test/` — fixed-tick rotation for measuring `ticks_per_meter`.
- `calibrate/` — same idea, dead-zone scan version.
- `test_hybrid/` — experimental variant; keep `test/test.ino` as the runtime firmware.

---

## Run

A handy cheatsheet also lives in [`commands`](commands).

### 1. Build a map (SLAM)

```bash
ros2 launch slamwalker_bringup slamwalker_slam.launch.py
# in another terminal
ros2 run slamwalker_bridge teleop                # WASD + arrows
# when satisfied, save:
ros2 run nav2_map_server map_saver_cli -f ~/walker_ws/maps/my_map
```

RViz opens automatically with `slam.rviz` pre-configured (Map + LaserScan + TF + RobotModel, top-down view).

### 2. Navigate using a saved map

```bash
ros2 launch slamwalker_bringup slamwalker_nav.launch.py \
    map:=~/walker_ws/maps/my_map.yaml
```

In RViz: click **2D Pose Estimate** to seed AMCL, then **Nav2 Goal** (or use the Navigation 2 panel) to send a goal.

### 3. Useful launch arguments

| Arg | Default | Notes |
|-----|---------|-------|
| `serial_port` | `/dev/arduino` | Arduino USB device |
| `lidar_port` | `/dev/ldlidar` | LD19 device |
| `lidar_model` | `LDLiDAR_LD19` | Other LD-series models also supported |
| `rviz` (slam) | `true` | Set `false` for headless operation |
| `map` (nav) | required | Path to the `.yaml` saved by `map_saver_cli` |

---

## Calibration

The walking mechanism's effective `ticks_per_meter` and wheel base must be measured experimentally. The default values in `slamwalker_slam.launch.py` (`ticks_per_meter: 21333.0`, `wheel_base: 0.26`) are correct for the as-built robot but should be re-measured if the mechanism changes.

A guided two-step calibrator is provided:

```bash
ros2 launch slamwalker_bringup slamwalker_slam.launch.py rviz:=false
# in another terminal
python3 ~/walker_ws/scripts/calibrate.py
```

It walks you through:

1. Driving in a straight line by a known distance → derives `ticks_per_meter`.
2. Rotating in place by a known angle → derives the **effective** `wheel_base`.

Plug the resulting numbers into the `serial_bridge` parameters in both launch files.

---

## Tuning notes

These parameters are the ones most likely to need adjustment on a different build of the same robot, and where to find them:

| Parameter | File | Purpose |
|---|---|---|
| `MAX_SPEED`, `PWM_MIN_MOVE` | `arduino/test/test.ino` | Map normalised speed → PWM range; lower `MAX_SPEED` for more torque, raise `PWM_MIN_MOVE` to stay above the dead-zone |
| `LEFT_MOTOR_SCALE`, `RIGHT_MOTOR_SCALE` | `arduino/test/test.ino` | Per-motor PWM trim for left/right balance |
| `WHEEL_BASE` | `arduino/test/test.ino` and the launch files | Differential-drive kinematics (in metres) |
| `ticks_per_meter` | `slamwalker_*.launch.py` | Encoder → distance conversion |
| `right_encoder_scale` | `serial_bridge_node.py` (param `right_encoder_scale`) | Compensate per-side encoder count drift |
| `inflation_radius`, footprint | `config/nav2_real.yaml` | Costmap inflation and robot polygon |
| `min_speed_xy`, `min_speed_theta`, `min_rotational_vel` | `config/nav2_real.yaml` | Keep DWB samples above the motor dead-zone |
| `minimum_travel_distance` | `config/slam_toolbox.yaml` | How often slam_toolbox publishes a TF/map update |

---

## Troubleshooting

- **`map_server` fails with `parameter 'yaml_filename' is not initialized`** — `nav2_real.yaml` must contain `map_server` and `amcl` sections; Nav2's `RewrittenYaml` only injects values for keys that already exist.
- **`map:=~/...` not expanded** — the launch file expands `~` via `OpaqueFunction`/`os.path.expanduser`. If you write the path yourself, use `$HOME` or the absolute path.
- **TF has two unconnected trees** — `serial_bridge_node` must publish `odom → base_footprint`, not `odom → base_link`, otherwise `base_link` gets two parents.
- **LiDAR scan looks rotated** — check the `base_link → base_laser` rpy in `slamwalker.urdf.xacro`; the stock LD19 mount uses `rpy="0 0 0"`.
- **Robot doesn't move at low speeds** — the walking mechanism has a high PWM dead-zone (~180/255). Confirm with `arduino/motor_test/`.
- **`Navigation 2` panel fails to load in RViz** — the plugin class name has a space in it (`nav2_rviz_plugins/Navigation 2`); the `nav.rviz` in this repo already uses the right name.
- **LiDAR data publish time-out, intermittently** — almost always USB hub power. Plug the LD19 directly into the Jetson, or use a powered hub.

---

## Simulation

`simulation/` is a separate ROS 2 workspace built around TurtleBot4 + Gazebo, used as a reference while tuning the real robot. See `simulation/SIM_WORLD_CHANGELOG_BILINGUAL.md` for the world / config history. It is not required to run the real robot.

---

## Status

- ✅ SLAM mapping, map saving, AMCL localisation, Nav2 goal navigation, RViz visualisation, keyboard teleop.
- 🟡 Per-motor PWM trim is still hand-tuned (`RIGHT_MOTOR_SCALE`).
- 🟡 No IMU fusion; odometry is encoder-only.
- ⚠️ LD19 occasional point-cloud time-outs depend on USB-hub power quality.

---

## Acknowledgements

- LDRobot LDLiDAR ROS 2 driver: <https://github.com/ldrobotSensorTeam/ldlidar_ros>
- TurtleBot4 (used as the simulation reference): <https://github.com/turtlebot/turtlebot4>
- Pololu 37D 70:1 motor #4754: <https://www.pololu.com/product/4754>
