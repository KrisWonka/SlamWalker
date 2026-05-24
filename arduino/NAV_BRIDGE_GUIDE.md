# SlamWalker 导航对接指南

将 `ros2_tb4_ws` 仿真中的 Nav2 导航栈部署到真实 Arduino + L298N 小车上。

---

## 1. 整体架构

```
┌───────────────────────────────────────────────────────────────┐
│                      ROS2 (PC / SBC)                          │
│                                                               │
│   Nav2 controller_server ─► velocity_smoother                 │
│            │                       │                          │
│            └───► /cmd_vel ◄────────┘                          │
│                     │                                         │
│         ┌───────────▼────────────┐    ┌─────────────────┐     │
│         │  serial_bridge_node    │    │  /odom (publish) │     │
│         │  订阅 /cmd_vel ────────┼──► │  /tf odom→base   │     │
│         │  写串口 → Arduino      │    └─────────────────┘     │
│         │  读串口 ← 编码器 ticks │                             │
│         └───────────┬────────────┘                            │
│                     │ UART 115200 (USB / GPIO)                │
└─────────────────────┼─────────────────────────────────────────┘
                      │
              ┌───────▼────────┐
              │   Arduino Uno  │
              │   test.ino     │
              │   差速运动学    │  ← cmd_vel → PWM
              │   编码器中断    │  → O<dL>,<dR>,<dt>
              │   → L298N PWM  │
              └───────┬────────┘
                      │
         ┌────────────▼────────────┐
         │  HW-095 (L298N) → 电机  │
         │  Pololu 37D + 64CPR enc │
         └─────────────────────────┘
```

Nav2 原来在仿真里把 `cmd_vel` 通过 ros_gz_bridge 送到 Ignition diff_drive 插件。
现在我们用一个 **serial_bridge_node** 替代这条链路：
- **下行**：把 `cmd_vel` 转成 `V<lx>,<az>\n` 串口帧发给 Arduino
- **上行**：解析 Arduino 回传的 `O<dTickL>,<dTickR>,<dt_ms>\n` 编码器增量，
  计算差速里程计，发布 `/odom` 和 `odom → base_link` TF

---

## 2. 需要新建的 ROS2 节点：serial_bridge_node

### 2.1 功能

| 方向 | 内容 |
|------|------|
| **订阅** | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| **发布** | `/odom` (`nav_msgs/msg/Odometry`) + TF `odom → base_link` |
| **串口发送** | `V<linear.x>,<angular.z>\n`（与 Arduino 固件协议一致） |
| **串口接收** | `O<dTickL>,<dTickR>,<dt_ms>\n` 编码器增量 / `OK` / `E:xxx` |

### 2.2 参考实现（Python，含 odom 发布）

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ---------- 参数 ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wheel_base', 0.22)
        self.declare_parameter('wheel_radius', 0.03575)
        self.declare_parameter('gear_ratio', 30.0)
        self.declare_parameter('encoder_cpr_on_a', 32.0)  # CHANGE edges on A channel

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('rate_hz').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        gear_ratio = self.get_parameter('gear_ratio').value
        enc_cpr = self.get_parameter('encoder_cpr_on_a').value

        self.ticks_per_meter = (enc_cpr * gear_ratio) / (2.0 * math.pi * self.wheel_radius)

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f'Serial opened: {port} @ {baud}')

        # ---------- 里程计状态 ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        # ---------- ROS 接口 ----------
        self.latest_twist = Twist()
        self.lock = threading.Lock()

        self.create_subscription(Twist, 'cmd_vel', self.twist_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.send_cmd)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()

    # ---------- cmd_vel → 串口 ----------

    def twist_cb(self, msg: Twist):
        with self.lock:
            self.latest_twist = msg

    def send_cmd(self):
        with self.lock:
            lx = self.latest_twist.linear.x
            az = self.latest_twist.angular.z
        frame = f'V{lx:.3f},{az:.3f}\n'
        self.ser.write(frame.encode())

    # ---------- 串口 → odom ----------

    def serial_reader(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                if line.startswith('O'):
                    self.process_odom(line)
                elif line.startswith('E:'):
                    self.get_logger().warn(f'Arduino error: {line}')
                elif line == 'READY':
                    self.get_logger().info('Arduino READY')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')

    def process_odom(self, line: str):
        try:
            parts = line[1:].split(',')
            d_ticks_l = int(parts[0])
            d_ticks_r = int(parts[1])
            dt_ms = int(parts[2])
        except (ValueError, IndexError):
            return

        dt = dt_ms / 1000.0
        if dt <= 0:
            return

        d_left = d_ticks_l / self.ticks_per_meter
        d_right = d_ticks_r / self.ticks_per_meter

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.vx = d_center / dt
        self.vth = d_theta / dt

        now = self.get_clock().now().to_msg()

        # TF broadcast: odom → base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.3 建包步骤

```bash
cd ~/ros2_tb4_ws/src
ros2 pkg create slamwalker_bridge --build-type ament_python \
    --dependencies rclpy geometry_msgs

# 把上面的 Python 文件放到:
#   slamwalker_bridge/slamwalker_bridge/serial_bridge_node.py

# 在 setup.py 的 console_scripts 里添加:
#   'serial_bridge_node = slamwalker_bridge.serial_bridge_node:main'

# 在 package.xml 里加:
#   <exec_depend>python3-serial</exec_depend>

cd ~/ros2_tb4_ws
colcon build --packages-select slamwalker_bridge
```

---

## 3. Launch 文件修改

### 3.1 仿真链路中需要替换的部分

仿真时 cmd_vel 的链路是：

```
Nav2 → /cmd_vel → motion_control_node → diffdrive_controller/cmd_vel_unstamped
      → ros_gz_bridge → Ignition diff_drive
```

**真机上不需要以下节点/launch：**
- `turtlebot4_ignition_bringup` 里的所有 Ignition 相关 launch
- `create3_ros_ignition_bridge`（ros_gz_bridge）
- `irobot_create_nodes` 里的 `motion_control_node`（它做了 Create3 特有的安全逻辑）

### 3.2 真机 launch 文件示意

```python
# slamwalker_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_dir = get_package_share_directory('turtlebot4_navigation')

    return LaunchDescription([
        # 1) 串口桥接节点
        Node(
            package='slamwalker_bridge',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'rate_hz': 20.0,
            }],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),

        # 2) 激光雷达（如果你有 RPLIDAR）
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB1',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame',
        #     }],
        # ),

        # 3) Nav2 导航栈（复用仿真的配置，但换参数文件）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(nav2_dir, 'config', 'nav2.yaml'),
            }.items(),
        ),
    ])
```

---

## 4. Nav2 参数调整

仿真里的 `nav2.yaml` 是为 TurtleBot4 / Create3 调的，真机需要改以下参数。

### 4.1 速度限制（最重要）

在 `nav2.yaml` 中找到 `controller_server` 和 `velocity_smoother`：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # 原值 0.9 / 3.5 是仿真用的，真机要降
      max_vel_x:     0.30     # 根据你电机实际最高速度
      min_vel_x:    -0.20
      max_vel_theta:  1.50    # 根据轮距和电机能力
      min_speed_xy:   0.05
      max_speed_xy:   0.30

velocity_smoother:
  ros__parameters:
    max_velocity:     [0.30, 0.0, 1.50]
    min_velocity:     [-0.20, 0.0, -1.50]
    max_accel:        [0.50, 0.0, 2.00]    # 加速度也要调小
    max_decel:        [-1.00, 0.0, -2.00]
```

### 4.2 Costmap 参数

```yaml
local_costmap:
  ros__parameters:
    robot_radius: 0.12         # 你的小车半径（米）
    resolution: 0.05
    width: 3
    height: 3
    # 障碍物层的 scan topic 要与你的雷达一致
    obstacle_layer:
      scan:
        topic: /scan

global_costmap:
  ros__parameters:
    robot_radius: 0.12
```

### 4.3 DWB 局部规划器

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # 离散采样和轨迹评估参数，建议先跑默认看效果再调
      sim_time: 1.5             # 轨迹预测时间
      vx_samples: 15
      vtheta_samples: 20
```

---

## 5. 里程计（Odometry）— 已支持

### 硬件

电机 [Pololu #4750](https://www.pololu.com/product/4750) 自带 **64 CPR 双通道霍尔编码器**。
Arduino 通过硬件中断（D2/D3）读取 A 通道 CHANGE 事件，digitalRead B 通道判方向，
得到 **32 counts/motor_rev**。乘以齿轮比后即为每圈轮轴的 ticks。

### 串口协议

Arduino 以 20Hz 上报增量帧：

```
O<dTickL>,<dTickR>,<dt_ms>\n
```

例如：`O48,-47,50\n` 表示 50ms 内左轮 +48 ticks、右轮 −47 ticks。

### ROS2 侧处理

`serial_bridge_node`（见第 2 节）解析增量帧后：
1. 将 ticks 换算为米：`d = dTicks / ticks_per_meter`
2. 差速运动学积分 `(x, y, θ)`
3. 发布 `nav_msgs/msg/Odometry` 到 `/odom`
4. 广播 TF `odom → base_link`

### 编码器接线（每个电机 4 根线）

| Pololu 线色 | 接到 | 说明 |
|-------------|------|------|
| 绿 (GND) | Arduino GND | 编码器地 |
| 蓝 (Vcc) | Arduino 5V | 编码器供电 3.5–20V |
| 黄 (A) | **左: D2 / 右: D3** | A 通道，硬件中断 |
| 白 (B) | **左: D4 / 右: D7** | B 通道，普通数字读取 |

> 红/黑是电机电源线，接 HW-095 (L298N) 的 OUT1/OUT2、OUT3/OUT4。

### 关键参数（需要按实际调整）

| 参数 | Arduino (`test.ino`) | Bridge 节点 |
|------|---------------------|-------------|
| 齿轮比 | `GEAR_RATIO = 70.0` | `gear_ratio: 70.0` |
| 轮距 | `WHEEL_BASE = 0.22` | `wheel_base: 0.22` |
| 轮半径 | — | `wheel_radius: 0.03575` |

> 如果你用的是 Pololu 30:1 减速版 (#4752)，齿轮比就是 30。
> 如果是其他比例（如 50:1 #4753），对应修改即可。
> **没有减速箱（裸电机 #4750）的话 10000RPM 太快，不适合直接驱动轮子。**

### 后续优化：加 IMU 融合

编码器 odom 在轮子打滑时会漂移。如果后续加一个 IMU（如 MPU6050），
可以用 `robot_localization` 包（EKF）融合编码器 + IMU，提升精度：

```yaml
# ekf.yaml
ekf_filter_node:
  ros__parameters:
    odom0: /odom              # 编码器里程计
    imu0: /imu/data           # IMU
    odom0_config: [true, true, false, false, false, true,
                   true, false, false, false, false, true,
                   false, false, false]
    imu0_config:  [false, false, false, true, true, true,
                   false, false, false, true, true, true,
                   false, false, false]
```

---

## 6. TF 树 / URDF

仿真里的 TF 树由 `robot_state_publisher` + Ignition 联合发布。
真机需要一个简化的 URDF 来发布静态 TF：

```
map ─► odom ─► base_link ─► laser_frame
                  │
                  └──► left_wheel / right_wheel (可选)
```

- `map → odom`：由 AMCL 或 slam_toolbox 发布
- `odom → base_link`：由里程计源发布（编码器 / slam_toolbox）
- `base_link → laser_frame`：由 `robot_state_publisher` 根据 URDF 发布

建议新写一个简化 URDF（或 xacro），只描述底盘 + 雷达支架的几何关系，
不需要 TurtleBot4 的完整模型。关键参数：
- 雷达相对底盘中心的 XYZ 偏移
- 雷达朝向（通常 yaw = 0 或 π）

---

## 7. 传感器

### 激光雷达

这是 Nav2 最核心的输入。仿真里用的 topic 是 `/scan`。
真机确保雷达节点也发布到 `/scan`，frame_id 与 URDF 中一致。

### 摄像头（可选）

仿真里用了 OAK-D。如果真机不带相机，在 nav2.yaml 中去掉
`obstacle_layer` 里的 voxel / depth 相关配置。

---

## 8. 快速上手检查清单

- [ ] **接线**：编码器黄/白线接 D2/D3 (A) + D4/D7 (B)，蓝→5V，绿→GND
- [ ] **接线**：L298N 电机引脚按新分配：enA=11, in1=10, in2=9, enB=5, in3=8, in4=12
- [ ] **烧录**：Arduino 烧录 `test.ino`（115200 baud）
- [ ] **验证电机**：串口监视器发 `V0.2,0.0\n`，确认两个电机同向转
- [ ] **验证编码器**：手动转轮子，看串口是否输出 `O<ticks>,<ticks>,<dt>\n`
- [ ] **确认串口**：`ls /dev/ttyUSB*` 或 `/dev/ttyACM*`
- [ ] **建 ROS2 包**：`slamwalker_bridge`，含 serial_bridge_node，编译通过
- [ ] **测试 bridge**：运行 bridge 节点，`ros2 topic echo /odom` 确认有数据
- [ ] **测试 cmd_vel**：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`，小车动
- [ ] **激光雷达**：接好雷达，确认 `/scan` topic 有数据
- [ ] **URDF + TF**：简化 URDF，`robot_state_publisher` 发布 base_link→laser_frame
- [ ] **SLAM**：启动 `slam_toolbox`，RViz 中能看到地图
- [ ] **导航**：启动 Nav2，发送目标，观察小车运动 + odom 轨迹
- [ ] **调参**：根据实际表现调 `nav2.yaml` 速度 / costmap / DWB 参数
