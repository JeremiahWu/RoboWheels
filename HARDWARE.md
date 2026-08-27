# RoboWheels 硬件实测记录

> 所有结论来自 2026-08-26 在 Orange Pi 3 LTS 上的实测验证，不是猜测。
> 接线表见项目需求文档，本文件只记录**实测确认/修正**的部分。

## 系统环境（已验证）

- Orange Pi 3 LTS, kernel 5.16.17-sun50iw6, Ubuntu 22.04
- ROS2 Humble, `/opt/ros/humble`
- wiringOP gpio v2.46 (`/usr/local/bin/gpio`)
- OPi.GPIO 0.5.2 (SUNXI 模式，寄存器直访，可用)
- Python 3.10.12（注意：未装 smbus2/smbus/gpiod，装 I2C 驱动时需 pip install smbus2）

## 电机实测映射（2026-08-26 修正）

原始接线表标注 A=左、B=右，**实测相反**：

| 通道 | 引脚 | 实测驱动 | 前进方向 |
|---|---|---|---|
| A | AIN1=PL10(wPi8), AIN2=PH5(wPi11) | **右轮** | AIN2(PH5)=PWM, AIN1=LOW |
| B | BIN1=PH4(wPi14), BIN2=PH6(wPi12) | **左轮** | BIN2(PH6)=PWM, BIN1=LOW |

- 后退 = IN1 PWM, IN2 LOW（与前进相反）
- AT8236 无 STBY 引脚，IN1/IN2 全 LOW = 滑行
- 软件 PWM 200Hz 实测可用；低速测试 20% 占空比两轮均正常转动
- 整车双轮 IN2 PWM → 小车**前进**（已实测确认）

## 编码器（2026-08-26 实测）

- **左轮编码器 = RA/RB (PL2/PL3)**，右轮编码器 = LA/LB (PD24/PD23)（与接线表交叉，实测确认）
- 前进时计数正向递增 ✅（drive 实测）
- 4x 正交解码；OPi.GPIO 轮询采样实测可用
- 标定（直线跑 ×2，卷尺实测，均值）：
  - 右轮 1656 ticks/m，左轮 1679 ticks/m（4x）
  - ticks/rev (4x) ≈ 334（轮周长 0.20m），≈83 沿/圈，符合 11PPR×~7.4:1 电机
  - wheel_radius = 0.0318 m
- 注意：低速 PWM 下转速与占空比非线性（12%~20% 占空比都约 0.5 m/s），差速控制需实测标定；两轮速度不完全对称（跑偏 ~20° 出现过一次，疑打滑）
- **2026-08-27 修正：右轮前进时计数为负**（与左轮符号相反，交叉接线的另一面）。
  motor.yaml 加 `right_encoder_inverted: true` 后，前进时 /odom linear.x > 0、angular.z ≈ 0 ✅
- 标定留待 odom 步复核

## 电机驱动链路（2026-08-27 实测，/cmd_vel 卡点已解决）

- **根因**：OPi.GPIO 0.5.2 的 output()/input() 每次重开 sysfs 值文件（~100-200us/次），
  200Hz 软件 PWM 相位混叠（实测占空比 11% vs 命令 24%），车不动。
- **方案**：`robowheels/fast_gpio.py` — /dev/mem mmap 寄存器直访（~3us/次），
  寄存器模型抄 wiringOP v2.46，**8 个引脚寄存器读与 sysfs 读全部一致**（实测验证）。
  热循环 ~17kHz，200Hz/24% 占空比实测准确（tools/analyze_pin.py）。
- 修复过的坑：
  - 热循环心跳日志 `log_t = t0 + 1.0`（墙钟 vs 相对时间混用）→ 永远不打日志
  - 编码器计数只累加在局部变量、循环退出才移交 → /odom 恒 0
  - CycloneDDS CLI 发布者发现需 2~3s：短于 3s 的 `ros2 topic pub` 命令可能根本没送达。
    测试时 pub 至少 6s，或先发几秒再开始采样
- 安全确认：启动/退出后电机引脚保持 OUTPUT LOW；方向寄存器实测（电机=out、编码器=in）

## 手柄 + 避障栈（2026-08-27 实测）

- 手柄：Microsoft Xbox 360 (045e:028e, xpad 驱动)，/dev/input/js0。
  ros-humble-joy 的 joy_node 打开正常；**手柄不动时无 /joy 消息**（autorepeat=0），
  摇杆上推 = 正轴（`invert_linear: false`，实测确认）。
- 按键映射（config/control.yaml，均可改）：RB(5)=使能死区、A(0)=模式切换、
  左摇杆 Y=线速度、X=角速度。
- 避障实测（合成雷达注入 + 真雷达，obstacle_avoid 节点）：
  - 前方斜坡：v_out = v_in×(d-0.15)/(0.5-0.15)，合成 0.25m 障碍 → 0.3→0.0857 精确命中
  - 转向偏置：左侧障碍 → +1.5 rad/s（max_steer_bias 封顶）远离 ✓
  - 雷达超时 0.5s 无 /scan → 停车 ✓；IMU 倾角 >20° → 停车 ✓（用户抬车 20~63° 实测）
  - IMU 重力轴自动标定（启动时取 1s 均值）：实测 Y 轴朝下 (-0.01, 1.00, -0.03)
- i2c 总线瞬时错误（Errno 6）会杀死 imu 节点 → 已加自动重连（imu_publisher 容错）
- **CPU 特征（实测）**：rclpy+CycloneDDS 在本板每次 executor 唤醒约 3~4ms 固有开销
  （干净节点仅 1 订阅+20Hz timer 即 9% CPU，非本项目代码问题）。
  稳态：motor_driver 100%（热循环单核）、obstacle_avoid ~29%（70 唤醒/s）、
  imu ~12%（30Hz）、jc 4%、joy_node 3%、lidar 7%。4 核板总计 ~155%，可接受。
  imu 从 50Hz 降到 30Hz 换取 CPU（23%→12%）+ 唤醒数。
- 一键启动：`tools/start_stack.sh`（source 三个 workspace + ros2 launch robowheels
  robowheels.launch.py，含 cspc_lidar；`nolidar` 参数可跳过雷达）。
- 已知坑：CLI `ros2 topic pub --once` 在发现完成前发出会丢失；测试用 -r 重复发。

## I2C / MPU6050（已验证）

- i2c-0 总线（PD26=SDA.0 / PD25=SCL.0，ALT2 已激活）
- MPU6050 @ 0x68，WHO_AM_I=0x68 ✅，上电默认 PWR_MGMT_1=0x40（睡眠态，驱动需唤醒）

## LiDAR COIN-D4（已验证）

- USB-TTL: CH340 (1a86:7523) → /dev/ttyUSB0
- udev 规则已装：/etc/udev/rules.d/sc_mini.rules → /dev/sc_mini → ttyUSB0
  （规则里还有 ttyS0→sc_mini 一条，板载串口不用，但当前 USB 占用符号链接，不影响）
- 驱动：drivers/cspc_lidar_sdk_ros2（已 colcon build）
  - baudrate 230400, samp_rate 9, 版本 M1CT_TOF
  - 启动：`ros2 launch cspc_lidar lidar_launch.py`
  - /scan @ 10Hz，每圈 345~401 点，实测数据正常
  - frame_id 当前为 laser_link（后续 TF 需统一为 laser）
- 注意：节点是 LifecycleNode，launch 后需 configure/activate（实测 launch 即出数据）

## ROS2 图（实测时）

- topics: /scan, /point_cloud, /lidar_status, /lsd_error
- 节点名 cspc_lidar；曾出现重名警告，疑似 ros2 daemon 缓存，必要时 `ros2 daemon stop`
