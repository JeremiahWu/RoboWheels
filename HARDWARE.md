# RoboWheels 硬件实测记录

> 所有结论来自真机实测，不是猜测。本文件记录**实测确认/修正**的部分。
> 接线表见项目需求文档。

---

# RDK X5（2026-09-10 实测，当前平台）

## 系统环境（已验证）

- D-Robotics RDK X5 V1.0，kernel 6.1.83 aarch64，Ubuntu 22.04 (RDK OS)，/etc/version = **3.5.0**
- ROS2 Humble @ `/opt/ros/humble`（joy/teleop 等 apt 包装这里）
- TROS @ `/opt/tros/humble`（mipi_cam 2.5.x、hobot_stereonet、hobot_shm 已装）
- Python 3.10：Hobot.GPIO 0.0.2、pyserial 3.5、opencv 4.11、numpy 1.26
- 以 root 登录，GPIO/UART 无权限问题

**多环境 source 顺序（重要）**：
```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash     # mipi_cam / hobot_shm
source ~/RoboWheels/ros2_ws/install/setup.bash
```

## 40PIN 引脚（Hobot.GPIO BOARD 模式，实测）

Hobot.GPIO 的 BOARD 模式直接使用**物理管脚号**（1-40），底层是 sysfs
GPIO（`/sys/class/gpio/gpioN`）。全部可用 BOARD 引脚：3,5,7,8,10,11,12,
13,15,16,18,19,21,22,23,24,26,27,28,29,31,32,33,35,36,37,38,40
（**没有 39**；39=GND，见下）。

本项目用到的引脚与 sysfs 编号（实测对照）：

| 用途 | BOARD 脚 | sysfs gpio | 备注 |
|---|---|---|---|
| AIN1 | 27 | 355 | AT8236 A 通道 IN1，另有 PWM 功能 pwmchip4/pwm1 |
| AIN2 | 29 | 399 | AT8236 A 通道 IN2，另有 PWM 功能 pwmchip0/pwm0 |
| BIN1 | 31 | 400 | AT8236 B 通道 IN1，另有 PWM 功能 pwmchip0/pwm1 |
| BIN2 | 33 | 357 | AT8236 B 通道 IN2，另有 PWM 功能 pwmchip6/pwm1 |
| E1A | 11 | 380 | 编码器 1，纯 GPIO |
| E1B | 13 | 379 | 编码器 1，纯 GPIO |
| E2A | 15 | 388 | 编码器 2，纯 GPIO |
| E2B | 16 | 382 | 编码器 2，纯 GPIO |
| GS130 同步线 | 37 | 401 | 相机黑线，LPWM 触发（用户确认 37） |
| 舵机控制器 TX | 8 | 383 | = UART1 TX（见下） |
| 舵机控制器 RX | 10 | 384 | = UART1 RX（见下） |

其他事实：
- 4 个电机脚都有**硬件 PWM** 功能（X5 PWM 支持 0.05Hz~1MHz，另有 PWM 脚
  18/28/32/37），但实测 `/sys/class/pwm/pwmchip0|4|6` export **全部失败**
  （管脚复用被占用，需 srpi-config 关闭对应专用功能才能用）。
  当前方案 = 软件 PWM（与 OPi 版一致，200Hz，fast-fd 直写）。
- Hobot.GPIO 是 sysfs 后端：`input()`/`output()` 每次重开文件（~50-100µs/次，
  慢）；实测**保持 fd 打开 + pread/pwrite 约 28µs/读（35kHz）**。
  motor_driver 与 tools 全部走 fast-fd 路径。
- Hobot.GPIO 的 `pull_up_down` 参数被**静默忽略**（sysfs 无上拉配置）。
  编码器模块板自带上拉（OPi 上已验证同款工作正常）。
- `GPIO.cleanup()` 会把引脚变回输入悬空——AT8236 IN 悬空可能误转，
  电机节点退出时**只置 LOW，不 cleanup**（红线不变）。

## 电机/编码器（待实测确认）

- 接线：AIN1=27 AIN2=29（A 通道），BIN1=31 BIN2=33（B 通道）；
  编码器 E1A=11 E1B=13，E2A=15 E2B=16
- AT8236 真值表沿用 OPi 实测：前进 = IN2 PWM + IN1 LOW，后退相反
- **哪个通道驱动哪个轮、哪个编码器属于哪个轮：X5 接线未验证**
  （OPi 上是 A=右 B=左、编码器交叉）。上线前必须跑：
  ```bash
  python3 tools/motor_test.py A fwd 20 2    # 看哪个轮转
  python3 tools/motor_test.py B fwd 20 2
  python3 tools/encoder_test.py watch       # 手转轮子看哪组编码器动
  python3 tools/encoder_test.py drive 20 4  # 直线跑，重标定 ticks/m
  ```
  然后修正 `config/motor.yaml` 的 channel_a_wheel / channel_b_wheel /
  encoder1_wheel / encoder2_wheel / encoder_sign_*，并更新本文件。
- 几何参数沿用 OPi 标定：wheel_radius 0.0318m、wheel_separation 0.155m、
  ticks/m 左 1679 右 1656（4x）——**X5 上需复核**。

## UART1 / 总线舵机（引脚 8/10）——暂停状态

- 40PIN 8 脚 = **UART1 TX**、10 脚 = **UART1 RX**，设备 `/dev/ttyS1`，
  3.3V TTL，默认使能（X3 是 UART3/ttyS3，X5 是 UART1/ttyS1，别搞混）
- 实测：ttyS1 可打开、可发送；但 DT 里 **UART1 开启了 RS485 模式**
  （TIOCGRS485 ioctl 返回 flag 0x2 = SER_RS485_ENABLED，实测 2026-09-10）。
  RS485 模式下发送时接收器被关闭，全双工外设（如舵机控制器）的应答
  可能收不到——这是通信排查的首要怀疑点。
- 实测向 ttyS1 发 LOBOT/SCS 协议帧（8 个波特率 × 2 协议 × 2 ID 扫描）：
  无任何合法应答；高波特率下 RX 收到的字节只取决于 RX 波特率、与 TX
  波特率无关 → 是发送时的边沿串扰/回波，不是控制器应答。
- **结论（阶段性）**：8/10 两脚的 UART 功能可用（有电平活动、内核 rx
  有计数），但当前接线/模式与控制器通信不通。按用户要求暂停实测。
- 恢复时按顺序排查：
  1. `python3 tools/uart_loopback_test.py`（8/10 短接跳线）→ 验证板侧
  2. 确认 TX(8)↔控制器 RX、RX(10)↔控制器 TX **交叉**、共地、控制器供电
  3. 若仍不通：禁用 UART1 的 RS485 模式（改 DT / srpi-config），或
  4. 换 USB-TTL（CH340→/dev/ttyUSB0，udev 规则已写好 ttyUSB_servo 软链）
- 工具：`tools/servo_bus_test.py`（LOBOT 0x55 0x55 / SCS 0xFF 0xFF 协议
  扫描，只读指令不动作）、`tools/uart_loopback_test.py`

## GS130WI 双目 IMU 相机（MIPI，实测通过）

- **接口是 MIPI CSI-2，不是 USB**。传感器双 SC132GS（i2c-6@0x32 已探测），
  IMU = ICM-42688-P（overlay 已在 /boot/config.txt：iio 设备
  icm42688-gyro / icm42688-accel / icm42688-fsync 实测存在）
- 相机模块背后开关需在 EXT 模式；**黑色同步线接 40PIN 37 脚**（用户确认）；
  开 LPWM 硬件同步用 `mipi_lpwm_enable:=true`
- EEPROM 标定自动读取（camType 17, cal_tpye 1, d_num 4）；
  tros 包内 `calib_params.yaml` 不存在（驱动日志有提示，EEPROM 替代）
- 实测 launch（已验证出数据）：
  ```bash
  source /opt/tros/humble/setup.bash
  ros2 launch robowheels gs130.launch.py   # 或直接 mipi_cam 包
  ```
  实测话题（816x960, nv12, gdc on, 10fps）：
  - `/image_combine_raw`：816x**1920** nv12（上下拼，2350080B）
  - `/image_left_raw` `/image_right_raw`：816x960 nv12（各 1175040B，
    Y 平面 = 前 640 行 → mono 816x640）
  - `/image_left_raw/camera_info` `/image_right_raw/camera_info`
  - `/imu_data`：sensor_msgs/Imu，**~100Hz**，重力轴 ~9.66（平放实测）
- `gs130_vio_bridge` 节点把上述话题转成 VIO 规范：
  `/cam0/image_raw` `/cam1/image_raw`（mono8 816x640）、`/imu0`、
  camera_info 透传、静态 TF imu0→cam0/cam1（基线 0.07m，YAML 可改）
- 后续 VIO（VINS-Fusion / FAST-LIVO / OpenVINS）消费 cam0/cam1/imu0；
  标定参数从 `/cam*/camera_info` 读取（双目为鱼眼模型，VIO 配置要匹配）

## 双目点云（stereo_pointcloud_node，2026-09-10 实测）

- 本包自研 CPU 立体匹配节点（不依赖 hobot_stereonet）：
  `ros2 launch robowheels stereo_pointcloud.launch.py`
  （自启 mipi_cam 816x960 nv12 gdc；与 gs130.launch.py / run_stereo.sh 互斥）
- 标定来源（mipi_cam 驱动日志 log_level:=info，实测提取）：
  - 矫正平面 rectify `[f, cx, cy, baseline] = [511.53, 539.059, 681.172, 0.06889]`
    （1088x1280；baseline 与规格书 70mm 吻合）
  - 816x960 GDC 输出 = 精确 0.75 缩放 → 输出平面 `f=383.65, cx=404.29, cy=510.88`
    （config/stereo_pc.yaml；如果云形变先怀疑 fx/fy）
- 实测性能（SGBM, downsample=2, ndisp=96）：**~3.7Hz**，~12K 点/帧，
  CPU 2.5 核；深度中位 0.83m / max 2.03m（室内场景合理）
  - 提速：downsample 3~4 / matcher: bm；提精度：downsample 1 + ndisp 112
- 发布：/stereo/points（x/y/z/intensity, frame camera_link）、
  /stereo/depth（32FC1 米）、/stereo/disparity、静态 TF imu0→camera_link
  （左目位于 imu -x 0.035m）
- 备选 hobot_stereonet（BPU，~30fps）话题 /StereoNetNode/stereonet_pointcloud2
  已真机验证，rviz 配置里默认关闭可对比

## Astra Pro 奥比中光（USB，实测）

- lsusb：`2bc5:0501 Astra Pro HD Camera`（RGB）+ `2bc5:0403 Astra Pro`
  （深度/IR，Vendor Specific Class，内核 UVC 不绑定）
- **RGB = 普通 UVC**：/dev/video0 实测，YUYV 640x480@30 或
  MJPG 1280x720@30（`v4l2-ctl --list-formats-ext` 确认）
  → `astra_rgb_node`（OpenCV，无 SDK 依赖）已可工作
- **IR/深度需要官方驱动，实测 orbbec_camera 2.8.6（OrbbecSDK v2）不兼容本机
  Astra Pro**：驱动能枚举设备，但初始化握手失败
  （`Device response size(0) < response header size(10), propertyId: 1000`，
  循环 reset 重试）。经典 Astra Pro 固件需要 OrbbecSDK v1 / OpenNI2 系驱动。
  备选路径（任选，脚本见 tools/setup_astra_ir.sh）：
  1. PC 上用 OrbbecView 升级相机固件后重试 v2 驱动（推荐，一劳永逸）
  2. 源码构建 ros2_astra_camera（openni2 系，本机已装 libopenni2-0 2.2.0.33，
     还需 Orbbec 的 openni2 驱动模块）
  3. 只用 RGB：`ros2 launch robowheels astra_camera.launch.py use_sdk:=false`
     （UVC 兜底节点，当前已可用）
- udev：/dev/astra_rgb 软链（规则已写好）

## 手柄（实测）

- DragonRise Inc. Controller（0079:181c），/dev/input/js0 存在
- joy + teleop_twist_joy（apt: ros-humble-joy-linux / ros-humble-teleop-twist-joy）
- `ros2 launch robowheels joy.launch.py`：左摇杆纵向=linear.x（0.5 m/s），
  右摇杆横向=angular.z（2.0 rad/s），RB（键 5）为使能键

## ROS2 图（目标状态）

- motor_driver：/cmd_vel 入，/odom（+odom→base_footprint TF）、
  /wheel_speeds、/joint_states 出
- joy → teleop_twist_joy → /cmd_vel
- gs130_vio_bridge：/cam0/image_raw /cam1/image_raw /imu0 + camera_info
- astra：/astra/... 或官方驱动 /color /depth /ir 话题族

---

# Orange Pi 3 LTS（2026-08-26 实测，已退役，历史记录）

## 系统环境（已验证）

- Orange Pi 3 LTS, kernel 5.16.17-sun50iw6, Ubuntu 22.04
- ROS2 Humble, `/opt/ros/humble`；wiringOP gpio v2.46；OPi.GPIO 0.5.2（SUNXI 模式）
- Python 3.10.12（注意：未装 smbus2/smbus/gpiod）

## 电机实测映射（2026-08-26 修正）

原始接线表标注 A=左、B=右，**实测相反**：

| 通道 | 引脚 | 实测驱动 | 前进方向 |
|---|---|---|---|
| A | AIN1=PL10(wPi8), AIN2=PH5(wPi11) | **右轮** | AIN2(PH5)=PWM, AIN1=LOW |
| B | BIN1=PH4(wPi14), BIN2=PH6(wPi12) | **左轮** | BIN2(PH6)=PWM, BIN1=LOW |

- 后退 = IN1 PWM, IN2 LOW（与前进相反）；AT8236 无 STBY 引脚，IN1/IN2 全 LOW = 滑行
- 软件 PWM 200Hz 实测可用；低速测试 20% 占空比两轮均正常转动
- 整车双轮 IN2 PWM → 小车**前进**（已实测确认）

## 编码器（2026-08-26 实测）

- **左轮编码器 = RA/RB (PL2/PL3)**，右轮编码器 = LA/LB (PD24/PD23)（与接线表交叉）
- 前进时计数正向递增 ✅；4x 正交解码
- 标定（直线跑 ×2，卷尺实测，均值）：右轮 1656 ticks/m，左轮 1679 ticks/m（4x）；
  ticks/rev (4x) ≈ 334（轮周长 0.20m）；wheel_radius = 0.0318 m
- 低速 PWM 下转速与占空比非线性（12%~20% 占空比都约 0.5 m/s）；两轮速度不完全对称

## I2C / MPU6050（已验证，X5 已移除）

- i2c-0 总线；MPU6050 @ 0x68，WHO_AM_I=0x68 ✅

## LiDAR COIN-D4（已验证，X5 已移除）

- USB-TTL: CH340 → /dev/ttyUSB0；udev → /dev/sc_mini；驱动 cspc_lidar_sdk_ros2
- baudrate 230400；/scan @ 10Hz，每圈 345~401 点

## 安全红线（每步都要遵守，X5 同样适用）

- 启动时电机引脚必须先 OUTPUT LOW
- /cmd_vel 500ms 无更新 → 停车（已实现 watchdog）
- 节点退出 → 引脚置 LOW；**禁止 GPIO.cleanup()**（引脚变输入悬空，AT8236 可能误转）
- 测试用低速（max_pwm 当前 40%）
- 电机/编码器/轮距参数全部放 config/motor.yaml，不要写死在代码里
