# RoboWheels

两轮差速底盘机器人（RDK X5 平台）——底盘控制 + 多相机感知 + 机械臂伺服的完整 ROS2 Humble 软件栈。

> 平台：D-Robotics RDK X5（Ubuntu 22.04 / RDK OS 3.5.0），由 Orange Pi 3 LTS 迁移而来（已移除 LiDAR 与 MPU6050）。
> 所有硬件结论均来自真机实测，见 **[HARDWARE.md](HARDWARE.md)**。

---

## 系统组成

| 子系统 | 硬件 | 软件 | 状态 |
|---|---|---|---|
| 底盘 | TT 电机 ×2，AT8236 双通道驱动，正交编码器 ×2 | `motor_driver`（软 PWM + 4x 解码 + odom） | ✅ 节点验证，电机映射待实车确认 |
| 控制 | 手柄（DragonRise） | joy + teleop_twist_joy → `/cmd_vel` | ✅ 已验证 |
| 感知① | Orbbec Astra Pro（USB，底盘上方，机械臂动作感知） | `astra_rgb_node`（UVC RGB 出图） | ✅ 30fps 出图 |
| 感知② | RDK GS130WI 双目 IMU（MIPI，底盘底部，VIO+ESDF planning 预留） | `gs130_vio_bridge` → `/cam0` `/cam1` `/imu0`；`stereo_pointcloud_node` → `/stereo/points` `/stereo/depth` | ✅ 双目+IMU 全部出数据；✅ 自研 SGBM 点云 3.7Hz 实测 |
| 机械臂 | LewanSoul 总线舵机（LX-15D / LD-27MG，40PIN 8/10 脚） | UART 测试工具已备 | ⏸ 通信排查暂停（见 HARDWARE.md） |

## 硬件连接速查（40PIN BOARD 脚号）

| 功能 | 引脚 |
|---|---|
| 电机 AIN1 / AIN2 / BIN1 / BIN2 | 27 / 29 / 31 / 33 |
| 编码器 E1A / E1B / E2A / E2B | 11 / 13 / 15 / 16 |
| 舵机控制器（UART1：TX/RX） | 8 / 10（`/dev/ttyS1`） |
| GS130WI 同步线（LPWM） | 37 |
| 手柄 | USB（`/dev/input/js0`） |
| Astra Pro | USB（`/dev/video0`） |

> ⚠️ 注意：8/10 脚是 **UART1 串口**（不是普通 GPIO）；4 个电机脚的硬件 PWM 功能被引脚复用占用，当前用软件 PWM。详见 HARDWARE.md。

## 目录结构

```
.
├── README.md                  # 本文件
├── HARDWARE.md                # 硬件实测记录（引脚/驱动/话题/排查结论，必读）
├── Dockerfile                 # 容器化运行环境（ROS2 Humble + TROS + uv）
├── compose.yaml               # docker compose 配置（privileged + 设备挂载）
├── pyproject.toml / uv.lock   # uv 管理的 Python 依赖（已锁定）
├── docker/                    # entrypoint.sh、sunrise.gpg 签名密钥
├── third_party/hobot_gpio/    # Hobot.GPIO 0.0.2 vendor 版（未发布 PyPI）
├── tools/                     # 硬件调试脚本（见下）
├── models/                    # 机械臂/底盘 SolidWorks 模型
├── prompt.md                  # 历史交接文档（OPi 时期，仅存档）
└── ros2_ws/src/robowheels/    # ROS2 包（节点/launch/配置/udev，含包级 README）
```

`tools/` 调试脚本：

| 脚本 | 用途 |
|---|---|
| `motor_test.py [A|B|AB] [fwd|rev] [duty%] [s]` | 单通道电机测试（确认通道↔轮子） |
| `encoder_test.py watch\|count\|spin\|drive` | 编码器归属确认 + ticks/m 重标定 |
| `pin_sampler.py <PIN> [s]` | 高速引脚占空比采样（验证 PWM 输出） |
| `uart_loopback_test.py` / `servo_bus_test.py` | 舵机 UART 板侧自测 / 协议扫描（8/10 短接） |
| `setup_astra_ir.sh` | Astra Pro IR 驱动备选安装（OpenNI2 路线） |

## 快速开始（原生运行）

```bash
# 1. 环境（顺序重要：humble 提供 joy/teleop，tros 提供 mipi_cam）
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ~/RoboWheels/ros2_ws/install/setup.bash

# 2. 构建（已构建过可跳过）
cd ~/RoboWheels/ros2_ws && colcon build --symlink-install

# 3. 启动（底盘 + 手柄 + 双目 + Astra 全部）
ros2 launch robowheels bringup.launch.py

# 或按需单独启动
ros2 launch robowheels motor.launch.py
ros2 launch robowheels joy.launch.py
ros2 launch robowheels gs130.launch.py
ros2 launch robowheels astra_camera.launch.py              # 官方 SDK 驱动
ros2 launch robowheels astra_camera.launch.py use_sdk:=false  # UVC RGB 兜底
```

**手柄操作**：按住 RB（键 5，使能键）→ 左摇杆前后 = 前进后退，右摇杆左右 = 原地转向。

## 容器化运行

环境已固化为 Docker 镜像（依赖由 uv 管理并锁定）：

```bash
docker build -t robowheels:rdkx5 .
docker run -it --rm --privileged --network host \
    -v /dev:/dev -v /sys:/sys:rw \
    robowheels:rdkx5 ros2 launch robowheels bringup.launch.py
```

`--privileged` + `/dev` + `/sys` 挂载是 GPIO/UART/MIPI 相机/IMU 的前提。
镜像源默认清华（可 `--build-arg` 覆盖）。详见 `Dockerfile` 注释与包级 README。

## RViz 可视化（红外 + 双目点云）

`config/rviz/perception.rviz` 预置了感知视图（话题与 QoS 均按真机实测配置，
全部 RELIABLE）：

- 双目点云 `/stereo/points`（本包 SGBM 节点，Fixed Frame: `camera_link`）
- 双目深度图 `/stereo/depth`（32FC1 米）
- 红外 `/astra/ir/image_raw`
- hobot_stereonet（BPU）点云/深度/矫正图、Astra RGB / VIO cam0
  （默认关闭，面板里手动开启对比）

```bash
# PC 上（与 X5 同一 ROS_DOMAIN_ID，配置按实际网络设置）：
rviz2 -d ros2_ws/src/robowheels/config/rviz/perception.rviz
# 或 X5 本机（装了 rviz2 + 显示器）：
ros2 launch robowheels rviz_perception.launch.py
```

点云数据源二选一（**都会独占 mipi 相机，不要同时跑**）：

```bash
# ① 本包自研节点（OpenCV SGBM，实测 ~3.7Hz @ 下采样2，CPU 2.5 核；
#    调速改 config/stereo_pc.yaml 的 downsample / matcher: bm）
ros2 launch robowheels stereo_pointcloud.launch.py

# ② hobot_stereonet（BPU 加速 ~30fps @640x352，精度更高）
bash /opt/tros/humble/share/hobot_stereonet/script/run_stereo.sh
```

⚠️ 注意：
- 红外当前**无数据**：官方 SDK 驱动与本机 Astra Pro 固件不兼容
  （详见「已知问题」），配置已就绪，等 IR 驱动恢复后自动出图；
  现在能直接看的是 Astra RGB（UVC 那路）与双目相关画面
- Astra 深度点云（`/astra/depth/points`）同样要等 SDK 驱动恢复，
  本包不自行生成（单目 RGB 无法产生度量级点云）

## 节点与话题

| 节点 | 订阅 | 发布 |
|---|---|---|
| `motor_driver` | `/cmd_vel` (Twist) | `/odom`、`/tf`（odom→base_footprint）、`/wheel_speeds`、`/joint_states` |
| `joy_node` + `teleop_twist_joy` | 手柄设备 | `/joy` → `/cmd_vel` |
| `gs130_vio_bridge` | `/image_left_raw` `/image_right_raw` `/imu_data` | `/cam0/image_raw` `/cam1/image_raw`（mono8 816x640）、`/cam0|cam1/camera_info`、`/imu0`（~100Hz） |
| `stereo_pointcloud_node` | `/image_left_raw` `/image_right_raw` | `/stereo/points`（PointCloud2，frame `camera_link`）、`/stereo/depth`（32FC1 米）、`/stereo/disparity`、静态 TF `imu0→camera_link` |
| `astra_rgb_node` | UVC `/dev/video0` | `/astra/rgb/image_raw` + camera_info |
| `mipi_cam`（tros 驱动） | GS130WI MIPI | `/image_combine_raw`、`/image_left_raw`、`/image_right_raw`、`/imu_data` |

坐标系：`odom → base_footprint`（轮式里程计），`imu0 → cam0/cam1`（双目基线 0.07 m，静态 TF，YAML 可调）。

## 实车验证清单（首次接线后必做）

1. **电机映射**：`python3 tools/motor_test.py A fwd 20 2`（再测 B）确认哪个通道驱动哪个轮
2. **编码器归属**：`python3 tools/encoder_test.py watch` 手转轮子
3. **里程计标定**：`python3 tools/encoder_test.py drive 20 4` 直线跑，卷尺实测 → 更新 `config/motor.yaml` 的 `ticks_per_meter_*`
4. 按实测结果修正 `channel_a_wheel` / `channel_b_wheel` / `encoder*_wheel` / `encoder_sign_*`，并同步更新 HARDWARE.md
5. 手柄试跑（低速，`max_pwm: 40` 已限速）

## 已知问题 / 待办

- **Astra Pro IR**：官方驱动（OrbbecSDK v2）与本机相机固件握手失败；RGB 已可用。IR 备选：PC 上 OrbbecView 升级固件重试，或 `tools/setup_astra_ir.sh`（OpenNI2）
- **舵机 UART（8/10 脚）**：板侧 UART1 可用，但 DT 开启了 RS485 模式且控制器无应答，按需恢复排查（步骤见 HARDWARE.md）
- **硬件 PWM**：4 个电机脚有硬件 PWM 功能但被引脚复用占用（需 srpi-config 释放），当前软件 PWM 200Hz
- **VIO**：GS130 话题已按 VINS/FAST-LIVO 规范发布，双目为鱼眼模型（标定随 camera_info），VIO 算法配置需与之一致
- **双目点云**：本包 SGBM 节点为 CPU 实现（~3.7Hz），跑 ESDF planning 若需更高帧率/精度，切换 hobot_stereonet（BPU，~30fps）

## 安全红线（每次改动都要遵守）

- 电机引脚启动必须先 OUTPUT LOW；节点退出只置 LOW，**禁止 GPIO.cleanup()**（AT8236 输入悬空可能误转）
- `/cmd_vel` 500ms 无更新自动停车（watchdog）
- 参数一律放 `config/*.yaml`，不写死进代码
