# robowheels — RDK X5 底盘 + 相机 ROS2 包

RoboWheels 两轮差速底盘在 RDK X5 上的完整控制与感知节点。
硬件实测记录见仓库根目录 `HARDWARE.md`（所有引脚/话题/波特率结论均来自真机验证）。

## 节点

| 节点 | 功能 | 验证状态 |
|---|---|---|
| `motor_driver` | AT8236 双通道软 PWM + 编码器 4x 解码 + /odom + TF + /wheel_speeds + /joint_states，/cmd_vel watchdog | ✅ 启动/话题验证（电机运动测试待实车） |
| `astra_rgb_node` | Astra Pro RGB（UVC/OpenCV，无 SDK 依赖）→ /astra/rgb/image_raw + camera_info | ✅ 30fps 出图 |
| `gs130_vio_bridge` | GS130WI（mipi_cam 驱动后）：左右目 nv12→mono8 → /cam0 /cam1 image_raw + camera_info、/imu_data→/imu0、静态 TF | ✅ 双目+IMU 全部出数据 |
| `stereo_pointcloud_node` | GS130WI 双目 SGBM 立体匹配 → /stereo/points + /stereo/depth（32FC1 米）+ /stereo/disparity + TF；参数 config/stereo_pc.yaml（含实测标定） | ✅ 真机 3.7Hz 实测 |
| `orbbec_camera`（官方 apt 包） | Astra IR/深度 | ❌ SDK v2 与老 Astra Pro 固件不兼容（实测），备选见 tools/setup_astra_ir.sh |

手柄链：joy_node + teleop_twist_joy → /cmd_vel（✅ 已验证启动，DragonRise 手柄）。
舵机 UART：暂停中，工具 tools/uart_loopback_test.py / tools/servo_bus_test.py。

## 环境（重要）

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash        # tros: mipi_cam / hobot_shm
source ~/RoboWheels/ros2_ws/install/setup.bash
```

## 启动

```bash
# 全部（底盘 + 手柄 + 双目 + Astra）
ros2 launch robowheels bringup.launch.py

# 单独
ros2 launch robowheels motor.launch.py           # 底盘
ros2 launch robowheels joy.launch.py             # 手柄
ros2 launch robowheels gs130.launch.py           # 双目IMU (VIO 话题)
ros2 launch robowheels astra_camera.launch.py    # Astra (SDK)
ros2 launch robowheels astra_camera.launch.py use_sdk:=false   # Astra RGB 兜底
```

## 首次实车验证顺序（必做）

1. 车架空转（或架起轮子）后：
   ```bash
   python3 tools/motor_test.py A fwd 20 2    # 确认 A 通道驱动哪个轮
   python3 tools/motor_test.py B fwd 20 2
   python3 tools/encoder_test.py watch       # 手转轮子确认编码器归属
   python3 tools/encoder_test.py drive 20 4  # 直线跑，重标定 ticks/m
   ```
2. 按实测修正 `config/motor.yaml`（channel_a_wheel、channel_b_wheel、
   encoder1_wheel、encoder2_wheel、encoder_sign_*、ticks_per_meter_*）
   并同步更新 HARDWARE.md
3. 手柄控制：
   ```bash
   ros2 launch robowheels motor.launch.py
   ros2 launch robowheels joy.launch.py
   ```
   按住 RB（使能键 5），左摇杆前后走，右摇杆左右转。

## 话题速查

- 控制入：`/cmd_vel`（Twist）
- 底盘出：`/odom`（nav_msgs/Odometry）、`/tf`（odom→base_footprint）、
  `/wheel_speeds`（[v_left, v_right] m/s）、`/joint_states`
- VIO（GS130）：`/cam0/image_raw` `/cam1/image_raw`（mono8 816x640）、
  `/cam0/camera_info` `/cam1/camera_info`、`/imu0`（~100Hz）
- Astra：`/astra/rgb/image_raw` + camera_info（SDK 驱动还有
  /color /depth /ir 话题族）

## 容器化运行（Docker + uv）

仓库根目录 `Dockerfile` 固定了完整运行环境：Ubuntu 22.04 + ROS2 Humble
（apt）+ TROS mipi_cam（D-Robotics 源）+ uv 管理的 Python 环境
（Hobot.GPIO 从 `third_party/hobot_gpio` 以 path 依赖安装——该包未发布
PyPI；另有 pyserial、colcon），并在镜像内完成 ros2_ws 的 colcon 构建。
依赖锁文件 `uv.lock` 已提交（按清华 PyPI 镜像生成）。

```bash
# X5 上先装 docker（若未装）：sudo apt-get install -y docker.io

# 构建（镜像源默认清华，境外构建需 --build-arg 覆盖，见 Dockerfile 注释）
docker build -t robowheels:rdkx5 .

# 运行（privileged + 设备挂载是 GPIO/UART/MIPI/IMU 的前提）
docker run -it --rm --privileged --network host \
    -v /dev:/dev -v /sys:/sys:rw \
    robowheels:rdkx5 \
    ros2 launch robowheels bringup.launch.py

# 或 compose
docker compose up -d      # 默认 bash 交互；command 可改成 bringup
```

容器内的 uv 环境：`/opt/venv`（--system-site-packages，ROS python 栈
rclpy/cv_bridge/cv2 来自 apt 保持可见；entrypoint 会把 venv 的
site-packages 加进 PYTHONPATH 供系统 python3 运行的 ros2 CLI 使用）。
