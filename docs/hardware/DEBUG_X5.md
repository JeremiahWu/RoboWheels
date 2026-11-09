# RDK X5 真机调试与 Mac RViz

厂家资料核对与更新后的模块接口定义见 [AT8236_REVIEW.md](AT8236_REVIEW.md)。当前已确认 MC130 1:48、编码器 3.3V、电机 VM 不超过 9V。

## 连接范围

X5：`root@192.168.124.24`，项目 `/root/RoboWheels`，ROS 2 Humble，`ROS_DOMAIN_ID=0`。本次仅处理 GS130WI（EXT，黑色同步线接 BOARD 37）、AT8236 和两组编码器。其他外设未启动。

## Mac 启动可视化

默认使用浏览器中的 RViz（noVNC），只需启动 Docker Desktop，然后在 Mac 仓库根目录执行：

```bash
python3 scripts/rdk_debug.py start
python3 scripts/rdk_debug.py status
# 停止 Mac RViz 与遥测，不停止 X5 传感器
python3 scripts/rdk_debug.py stop
```

打开 http://localhost:16081/vnc.html?autoconnect=1&resize=scale 。页面只监听本机。

XQuartz/VirtualGL 在本次多图像面板调试中出现退出码 139，因此默认改用浏览器显示。需要手动尝试原生显示时，先启动 XQuartz，再用 `ROBOWHEELS_DEBUG_DISPLAY=xquartz python3 scripts/rdk_debug.py start`。

此入口使用现有 `robowheels:xquartz-humble` 镜像。镜像未构建时，参见 [XQuartz 环境说明](../simulation/XQUARTZ.md)。启动脚本会把只读转发程序复制到 X5 的 `/tmp`，需要现有 SSH 登录权限；不会启动电机节点或发布 `/cmd_vel`。

数据沿 X5 ROS → SSH → Mac Docker ROS → RViz 传输，避免跨 Docker Desktop 网络直接做 DDS 发现。本地 ROS 域 0 仅开启 localhost，传感器时间戳与 TF 保留，不生成虚假的相机到底盘外参。只转发白名单内的图像、点云、IMU、TF、里程计、轮速与关节状态。断线后自动重连；如果 X5 重启，仍需先恢复真机传感器节点。

RViz 默认 Fixed Frame=`camera_link`，显示合并的左右目、双目深度、点云、IMU 加速度箭头/数值，以及 A/B 编码器原始计数。左右目和深度仅在显示副本中旋转 180°；X5 的原始图像、CameraInfo、点云和 IMU 未改变。箭头表示加速度（包含重力），不是姿态估计。里程计显示默认关闭；没有实际相机与底盘外参时，不能将其与点云可靠叠加。

## X5 启动双目与 IMU

```bash
cd /root/RoboWheels
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robowheels gs130.launch.py mipi_lpwm_enable:=true
```

相机初始化需要等待；不能同时启动第二份 mipi_cam。另开终端，使用相同 source 环境，仅启动点云计算：

```bash
ros2 run robowheels stereo_pointcloud_node --ros-args \
  --params-file /root/RoboWheels/ros2_ws/src/robowheels/config/stereo_pc.yaml
```

不要同时运行 `stereo_pointcloud.launch.py`，因为它也会启动相机。

| 数据 | 话题 |
|---|---|
| 左右目 mono8 | `/cam0/image_raw`、`/cam1/image_raw` |
| 原始/桥接 IMU | `/imu_data`、`/imu0` |
| 双目点云、深度 | `/stereo/points`、`/stereo/depth` |
| 电机节点运行后的反馈 | `/joint_states`、`/wheel_speeds`、`/odom` |

## 底盘与手柄

X5 上启动（避免重复启动已有同类节点）：

```bash
source /opt/ros/humble/setup.bash
source /root/RoboWheels/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robowheels teleop.launch.py
```

按住 RB 启用，左摇杆前后、右摇杆左右；线速度指令上限 0.3m/s，PWM 上限 20%。松开 RB 或指令超时停车。完整操作见 [TELEOP_X5.md](TELEOP_X5.md)。

编码器由 motor_driver 的 GPIO v2 事件采样统一读取，不另开轮询监控。反馈包括 `/rdk_debug/encoder_counts`（原始 `[E1,E2]`）、`encoder_rates`、`encoder_invalid`、`encoder_event_drops`，以及 `/wheel_speeds`、`/wheel_speed_targets`、`/joint_states` 和 `/odom`。

## 模型与验证

[BASE_MODEL.md](BASE_MODEL.md) 记录 STEP 几何、MC130 分辨率与地面验证边界。工具按用途保留：

| 工具 | 用途 |
|---|---|
| `tools/validation/validate_drive_model.py` | 本地无动作测试，核对 STEP/仿真/真机几何及运动学 |
| `tools/validation/validate_encoder_events.py` | 本地无动作测试，核对事件解码、排序和丢失诊断 |
| `tools/validation/validate_joy.py` | X5 隔离话题验证摇杆映射与 RB，不接电机 |
| `tools/validation/validate_motor_teleop.py` | X5 架空后短时验证前后驱动、轮角与超时停机 |
| `tools/hardware/base/calibrate_encoder_turns.py` | 手动转轮、记录稳定基线及每圈计数 |
| `tools/validation/validate_x5_wiring.py` | 独立接线排查；必须先停止电机驱动，不能与事件采样同时占用引脚 |

接线排查工具中的轮询仅用于观察，不作为当前标定计数。已有电机与编码器 GPIO 工具属于独立低层排查入口，运行前须核对引脚复用并停止占用它们的节点。

运行报告、认证文件、代理 PID、标定原始数据统一放在被忽略的 `.local/`。原始 STEP 和厂家 PDF 不纳入运行镜像或本次提交。
