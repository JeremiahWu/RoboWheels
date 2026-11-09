# RDK X5：BPU 深度、ORB 视觉里程计与 ESDF 规划预览

本版基于 GS130WI 和 X5 本机已有 TROS 包，不引入 TensorRT/CUDA，不修改电机接线或恢复自动运动。代码在 Mac 整理，再同步到 `/root/RoboWheels`。

## 当前链路

```text
GS130WI 双目/IMU → 官方 mipi_cam
  ├─ 合并双目 → hobot_stereonet（BPU）→ 校正图像、深度
  │                                  ├─ ORB + Hamming + PnP-RANSAC → VO/轨迹
  │                                  └─ 米制光学点云 → 局部栅格/ESDF → 路径预览
  └─ IMU → /imu0（数据已通；待标定后接惯性融合）
```

ORB 在 CPU 上提取 800 个以内的特征，使用双目深度建立前一帧 3D 点到当前帧 2D 点的约束。它属于视觉里程计前端，**不是完整 VIO 或 ORB-SLAM3**：目前无 IMU 预积分、偏置估计、回环、重定位与全局地图。失跟踪开启新 `vo_odom_N` 段，不伪造连续位姿；输出协方差为保守占位，不应直接用于生产级融合。

## 为什么用两个官方组件

| 组件 | 本次核对 | 使用方式 |
| --- | --- | --- |
| `hobot_stereonet` | X5 已安装 2.5.5 与多种 `.bin` 模型 | 实机运行 `DStereoV2.4_int8_544_448_96.bin`；神经网络深度推理在 BPU |
| `hobot_vio` | 已安装 2.1.1；默认 RealSense D435i 配置 | 保留独立接入入口；没有 GS130WI 相机—IMU 标定，不启用惯性定位 |
| ORB/PnP、ESDF/搜索 | CPU 几何运算 | 不把这些算法声称为 BPU 加速 |

StereoNet 只解决双目深度，不替代 VIO。hobot_vio 与自建 ORB VO 是可选定位后端；目前未修改其内部特征提取实现，也不声称其使用 ORB。参考 [官方深度组件](https://github.com/D-Robotics/hobot_stereonet)、[官方 VIO](https://github.com/D-Robotics/hobot_vio)、[GS130WI 快速开始](https://developer.d-robotics.cc/accessories_stereo_camera_doc/stereo_camera_gs130wi/quick_start)。

## 启动与停止

先正常停止其他拥有 MIPI 相机的 launch，不能同时运行 `gs130.launch.py`、`stereo_pointcloud.launch.py`、官方 `run_stereo.sh` 与本启动文件。

```bash
cd /root/RoboWheels
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select robowheels
source install/setup.bash
ros2 launch robowheels navigation_bpu.launch.py
```

前台运行可用 Ctrl+C 停止。本次调试后台日志在 `/root/RoboWheels/.local/navigation-bpu.log`；不提交日志和构建缓存。该 launch **没有电机节点，没有 `/cmd_vel` 发布者**。

默认图像 544×448、10Hz、GDC 开启、LPWM 开启、EXT 开关；同步线使用已核对的 BOARD 37。相机倒装保持传感器原方向，不使用观看用的旋转副本进行定位。

本次发现 StereoNet 2.5.5 在输入与模型尺寸不同时，可能在 CameraInfo 回调和图像 resize 后重复缩放内参。这里让相机与模型尺寸一致，避免第二次缩放。切换模型时必须重新核对输入尺寸、输出 CameraInfo、深度尺度及延迟，不能只换模型文件名。GDC 在不同输出尺寸可能重新计算虚拟相机内参，不能盲目套用另一分辨率的固定 K。

StereoNet 输出 `mono16` 深度以毫米计，适配器显式乘 0.001 转为米，拒绝不支持的编码；0.2～5m 以外为 NaN。点云直接用同一帧深度与匹配内参反投影，不沿用旧的硬编码 SGBM 内参。米制量纲不代表深度绝对精度已通过实物量测验收。

## 话题与坐标

| 话题 | 用途 |
| --- | --- |
| `/cam0/image_raw`、`/cam1/image_raw`、`/imu0` | 保留的传感器与后续标定输入 |
| `/perception/rectified/image`、`/perception/rectified/camera_info` | ORB 使用的校正图像及匹配内参 |
| `/stereo/depth` | 32FC1 米；无效深度为 NaN |
| `/stereo/points` | 米制 XYZ，`camera_optical_frame`，保留采集时间戳 |
| `/perception/orb_features` | ORB 特征叠加图，保持原始朝向 |
| `/localization/vo`、`/localization/vo_path` | 相机光学坐标位姿及分段轨迹 |
| `/localization/status` | 特征数、匹配数、内点数、段号、跟踪状态与 `imu_fused=false` |
| `/perception/local_grid` | 120×120、分辨率 0.05m；未观测=-1，已观测自由=0，障碍及膨胀=100 |
| `/perception/esdf` | 120×120 32FC1，距离单位 m；行列与栅格对应，不是透视相机图像 |
| `/goal_pose` | `camera_nav` 内的相对目标；不是底盘或世界坐标目标 |
| `/planning/path`、`/planning/status` | ESDF 代价引导的 A* 路径预览与原因状态 |

`camera_optical_frame` 为右/下/前。官方 StereoNet 发布标准 `camera_link → camera_optical_frame` 换轴。ORB 可发布 `vo_odom_N → camera_link`；这只是 VO 可视化树，不代表与底盘已经标定。`camera_nav` 是光学坐标的换轴：前=光学 Z，左=-光学 X，上=-光学 Y；相机倒装时它不代表机器人实际左右或重力方向。

ESDF 将未知区域保守视作阻挡，正值在已观测自由空间、负值在阻挡区域。地图边界也作为障碍。默认 0.18m 膨胀半径只是预览参数，尚未验证整机外形；高度筛选相对相机，**不是重力对齐的地面分割**。相机近处盲区或稀疏深度可能使起点不可通行，这时正确输出空路径，不能把未知区强行清空。

路径搜索结合距离代价避开贴障，禁止穿过未知区和对角夹角。它不是轨迹优化、ESDF 梯度控制或已验证的差速可执行轨迹；还缺曲率/速度/加速度约束、动态障碍与执行控制。没有目标、数据过期、目标坐标系不匹配或无通路时发布空路径。

仅测试目标输入（不驱动底盘）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: camera_nav}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
```

目标是随相机定义的局部相对点，尚不支持世界固定目标。使用 `navigation_preview.launch.py` 可运行旧 CPU SGBM 备用链路，但其标定参数对应 816×960，不能当作 BPU 544×448 的替换参数。

## 检查与可视化

X5 source 工作区后，在仓库根目录：

```bash
python3 tools/validation/validate_orb_esdf.py
python3 tools/validation/validate_navigation_grid.py
python3 tools/validation/validate_planning_preview.py
```

检查包括合成平面上的已知位移尺度、静止帧、失跟踪、时间倒序、ESDF 符号和单位、障碍绕行、未知区拒绝、ROS 坐标系不匹配与过期路径清空。ROS 规划测试使用独立 `/validation/*` 话题，不接触真实目标或电机。

Mac 的只读转发白名单已加入 ORB 图、VO/轨迹、ESDF、栅格与状态；未加入目标或运动控制转发。RViz 配置增加特征图、局部地图、VO 轨迹和规划路径。若使用旧的遥测进程，需重新启动 `scripts/rdk_debug.py` 使其加载新白名单；GUI 显示仍受当前运行的配置与 TF 状态影响。

## 下一步

按 [相机—IMU 标定指南](GS130WI_IMU_CALIBRATION.md)生成外参、时间偏移和噪声参数，再选择 hobot_vio 或保留 ORB 前端的惯性融合后端。之后测量相机到底盘外参，验证移动轨迹、深度实测误差、闭合路线与失跟踪恢复，再把规划结果接到具有明确使能条件的差速控制器。

## 本次实机验证记录

2026-11-09 完成最后一轮 12 秒订阅采样（短时统计，不等于长期压力测试）：

| 输出 | 订阅端频率 | 平均采集到接收延迟 |
| --- | --- | --- |
| 规范化深度/点云 | 约 7Hz | 约 130～148ms |
| ESDF/局部栅格 | 约 7Hz | 约 201～205ms |
| ORB VO | 约 2.2Hz | 约 452ms |
| IMU | 约 200Hz 量级 | 约 31ms |

StereoNet 自身日志约 10Hz、BPU 占用约 38～42%；完整 Python 链路频率低于 BPU 本身频率。最后 ORB 状态 `TRACKING_VO_ONLY`，800 特征、742 内点、744 有效深度匹配，VO 到光学帧 TF 可查询。规划状态为等待目标，未触发实车运动。未知和近场盲区仍可能使真实场景没有可行预览路径。

局部地图曾因队列积压出现约 1.2 秒延迟，本版通过深度 1 的最新点云订阅与数组读取，将该轮平均延迟降至约 0.2 秒。已通过构建、ORB/ESDF 数值检查、隔离 ROS 规划检查与原有差速模型/编码器检查。实车已知距离运动精度、长时间漂移和 IMU 融合未验收；没有以合成数据结果代替实车精度结论。
