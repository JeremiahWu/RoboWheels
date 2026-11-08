# TinyNav 与 RoboWheels 的接入评估

检查日期：2026-11-08。当前实际链路已更新为 BPU StereoNet + ORB VO + 二维 ESDF，详见 [X5 导航说明](RDK_X5_NAVIGATION.md)。本文保留初次架构评估与 CPU 备用链路说明。参考 [TinyNav](https://github.com/UniflexAI/tinynav)，未复制其源代码或模型，本仓库不包含 TinyNav 安装包。上游代码采用 Apache-2.0；以后引入代码须保留其许可证与相关声明，模型还需分别核对来源与授权。

## 适合借鉴的部分

TinyNav 按 driver、perception、map、planning、control 分层。其 perception 不只是识别：包含双目深度、特征匹配、IMU 预积分和 GTSAM 优化。规划根据局部障碍地图、机器人尺寸及速度约束生成运动候选。这种结构适合 RoboWheels，但不能把现有点云节点称为 VIO。

核对依据：[perception_node.py](https://github.com/UniflexAI/tinynav/blob/main/tinynav/core/perception_node.py)、[planning_node.py](https://github.com/UniflexAI/tinynav/blob/main/tinynav/core/planning_node.py)、[robot_specs.py](https://github.com/UniflexAI/tinynav/blob/main/tinynav/core/robot_specs.py)。

## 平台适配

| 部分 | TinyNav 当前实现 | RoboWheels 处理方式 |
| --- | --- | --- |
| 相机 | 多种驱动与话题适配 | 保留 GS130WI 官方 mipi_cam；输出 mono8 和匹配尺寸的 CameraInfo |
| 深度及特征 | TensorRT 的 StereoEngine、SuperPoint、LightGlue | X5 已接 StereoNet BPU 深度及 CPU ORB；CPU SGBM 为备用，不能加载 TensorRT 引擎 |
| 定位 | GTSAM 与视觉/惯性约束 | 先准备已安装 hobot_vio 的独立启动入口；需完成 GS130WI 标定后验证 |
| 感知/地图 | 局部地图、障碍与距离信息 | 新增相机局部栅格预览；未知区域不作为可通行区域 |
| 规划 | 地图与运动候选 | 新增 ESDF 代价引导的 A* 路径预览；尚无动态障碍处理、曲率约束或轨迹执行 |
| 控制 | 上游机器人参数及控制接口 | 保留 STEP 几何、2496 编码器计数/圈、限速和手柄控制 |

[上游依赖](https://github.com/UniflexAI/tinynav/blob/main/pyproject.toml)包含 CUDA；感知构造函数直接创建 TensorRT 模型。因此现有版本不能原样部署到 X5，也不能通过 Mac Docker 获得 NVIDIA GPU。这里采用其分层思路，保持 ROS 2 Humble 与 X5 原生后端。

## 已整理的代码结构

`ros2_ws/src/robowheels/robowheels/` 下：

- `base/`：电机、硬件 PWM、编码器事件、差速运动学。
- `sensors/`：GS130WI 图像/IMU 适配、Astra RGB。
- `perception/`：双目点云、相机局部障碍栅格。
- `planning/`：独立栅格算法、ROS 路径预览节点。
- `localization/`：定位模块边界；当前 VIO 后端为系统 hobot_vio，不在此重复实现滤波器。

根目录旧 Python 模块保留兼容导入；已有 console_scripts 名称保留。`bringup.launch.py` 改为组合独立 launch，默认只开 GS130WI；电机、手柄、未接的 Astra 分别显式开启。原先手柄操作仍可用 `teleop.launch.py`。

## 运行局部预览

在 X5 的 `/root/RoboWheels` 中 source ROS、TROS 和工作区环境后：

```bash
ros2 launch robowheels navigation_preview.launch.py
```

此命令拥有一个相机驱动实例，并启动图像桥、SGBM、局部栅格和路径预览。如果已有 `gs130.launch.py` 在运行，使用 `start_camera:=false`；如果已有点云节点，先停掉重复节点。不会启动电机或发送 `/cmd_vel`。

| 话题 | 类型/用途 |
| --- | --- |
| `/cam0/image_raw`, `/cam1/image_raw` | mono8；保留传感器图像方向 |
| `/cam0/camera_info`, `/cam1/camera_info` | 缩放至实际输出尺寸，同帧时间戳 |
| `/imu0` | 保留原始测量值；重命名坐标系不代表已完成安装标定 |
| `/stereo/points` | 光学坐标系点云，使用采集时间戳 |
| `/perception/local_grid` | 相机相对 OccupancyGrid；未知=-1，自由=0，障碍/膨胀=100 |
| `/perception/status` | 数据新鲜度与预览状态 |
| `/goal_pose` | 仅接受 `camera_nav` 坐标目标 |
| `/planning/path` | Path，仅预览；无路或数据过期时发布空路径 |
| `/planning/status` | 等待目标、坐标系不匹配、无通路或路径已生成 |

`camera_nav` 只是光学坐标的固定换轴：前=光学 Z，左=-光学 X，上=-光学 Y。原相机倒装仍保留在原始数据中，因此它不代表机器人前方或重力对齐的世界坐标。栅格高度筛选相对相机，不是地面分割。半径 0.18m 仅为预览膨胀参数，不是经机械碰撞验证的整机外形。

RViz 中添加 Map `/perception/local_grid` 与 Path `/planning/path`，用 `camera_nav` 设置目标坐标。Mac 只读遥测转发已加入这些输出话题，未加入目标或运动命令转发。

## VIO 还缺什么

实机已安装 `hobot_vio 2.1.1`，默认配置针对 RealSense D435i。新增入口不使用这份错误的相机配置：

```bash
ros2 launch robowheels vio.launch.py \
  config:=/absolute/path/to/calibrated-gs130wi.yaml \
  calibration_verified:=true
```

配置文件需符合该后端格式，包含实际输入尺寸、畸变模型、`cam_k`、`cam_d`、`T_CtoI`、IMU 噪声与重力参数。还需确认相机/IMU 时间差、量纲和轴向。当前 `gs130_vio.yaml` 与 `stereo_pc.yaml` 中以基线中点估算的静态 TF **不能替代相机-IMU 标定**。此入口只完成后端接线，尚未验证 GS130WI 的 VIO 位姿输出、精度或丢跟踪恢复，不声称已完成 VIO。

下一验收顺序：记录同步双目/IMU → 标定及验证 T_CtoI/时间差 → 静止漂移与闭合路线测试 → 确认相机到 base_link 安装关系（含 180° 倒装）→ 里程计和地图融合 → 差速可行轨迹 → 独立使能的低速实车验证。上游默认速度和足式机器人外形不可直接套用。

## 图像修正

本次实测原始消息 `width=816,height=960,step=816,encoding=nv12,len(data)=1175040`，即完整 960 行 Y 加 480 行 UV。旧实现错误裁为 640 行。现修正为完整 816×960，并将驱动发布的 1088×1280 CameraInfo 缩放到输出尺寸。这是无裁剪 GDC 缩放路径，若更改 GDC/输出布局必须重新核对内参。

官方入口：[GS130WI 快速开始](https://developer.d-robotics.cc/accessories_stereo_camera_doc/stereo_camera_gs130wi/quick_start)。
