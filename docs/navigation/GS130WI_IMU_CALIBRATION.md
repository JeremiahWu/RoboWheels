# GS130WI 相机—IMU 外参与时间偏移标定

本流程为 RoboWheels 的后续操作指南；本次没有标定板和人工运动采集，尚未生成有效标定结果。当前运行的是 ORB + BPU 双目深度视觉里程计，`imu_fused=false`。不要把它称为已完成 VIO。

## 需要得到什么

- 双目内参、畸变和左右目外参：`camchain.yaml`。
- 相机相对 IMU 的刚性变换与时间偏移：`camchain-imucam.yaml`。
- 估计报告 PDF、数值结果与原始录包，用于复核。
- 最终相机到机器人底盘的安装外参单独测量；这项不属于相机—IMU 标定。

GS130WI 的 IMU 与双目应作为同一个刚性模块运动。相机倒装属于实际安装关系，标定输入保持传感器原方向；不要使用 Mac 上为观看而旋转 180° 的 `/rdk_debug/*/upright` 图像。两根同步/信号已接好不等于时间偏移一定为零。

## 1. 固定运行时的图像几何

本次 BPU 链路使用 544×448、GDC 开启、EXT 与 LPWM 同步。建议对同一条几何处理链标定：GDC、旋转角、裁剪、分辨率一旦改变，应重新验证内参及虚拟相机坐标。不要混用未校正镜头和 GDC 图像的标定参数。

在 X5 停止 `navigation_bpu.launch.py` 后，单独启动传感器。不要同时启动两个 MIPI 驱动：

```bash
cd /root/RoboWheels
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robowheels gs130.launch.py \
  mipi_image_width:=544 mipi_image_height:=448 \
  mipi_image_framerate:=20.0 mipi_gdc_enable:=true mipi_lpwm_enable:=true
```

确认 `/cam0/image_raw`、`/cam1/image_raw` 为 mono8 544×448；确认 IMU 时间戳单调、角速度为 rad/s、加速度为 m/s²，静止加速度模长接近 9.81。用 `ros2 topic hz /imu0` 测真实频率；不要直接填手册或旧日志中的频率。

相机 20Hz 是建议采集设置，需先确认驱动实际稳定出图；不足时降低频率并记录实际值。不要为了追求频率而用接收时间替代采集时间戳。

## 2. 准备标定板和输入文件

推荐打印 AprilGrid，贴到平整硬板。可在后面的 Kalibr 容器中生成图案：

```bash
kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.025 --tspace 0.3
```

打印设为实际尺寸，量出实际标签边长与间距后填写 `aprilgrid.yaml`。以下仅对应标签边长 25mm、间隙 7.5mm 的板，不能直接用于尺寸不同的打印品：

```yaml
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.025
tagSpacing: 0.3
```

IMU 配置 `imu.yaml` 需要 `accelerometer_noise_density`、`accelerometer_random_walk`、`gyroscope_noise_density`、`gyroscope_random_walk`、`rostopic: /imu0` 及实测 `update_rate`。四个噪声参数必须由对应量程/采样配置的器件资料或静止数据 Allan 分析获得，不能抄 RealSense 的值。建议另录一段充分预热后的长时间静止 IMU 数据，按 [Kalibr IMU 噪声模型](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)及其链接的 Allan 工具估计。没有可信噪声参数时，可以采集数据，但先不要把外参结果验收为最终值。

## 3. 在 X5 录制原始图像和 IMU

在 `/root/RoboWheels/.local/calibration` 内保存数据，该目录不提交 Git。若录包对传感器 QoS 不匹配，可创建 `qos.yaml`：

```yaml
/cam0/image_raw:
  reliability: best_effort
  durability: volatile
/cam1/image_raw:
  reliability: best_effort
  durability: volatile
/imu0:
  reliability: best_effort
  durability: volatile
```

另开已 source 的终端：

```bash
mkdir -p /root/RoboWheels/.local/calibration
cd /root/RoboWheels/.local/calibration
ros2 bag record --storage sqlite3 --qos-profile-overrides-path qos.yaml \
  -o cam_imu /cam0/image_raw /cam1/image_raw /imu0 \
  /cam0/camera_info /cam1/camera_info
```

固定标定板，缓慢移动整个相机—IMU 模块，使两目都能识别板。让板覆盖画面不同位置，并包含三个方向的平移与旋转，避免只平面推车或只做偏航。录制几分钟清晰、连续的数据，开始和结束各保留静止片段。可以将一体相机模块从底盘取下手持，保持镜头与板载 IMU 的内部相对位置不变；重新装回后再核对到 base_link 的安装外参。

按 Ctrl+C 正常结束后执行 `ros2 bag info cam_imu`，确认三路都有足够消息。不要在录包期间断电。把整个录包目录拷到 Mac：

```bash
scp -r root@192.168.124.24:/root/RoboWheels/.local/calibration \
  /Users/wuruihao/GitHub/机器人/RoboWheels/.local/
```

## 4. 在 Mac Docker 中准备 Kalibr

上游 Kalibr 主要使用 ROS 1；ROS 2 的 db3 目录不能直接作为其 `--bag` 输入。先转换，保留原始文件。可在具备兼容 Python 的独立环境中安装 `rosbags`，按 [官方转换说明](https://ternaris.gitlab.io/rosbags/topics/convert.html)执行：

```bash
rosbags-convert --src cam_imu --dst cam_imu.bag
```

转换后复核图像、IMU 消息数量及消息内 header 时间戳没有改变。转换 bag 格式不会自动修复时间同步。

Kalibr 使用 [上游 Dockerfile](https://github.com/ethz-asl/kalibr/blob/master/Dockerfile_ros1_20_04)。以下在 Mac 仓库根目录执行，依赖下载与构建可能较久；本次未执行该构建：

```bash
git clone https://github.com/ethz-asl/kalibr.git .local/kalibr-src
docker build --platform linux/amd64 -t robowheels:kalibr \
  -f .local/kalibr-src/Dockerfile_ros1_20_04 .local/kalibr-src
docker run --rm -it --platform linux/amd64 \
  -v "$PWD/.local/calibration:/data" robowheels:kalibr
```

Apple Silicon 使用 amd64 模拟通常较慢；这是离线标定，不在 X5 控制环中运行。容器内：

```bash
source /catkin_ws/devel/setup.bash
cd /data
```

## 5. 先内参，再相机—IMU 联合标定

先对当前实际输入图像估计双目参数。GDC 图像可先采用 pinhole-radtan 模型，再检查拟合残差；未经过 GDC 的原始镜头需另外根据镜头特性选择模型，不要一律当鱼眼：

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag cam_imu.bag --target aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw --dont-show-report
```

检查生成的 `*-camchain.yaml`、报告及左右目基线是否符合实物。将实际输出文件名作为后续 `--cam`；不要把 `camchain.yaml` 当成固定必然的文件名。然后运行：

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag cam_imu.bag --target aprilgrid.yaml \
  --cam cam_imu-camchain.yaml --imu imu.yaml --dont-show-report
```

这里的 `cam_imu-camchain.yaml` 是常见输出名示例，以实际文件为准。时间偏移默认参与估计，不要加 `--no-time-calibration`。查看 `*-camchain-imucam.yaml`、`*-results-imucam.txt` 和 `*-report-imucam.pdf`。入口及参数定义以 [Kalibr 标定说明](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)为准。

## 6. 检查结果，再接 hobot_vio

至少用两段独立运动数据重复标定，检查旋转、平移和时间偏移是否稳定；对照相机模组物理布局、报告残差与重力方向。留出一段没有用于拟合的运动数据测试轨迹，不能只凭优化器退出成功验收。

[Kalibr YAML 定义](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)中：

- `T_cam_imu` 将 IMU 坐标变到相机坐标；hobot_vio 配置中的 `T_CtoI` 方向相反，需要求逆，不能直接复制。
- `timeshift_cam_imu` 满足 `t_imu = t_cam + shift`。必须按后端时间约定应用一次，避免反号或重复补偿。

当前 `vio.launch.py` 只提供后端启动入口，尚未实现 Kalibr 文件自动转换和时间偏移补偿。后续应生成符合 hobot_vio OpenCV YAML 格式的内参、畸变、`T_CtoI` 与噪声配置，并在输入适配层明确实施时间补偿；不能仅填写 `calibration_verified:=true` 就视为接入完成。

hobot_vio 的默认 RealSense 文件不可使用。其实际输出是 `/horizon_vio/horizon_vio_path`，不是标准 Odometry；需要额外检查输出状态与坐标约定。它与 ORB VO 是两种定位后端，不要同时争用同一条 TF。当前 ORB 不含 IMU 预积分、偏置估计、回环或重定位，后续选择保留 ORB 前端时，还需要真正的惯性融合后端。

## 保存与复现

原始录包、转换包、长时间 IMU 数据、报告和标定软件版本放 `.local/calibration`。经验证的配置可放到受版本管理的配置目录，并记录序列号、图像尺寸、GDC/旋转/采样设置、时间戳类型和标定日期。改变这些设置后重新验证。
