# Mac Docker 仿真（ROS 2 Humble + Gazebo Fortress）

新增 **[ros2_control 虚拟硬件与浏览器 RViz](ROS2_CONTROL.md)**：双臂 10 关节位置控制、轨迹 action 执行及反馈验证，入口为 [http://localhost:18081](http://localhost:18081)。下文为原底盘物理仿真模式。

本目录对应 `step/robot.STEP` 的实际转换结果。模型已按底盘、左右轮、支撑轮、立柱、机械臂安装台、左右臂和 Astra 拆分为 9 个网格组，统一转为米，并校正原 CAD 的整体倾斜。

## 启动

在仓库根目录执行，首次构建需要下载 Gazebo 依赖：

```bash
docker compose -f compose.sim.yaml up --build -d
docker compose -f compose.sim.yaml logs -f
```

默认在 Mac Docker 内进行物理计算，浏览器打开 **[http://localhost:18080](http://localhost:18080)** 即可查看三维机器人、实时状态及行驶轨迹，无需登录或安装其他客户端。

网页操作：拖动旋转、滚轮缩放；按住方向按钮或 W/A/S/D 移动，松开停止，空格停车。滑块调整前后移动速度，网页限速 0.25 m/s。页面失去焦点会发送停车指令，原有 0.5 秒超时停车仍然有效。网页通过 `/sim/ground_truth` 显示 Gazebo 车体真实位姿；轮子打滑时，显示位置不会跟随轮式里程计穿过障碍物。所有网页依赖随镜像打包，加载不需要外网。HTTP 端口仅映射到本机回环地址。

Foxglove 仍可连接 `ws://localhost:8765`。添加 3D 面板，固定坐标系选择 `odom`，启用机器人模型（来自 `/robot_description`）和 TF。模型中的相机目前只提供外观，没有模拟图像输出。

当前 Bridge 使用 `foxglove.sdk.v1` 子协议，已验证 Mac 端连接及 STL 资源读取；使用支持该协议的 Foxglove 客户端。自定义 WebSocket 客户端也需提供该子协议。[协议说明](https://github.com/orgs/foxglove/discussions/1131)

控制机器人（指令必须持续发布，停止后 0.5 秒触发停车）：

```bash
docker compose -f compose.sim.yaml exec sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}, angular: {z: 0.0}}"'
```

按 Ctrl-C 停止发布。转弯可改为 `linear.x: 0.0`、`angular.z: 0.5`。输出包括 `/clock`、`/odom`、`/tf`、`/tf_static` 和 `/joint_states`。仿真内部通过 `/sim/cmd_vel` 连接 Gazebo 驱动，外部继续使用 `/cmd_vel`。

运行自动运动检查（会让机器人前进及转向，建议在刚启动的场景执行）：

```bash
docker compose -f compose.sim.yaml cp tools/sim_smoke_test.py sim:/tmp/sim_smoke_test.py
docker compose -f compose.sim.yaml exec sim bash -lc 'source /ws/install/setup.bash; python3 /tmp/sim_smoke_test.py'
```

停止并重置世界：

```bash
docker compose -f compose.sim.yaml down
```

仿真使用独立的 ROS_DOMAIN_ID=42。不启动真机 GPIO、串口和 MIPI 驱动，不需要 privileged 或设备映射，也不修改真机配置。外部算法节点须加入相同 ROS 域并启用 `use_sim_time`；Mac 与 Docker 间的原生 DDS 通信还需单独配置网络，Foxglove 的 WebSocket 显示不依赖这一步。

`gui:=true` 可在有图形显示环境的 Linux 中启用 Gazebo 窗口。Mac Docker 默认未配置 X11/VNC，不能仅凭该参数直接打开 Mac 窗口。

## 模型与精度

| 项目 | 当前实现 |
|---|---|
| 外观 | 从 STEP 导出 STL；原装配有少量退化面，OCCT 导出时跳过，非碰撞网格 |
| 轮距 | CAD 轮轴计算约 0.156312 m |
| 轮胎半径 | CAD 外包络约 0.0332 m；轮子碰撞体为圆柱 |
| 碰撞 | 固定组件使用分组包围盒，支撑轮使用低摩擦球近似 |
| 质量、重心、惯量 | 质量为显式估算，重心及惯量按分组包围盒计算，尚未按真机标定 |
| 机械臂 | 保留 STEP 装配姿态，固定连接；尚不支持关节控制与抓取 |
| 旧设备 | 排除 STEP 中 LD19 雷达和树莓派；未凭空生成 X5 和 GS130 的 CAD |
| 坐标 | x 前、y 左、z 上；前向按 CAD -Z 附近定义，需对照实车确认 |
| 真机差异 | 真机 `motor.yaml` 半径 0.0318 m、轮距 0.155 m，保留原值未改 |

本版用于底盘控制、ROS 接口和碰撞流程开发。不能据此认定真机的轮胎打滑、倾覆边界、电机扭矩或机械臂负载能力。机械臂可动化还需逐关节确认连杆分组、转轴、零位、角度范围和实际质量。

本机 ARM64 Docker 上已通过 ROS 包构建、SDF 语法校验、实际车体前进/稳定性、障碍物碰撞、正向转弯、轮关节反馈、指令限速及超时停车检查。验证脚本保留在 `tools/`，本机运行记录存放于 `.local/validation/`，不提交到 Git。

URDF：`ros2_ws/src/robowheels_description/urdf/robot.urdf`。
SDF：同目录 `robot.sdf`，由 Fortress 的 libsdformat 转换；固定连杆可能被合并，这是正常行为。启动流程从 URDF 加载，不依赖 SDF 的重复维护。

## 重建转换结果

原始 `step/robot.STEP` 仅在本机保存，不提交到 Git，详见 [源模型说明](../../step/README.md)。普通运行使用仓库中的 STL 和 URDF/SDF，不需要 STEP。

工具链固定在 `docker/cad.Dockerfile`，无需在 Mac 全局安装 CAD 库：

```bash
docker build -t robowheels:cad -f docker/cad.Dockerfile .
docker run --rm -v "$PWD:/work" robowheels:cad tools/cad/export_robot.py
```

`tools/cad/model_config.json` 保存 STEP 校验和、轴向、零件分组所依赖的参数及估算质量。STEP 更换后脚本会拒绝沿用旧映射，需先重新检查：

```bash
docker run --rm -v "$PWD:/work" robowheels:cad tools/cad/inspect_step.py step/robot.STEP /work/step/assembly_inventory.json
docker run --rm -v "$PWD:/work" robowheels:cad tools/cad/axes.py step/robot.STEP
```

运行所需的转换报告在描述包的 `config/`，完整零件清单生成在 `.local/cad/assembly_inventory.json`。STL 已是米制，不要再套 0.001 缩放。

更改模型后重建仿真镜像。重新导出 SDF：

```bash
docker compose -f compose.sim.yaml exec -T sim bash -lc 'ign sdf -p /ws/src/robowheels_description/urdf/robot.urdf' > /tmp/robowheels.sdf
python3 -c 'from pathlib import Path; p=Path("/tmp/robowheels.sdf"); s=p.read_text().replace("package://robowheels_description/", "model://robowheels_description/"); Path("ros2_ws/src/robowheels_description/urdf/robot.sdf").write_text(s)'
```

单独使用该 SDF 时，将包含 `robowheels_description` 的 share 目录加入 `IGN_GAZEBO_RESOURCE_PATH`，让 `model://robowheels_description/meshes/...` 可解析。若只使用本项目 launch，无需手动配置。
