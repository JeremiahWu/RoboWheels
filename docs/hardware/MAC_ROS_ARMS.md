# Mac USB 舵机与 ROS 2 / RViz 同步

本模式让 RViz 中的 STEP 机器人模型跟随真实舵机的位置反馈。
Mac 原生 Python 独占 USB 串口；Docker 运行 ROS 2 Humble、robot_state_publisher、
RViz 和 noVNC，不运行虚拟 ros2_control 控制器或 Gazebo 动力学。

## 本机配置

本机已有配置在 `.local/config/arms/`，目录重构时已迁移，原零位保持不变。
新机器先建立隔离环境，再从模板创建本机配置：

```bash
python3 -m venv .local/mac-hardware
.local/mac-hardware/bin/pip install -r requirements/mac-hardware.txt
mkdir -p .local/config/arms
cp ros2_ws/src/robowheels/config/arms/servo_bridge.example.json .local/config/arms/servo_bridge.json
cp ros2_ws/src/robowheels/config/arms/servo_zero_offsets.example.json .local/config/arms/servo_zero_offsets.json
```

将端口改为实际 `/dev/cu.*`，逐关节读取并填写软件零位。零位模板为null，
未校准时桥接会拒绝启动，不会默认为0或自动回零。不要覆盖已有本机配置。
首次构建依赖现有 `robowheels:xquartz-humble` 基础镜像，构建流程见
[显示环境文档](../simulation/XQUARTZ.md)。

## 打开和使用

首次/修改插件后启动：

```bash
python3 scripts/mac_physical_arms.py start --build
```

已构建镜像后启动：

```bash
python3 scripts/mac_physical_arms.py start
```

浏览器打开 http://localhost:16082/vnc.html?autoconnect=1&resize=scale 。
这是实体模式入口；16080 仍是原来的虚拟控制环境。

RViz 左侧 **Physical arms** 面板显示每个舵机的原始位置及相对软件零位的偏移。
启动默认为只读；点击 **Enable physical outputs** 后，点击某关节的 **-20 / +20**
即可发送一次约20原始单位的目标变化，时长2.5秒。每次只允许一个关节运动，
完成后才能再次点击；没有自动回位或批量示例动作。
**Stop / disarm** 禁止后续指令，并在当前活动关节反馈可信时向其发当前位置保持目标，
不会关闭扭矩。它不是切断电源的硬件急停。

ID3（右肘）此前出现异常大幅转动；用户要求复测后，在3396附近完成20单位往返，
未再出现跳变，现开放小步按钮；按用户后续要求，当前与其他关节使用相同限制：每步最多40、距软件零位最多120原始单位。
历史异常原因仍未完全确认，不进行大范围轨迹或自动回零。
所有关节每步不超过40原始单位，目标限制在软件零位±120单位及设备寄存器限位内。
这些只是调试保护范围，不是机械安全限位，也没有碰撞检测。

查看状态/停止：

```bash
python3 scripts/mac_physical_arms.py status
python3 scripts/mac_physical_arms.py stop
```

Mac 串口桥独占设备，运行时不要再启动单独的串口扫描工具。
重新插拔后检查 `/dev/cu.*`，必要时修改 `servo_bridge.json` 中的端口。
配置或零位更新后需停止再启动桥接，让新参数生效。

## 用户确认的映射

| 实体关节 | 右臂 ID / ROS关节 | 左臂 ID / ROS关节 |
|---|---|---|
| 第1，肩前后摆 | 1 / right_shoulder_pitch | 5 / left_shoulder_pitch |
| 第2，肩侧摆 | 2 / right_shoulder_roll | 6 / left_shoulder_roll |
| 第3，肘 | 3 / right_elbow_pitch | 7 / left_elbow_pitch |
| 第4，腕 | 4 / right_wrist_pitch | 8 / left_wrist_pitch |

用户要求不用仿真中每臂最后的腕旋转自由度，因此 physical 模型将 wrist_roll
固定为0，保留对应外观网格。原仿真URDF和虚拟控制器配置不变。

软件零位来自 `.local/config/arms/servo_zero_offsets.json`，
参考姿态为用户确认的“双臂放下”。桥接配置为同目录 `servo_bridge.json`。
每个舵机依实际反馈计算 `relative_raw = current_raw - zero_raw`，
ROS位置暂用 `relative_raw × sign × radians_per_raw`。

**角度比例、正方向和模型零姿态仍为暂定值。**
比例暂用300°/4096，sign暂为+1，未进行逐轴物理标定。
实体模式将CAD肩前后摆参考旋转约90°，使软件零位时模型双臂近似放下；
这是显示参考，不是经过测量的装配标定。
因此当前完成的是反馈驱动显示和小步控件联动，不能作为准确角度控制、逆运动学、
自动回零、运动规划或碰撞判断依据。完整动作方向和幅度仍需逐关节观察校准。

## 数据链路与隔离

1. Mac `robowheels.arms.mac_bridge` 用115200/8N1读取德晟位置寄存器，验证应答和校验。
2. 本地HTTP端口18765，绑定127.0.0.1，使用 `.local/physical-arms/token` 鉴权。
3. Docker `robowheels.arms.ros_shadow` 读取反馈，在ROS域44发布 `/joint_states`。
   Docker只用这些真实关节反馈生成TF，断链时停止发布新状态。
4. RViz原生插件 PhysicalArmPanel 通过容器内回环API向Mac提交受限单关节步进。
5. noVNC只映射本机16082端口。没有指令从旧虚拟控制器转发到实体舵机。

串口读取失败、过期、位置突跳或跟随误差过大时关闭输出；故障保留到重启，
需先核查实际状态再重启。API不提供改ID、扭矩开关、限位写入或任意轨迹入口。
底盘仅静态显示，不连接实体电机；相机图像当前仍在原先域43的环境中。

## 验证记录（2026-11-09）

- 9项使用假串口的保护测试通过：未使能、禁用关节锁定、步长、类型、过期/故障、
  零位范围、扭矩关闭、单关节互斥和停止保持、未标定模板拒绝启动，以及8关节模型结构。
- 镜像内C++插件构建成功，noVNC中控件和模型已检查。
- ID1–8均有新鲜真实反馈；域44的 `/joint_states` 仅一个发布者
  `physical_arm_feedback`，无mock关节状态。
- 通过真实RViz面板对ID8执行+20、-20，原始反馈从2086附近变为2098附近再回2086附近，
  ROS `left_wrist_pitch` 相应变化，控件输出在验证后已关闭。

复现纯软件测试（不会打开真实串口）：

```bash
.local/mac-hardware/bin/python tests/test_physical_arm_bridge.py
```

ID3已完成小步往返复测；当前使用与其他关节相同的调试范围，历史异常仍需在正式标定时复核。
