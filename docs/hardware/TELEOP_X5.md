# RDK X5 手柄控制

2026-11-08 已连接并验证 Xbox 360 兼容 USB 手柄（ZhiXu Controller），设备 `/dev/input/js0`。使用 `joy_linux_node`，不是依赖 SDL 自动重排的轴编号。

| 操作 | 控制 |
|---|---|
| 按住 RB（按钮 5） | 启用驾驶 |
| 左摇杆纵向（轴 1） | 前后；线速度指令最大 ±0.3 m/s |
| 右摇杆横向（轴 3） | 左右；角速度指令最大 ±2.0 rad/s |
| 摇杆回中 / 松开 RB | 停车 |
| 0.5 秒无速度指令 | 控制线程自动停止 PWM |

保留 8% 摇杆死区、20Hz 自动重复；禁用 turbo。电机 PWM 最大 20%，200Hz 硬件 PWM；停车明确禁用四路并归零，不释放成悬空输入。200Hz 可能产生可闻声音，本次未提高 PWM 频率以免改变已验证的低占空比起转行为。

## 已确认的真机对应关系

| 项目 | 结果 |
|---|---|
| A 通道 | 右轮，IN2（BOARD29）PWM 为前进，IN1（BOARD27）为后退 |
| B 通道 | 左轮，IN1（BOARD31）PWM 为前进，IN2（BOARD33）为后退 |
| E1（11/13） | 左轮 / B |
| E2（15/16） | 右轮 / A |

以上由单路短脉冲和用户目视共同确认，不能再把两轮都配置为 IN2 前进。驱动中的正交解码符号与只读原始监控符号相反；`motor.yaml` 的左右 `encoder_sign` 均为 +1 时，整车前进对应正向里程计，`/rdk_debug/encoder_counts` 仍保留原始电气符号。

## 启动

当前真实手柄、速度转换与电机驱动已在 X5 后台运行，不要重复启动。后续重新启动时，先停止已有同类节点，再执行：

```bash
cd /root/RoboWheels
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robowheels teleop.launch.py
```

该入口只启动手柄与底盘，不启动未接外设。Mac RViz 仍使用：

```bash
python3 scripts/rdk_debug.py start
```

浏览器：http://localhost:16081/vnc.html?autoconnect=1&resize=scale

`motor_driver` 接管后已持续发布 `/odom`、`/wheel_speeds`、`/joint_states` 和 `/rdk_debug/encoder_*`。编码器由该驱动统一采集，避免重复采样或发布。

## STEP / MC130 模型更新

已采用 STEP 半径 0.0332m、轮距 0.15631239107433037m 和 MC130 1:48 的 2496 ticks/圈；每米 11965.383673414783 ticks。编码器改用内核 GPIO v2 事件采样。控制模型及地面精度边界见 [BASE_MODEL.md](BASE_MODEL.md)。

## 验证结果与精度边界

- `tools/validate_joy.py` 在隔离话题验证：前/后最大指令 ±0.3、右转为负 angular.z、扳机不触发转向、RB 松开归零。
- 真实手柄预览：左摇杆推前产生正 linear.x，右摇杆推右产生负 angular.z；RB 松开后指令归零。
- `tools/validate_motor_teleop.py` 以 15% 等效占空比、短时间 `/cmd_vel` 指令验证双轮前进/后退和超时停机。前进的 E1/E2 原始增量为 [-5805,-5835]，后退为 [+5127,+6092]；停止发命令后四路 enable 和 duty_cycle 均为零。
- 驱动另外验证了线速度指令钳制、20% PWM 上限和非有限输入触发停车。JointState 的轮角位置/速度已由米、米每秒换算为 rad、rad/s。

**0.3 m/s 是当前线速度指令上限，尚不是经实测标定的车速保证。** 轮径和轮距已使用 STEP，编码器使用 MC130 手册分辨率，当前仍为开环占空比映射、未启用 PID。需要用实际行驶距离复核有效滚动半径和载荷下速度，才能准确验证实际车速和里程计；不能用未经标定的 `/odom` 数值直接认定整车超速或达标。
