# 明日工作交接（2026-08-27 → 08-28）

> 把这个文件的内容作为明天的开场 prompt 发给 Claude 即可接着干。

## 今天（08-27）已完成

1. ✅ **/cmd_vel 卡点解决**：根因 = OPi.GPIO sysfs 太慢导致 PWM 混叠；
   改用 `robowheels/fast_gpio.py`（/dev/mem mmap 寄存器直访，抄 wiringOP v2.46，
   8 引脚寄存器读=sysfs 读验证过）。热循环 ~15-17kHz，200Hz PWM 占空比实测准确，
   轮子转动实测确认。
2. ✅ 编码器+odom 修复：右轮前进计数为负 → `right_encoder_inverted: true`；
   热循环计数移交 bug 修复（之前 odom 恒 0）。前进时 /odom linear.x>0 ✓
3. ✅ 手柄栈：ros-humble-joy + Xbox 360 (045e:028e)。joy_controller 节点：
   RB(5)=使能死区、A(0)=模式切换、左摇杆驾驶。**上推=前进（invert_linear: false）**。
4. ✅ imu_publisher：MPU6050 @ i2c-0 0x68，30Hz，重力轴自动标定（Y 轴朝下），
   i2c 瞬时错误（Errno 6）自动重连容错。
5. ✅ obstacle_avoid 节点实测（合成雷达注入 + 真雷达）：
   - manual 直通 / 前方斜坡 v×(d-0.15)/(0.5-0.15) 精确命中 /
     侧向势场转向 ±1.5 rad/s 封顶 / 雷达超时停车 / 倾角>20° 停车（用户抬车实测）
6. ✅ 一键启动 `tools/start_stack.sh`（source 三个 workspace + 全栈 launch）。

## 当前系统架构

```
Xbox360 → joy_node(/joy) → joy_controller(/cmd_vel_joy, /robowheels/mode)
        → obstacle_avoid(/cmd_vel) → motor_driver(AT8236 PWM + 编码器/odom)
MPU6050 → imu_publisher(/imu/data_raw) ──┐
COIN-D4 → cspc_lidar(/scan 10Hz) ────────┴→ obstacle_avoid（避障模式用）
```

- 所有参数在 `ros2_ws/src/robowheels/config/{motor,control}.yaml`，不许写死。
- 调试工具：tools/pin_sampler.py（寄存器只读采样）、tools/analyze_pin.py（PWM 信号形状）。

## 明天任务（用户 08-27 明确）

### 1. 开发"带上编码器"的整个系统
- 编码器/odom 已通（见上），明天把**轮速**用进 AUTO 模式：
  速度闭环反馈 + 自主转向时监测实际行进速度。

### 2. 手柄热键切换三个模式
用户明确（08-27 修正）：

| 模式 | 摇杆作用 | 行为 |
|---|---|---|
| **手动 (manual)** | 全权（前后+转向） | 直通，无传感器干预（已有） |
| **APAS（辅助避障导航行进）** | 全权（前后+转向） | 现有 avoid 模式：雷达前方减速 + 侧向转向辅助 + IMU 倾角/超时保护（已有，需改名 APAS） |
| **AUTO（自主避障）** | **只有前后**（摇杆绝对位置 = 目标速度） | 车基于**雷达+轮速(编码器)+IMU** 自主避障行进：自行转向、速度闭环 |

### 3. 需要的代码改动（建议方向，明天实测验证）
- joy_controller：模式状态机 2→3 模式，**A 键单键循环切换** manual → apas → auto → manual，
  模式发布沿用 /robowheels/mode（String，值 manual / apas / auto）。
- obstacle_avoid 增加 AUTO 分支：
  - 输入 = 摇杆前后（目标速度 v_target），无转向输入
  - 转向策略候选：雷达扇形势场自主转向 + 前方死区停车 + 直行偏好（前方空旷时回正）
  - 轮速闭环：/odom 的 twist 作为实际速度反馈（如地面阻力大时提 PWM）
  - IMU：倾角保护 + gyro z 防失控（已有）
- AUTO 模式速度语义（用户已确认）：摇杆**绝对位置 = 目标速度**（中位=停，上推=前进目标速度，下推=后退）

## 实测硬件事实（重要，不要改不要猜）

- 详见 HARDWARE.md（今天已更新）：电机映射、编码器交叉、右轮反号、
  FastGPIO、CPU 特征、i2c 容错等全部实测结论。

## 调试命令

```bash
# 一键全栈启动（含雷达）
/root/RoboWheels/tools/start_stack.sh            # 跳过雷达：start_stack.sh nolidar

# 发命令（注意：CLI 发现要 2~3s，短 pub 会丢失；--once 不可靠，用 -r 重复发）
source /opt/ros/humble/setup.bash
ros2 topic pub -r 10 /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic pub -r 5 /robowheels/mode std_msgs/msg/String "{data: apas}"

# 观察
ros2 topic echo /cmd_vel --field linear.x
ros2 topic echo /odom --field twist.twist
```

## 安全红线（每步都要遵守）

- 启动时电机引脚必须先 OUTPUT LOW；节点退出→引脚置 LOW；禁止 GPIO.cleanup()
- /cmd_vel 500ms 无更新 → 停车（motor_driver watchdog 已实现，最终兜底）
- 手柄断开/RB 松开 → 停车；LiDAR/IMU 超时 → 停车（避障类模式）
- 测试用低速（max_pwm 当前 40%）
- 所有参数放 config/*.yaml，不要写死

## 已知坑（今天踩过）

- CycloneDDS：CLI 发布者发现 2~3s；`--once` 在发现前发出会丢消息
- rclpy 每 executor 唤醒 ~3-4ms 固有 CPU 开销（本板实测，非代码问题）；
  节点合并可减总开销
- i2c 总线会瞬时 Errno 6（杜邦线），imu 节点已容错
- `pkill -f` 会误杀包装 shell（cmdline 含同样文本）；用 PID 精确 kill

## 待确认问题

- （用户 08-27 已全部确认：AUTO 摇杆绝对位置=目标速度；A 键单键循环切模式）
