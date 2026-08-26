# 明日工作交接（2026-08-27）

> 把这个文件的内容作为明天的开场 prompt 发给 Claude 即可接着干。

## 昨天（08-26）已完成

1. ✅ 硬件检查：GPIO / I2C / MPU6050 / USB-TTL / COIN-D4 / ROS2 全部正常
2. ✅ AT8236 双通道正反转 + 整车方向实测（见 HARDWARE.md）
3. ✅ 编码器读取 + 标定（见 HARDWARE.md）
4. ⚠️ 第 4 步差速控制：节点 `motor_driver` 已写好并能收到 /cmd_vel
   （/wheel_speeds 显示 0.15/0.15），**但小车没动，原因未定位**（见下）

## 实测硬件事实（重要，不要改不要猜）

- **A 通道 (PL10/PH5) = 右轮**，B 通道 (PH4/PH6) = 左轮（与原始接线表相反）
- 前进 = IN2 PWM：左轮 PH6、右轮 PH5；后退 = IN1 PWM：左轮 PH4、右轮 PL10
- **左轮编码器 = RA/RB (PL2/PL3)**，右轮编码器 = LA/LB (PD24/PD23)
- 前进时编码器计数正向
- wheel_radius = 0.0318 m（周长 0.20 m）
- wheel_separation = 0.155 m
- 右轮 1656 ticks/m、左轮 1679 ticks/m（4x）；ticks/rev ≈ 334
- MPU6050 @ i2c-0 0x68；雷达 /dev/sc_mini（=ttyUSB0），230400，/scan 10Hz
- 低速 PWM 转速与占空比非线性（12%~20% 都约 0.5 m/s）；两轮速度不完全对称

## 当前卡点：/cmd_vel 车不动

- 节点正常收 /cmd_vel（/wheel_speeds = 0.15, 0.15）
- motor_driver 进程 CPU 100%（PWM 线程在跑）
- `gpio readall` 显示 4 个电机引脚 OUT 且全 LOW（但 readall 采样慢，可能没抓到 12% 占空比的 HIGH 脉冲）
- **已准备好调试工具 tools/pin_sampler.py**（只读高速采样，不干扰）
- 怀疑方向（按优先级）：
  1. PWM 实际在转，只是 12% 占空比 + 电池电量下降 → 用 0.3 m/s (24%) 重试 + pin_sampler 验证
  2. PWM 线程异常没输出 → 在 _pwm_loop 加 try/except + 每秒日志
  3. YAML 未生效（install/ 里是旧拷贝，--symlink-install 对 data_files 不链接）→ 重新 colcon build
  4. 两个节点实例打架 → `ps aux | grep motor_driver` 确认只有一个

## 调试命令

```bash
# 终端 A：启动电机节点
source /opt/ros/humble/setup.bash
source /root/RoboWheels/ros2_ws/install/setup.bash
ros2 launch robowheels motor.launch.py

# 终端 B：发速度（0.3 m/s 前进 6 秒）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# 终端 C：验证 PWM 引脚
python3 /root/RoboWheels/tools/pin_sampler.py PH6 1
python3 /root/RoboWheels/tools/pin_sampler.py PH5 1
```

## 第 4 步通过后，按顺序继续

5. **/odom + TF**：在 motor_driver 的 PWM 紧循环里加编码器边沿计数（4x 解码），
   积分出 odom（odom→base_link TF）。验证：手动推车 1 米看 /odom 读数。
6. **MPU6050**：pip install smbus2，读 i2c-0 0x68，发 /imu/data_raw（sensor_msgs/Imu）
7. **COIN-D4**：ros2 launch cspc_lidar lidar_launch.py（已通，frame_id 改 laser）
8. **键盘**：apt install ros-humble-teleop-twist-keyboard
9. **手柄**：ros-humble-joy + ros-humble-teleop-twist-joy
10. **避障**：安全节点 /cmd_vel → 过滤 → /cmd_vel_safe（阈值放 YAML）
11. **URDF + RViz2**：map→odom→base_link→(left_wheel,right_wheel,laser,imu_link)
12. **slam_toolbox** 建图
13. **Nav2** 导航

## 安全红线（每步都要遵守）

- 启动时电机引脚必须先 OUTPUT LOW
- /cmd_vel 500ms 无更新 → 停车（已实现 watchdog）
- 节点退出 → 引脚置 LOW；**禁止 GPIO.cleanup()**（会把引脚变回输入，AT8236 IN 脚悬空可能误转）
- 测试用低速（max_pwm 当前 40%）
- 电机/编码器/轮距参数全部放 config/motor.yaml，不要写死在代码里
