# RoboWheels 硬件实测记录

> 所有结论来自 2026-08-26 在 Orange Pi 3 LTS 上的实测验证，不是猜测。
> 接线表见项目需求文档，本文件只记录**实测确认/修正**的部分。

## 系统环境（已验证）

- Orange Pi 3 LTS, kernel 5.16.17-sun50iw6, Ubuntu 22.04
- ROS2 Humble, `/opt/ros/humble`
- wiringOP gpio v2.46 (`/usr/local/bin/gpio`)
- OPi.GPIO 0.5.2 (SUNXI 模式，寄存器直访，可用)
- Python 3.10.12（注意：未装 smbus2/smbus/gpiod，装 I2C 驱动时需 pip install smbus2）

## 电机实测映射（2026-08-26 修正）

原始接线表标注 A=左、B=右，**实测相反**：

| 通道 | 引脚 | 实测驱动 | 前进方向 |
|---|---|---|---|
| A | AIN1=PL10(wPi8), AIN2=PH5(wPi11) | **右轮** | AIN2(PH5)=PWM, AIN1=LOW |
| B | BIN1=PH4(wPi14), BIN2=PH6(wPi12) | **左轮** | BIN2(PH6)=PWM, BIN1=LOW |

- 后退 = IN1 PWM, IN2 LOW（与前进相反）
- AT8236 无 STBY 引脚，IN1/IN2 全 LOW = 滑行
- 软件 PWM 200Hz 实测可用；低速测试 20% 占空比两轮均正常转动
- 整车双轮 IN2 PWM → 小车**前进**（已实测确认）

## 编码器（2026-08-26 实测）

- **左轮编码器 = RA/RB (PL2/PL3)**，右轮编码器 = LA/LB (PD24/PD23)（与接线表交叉，实测确认）
- 前进时计数正向递增 ✅（drive 实测）
- 4x 正交解码；OPi.GPIO 轮询采样实测可用
- 标定（直线跑 ×2，卷尺实测，均值）：
  - 右轮 1656 ticks/m，左轮 1679 ticks/m（4x）
  - ticks/rev (4x) ≈ 334（轮周长 0.20m），≈83 沿/圈，符合 11PPR×~7.4:1 电机
  - wheel_radius = 0.0318 m
- 注意：低速 PWM 下转速与占空比非线性（12%~20% 占空比都约 0.5 m/s），差速控制需实测标定；两轮速度不完全对称（跑偏 ~20° 出现过一次，疑打滑）
- 标定留待 odom 步复核

## I2C / MPU6050（已验证）

- i2c-0 总线（PD26=SDA.0 / PD25=SCL.0，ALT2 已激活）
- MPU6050 @ 0x68，WHO_AM_I=0x68 ✅，上电默认 PWR_MGMT_1=0x40（睡眠态，驱动需唤醒）

## LiDAR COIN-D4（已验证）

- USB-TTL: CH340 (1a86:7523) → /dev/ttyUSB0
- udev 规则已装：/etc/udev/rules.d/sc_mini.rules → /dev/sc_mini → ttyUSB0
  （规则里还有 ttyS0→sc_mini 一条，板载串口不用，但当前 USB 占用符号链接，不影响）
- 驱动：drivers/cspc_lidar_sdk_ros2（已 colcon build）
  - baudrate 230400, samp_rate 9, 版本 M1CT_TOF
  - 启动：`ros2 launch cspc_lidar lidar_launch.py`
  - /scan @ 10Hz，每圈 345~401 点，实测数据正常
  - frame_id 当前为 laser_link（后续 TF 需统一为 laser）
- 注意：节点是 LifecycleNode，launch 后需 configure/activate（实测 launch 即出数据）

## ROS2 图（实测时）

- topics: /scan, /point_cloud, /lidar_status, /lsd_error
- 节点名 cspc_lidar；曾出现重名警告，疑似 ros2 daemon 缓存，必要时 `ros2 daemon stop`
