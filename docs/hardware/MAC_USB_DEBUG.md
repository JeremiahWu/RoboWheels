# Mac USB 硬件调试

本页保留当前可复现步骤。逐次扫描、改号和运动的完整调试记录保存在本机
`.local/notes/MAC_USB_DEBUG.full.md`，不随仓库提交。

## 舵机与调试板

实体双臂每臂4个德晟 DS-SY15A，总共8个。YB-SD15M两个舵机接口是同一总线的并联接口，
右臂ID1–4、左臂ID5–8，不能重复。具体转轴映射和RViz操作见 [ROS 双臂控制](MAC_ROS_ARMS.md)。

当前使用调试板 **DC独立供电 + microUSB连接Mac**。microUSB与外接TTL二选一，
DC与供电端子二选一。TTL接口5V是输入；供电按实际舵机规格选择。
不要直接运行厂商会广播改ID、循环运动的示例。

协议使用115200、8N1，请求头FF FF、应答头FF F5，PING=01、READ=02，
位置地址0x38、长度2、大端。一个验证过的ID1读位置请求为 `FF FF 01 04 02 38 02 BE`。

```bash
# 先停止占用串口的桥接，再使用只读工具；端口按当前 /dev/cu.* 修改。
python3 scripts/mac_physical_arms.py stop
.local/mac-hardware/bin/python tools/hardware/arms/desheng_servo_read.py /dev/cu.usbserial-REPLACE --last-id 250
```

该工具仅查询ID和位置，不修改ID、扭矩、目标位置或EEPROM。
改ID时应隔离目标舵机；软件零位是本机数据，不写入舵机硬件。
ID3曾有异常大幅运动，后续稳定姿态的小幅往返复测通过，不能据此认定角度比例或机械限位已标定。

## Astra Pro RGB → Docker ROS

Mac已识别Astra RGB（USB 2bc5:0501）和深度设备（2bc5:0403）。RGB原始采集
1920×1080成功，下列调试桥按640×480 BGR发布，实测约6 fps。深度尚未验证。
AVFoundation相机索引可能随插拔变化，请确认选择的是Astra而非内置摄像头。

```bash
python3 -m venv .local/mac-hardware
.local/mac-hardware/bin/pip install -r requirements/mac-hardware.txt
docker compose -f compose.control.yaml up -d
docker cp ros2_ws/src/robowheels/robowheels/sensors/host_image_receiver.py robowheels-control-1:/tmp/host_image_receiver.py
.local/mac-hardware/bin/python -u ros2_ws/src/robowheels/robowheels/sensors/mac_astra_capture.py --index 2 | \
  docker exec -i robowheels-control-1 bash -lc \
  'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=43; python3 -u /tmp/host_image_receiver.py'
```

首次需在macOS允许相机访问。桥接需保持运行，Ctrl-C停止。
话题 `/astra/rgb/image_raw` 尚无标定camera_info，不用于度量处理。
浏览器 http://localhost:16080/vnc.html?autoconnect=1&resize=scale 的RViz可添加Image显示，
或加载 `robowheels/config/rviz/mac_astra.rviz`。此模式的机器人仍为虚拟反馈，
真实双臂反馈在独立ROS域44/端口16082，见上面的双臂控制文档。
