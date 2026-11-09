# 开发与提交约定

## 放置新代码

实体运行代码放 `ros2_ws/src/robowheels/robowheels/` 对应子系统，通用代码不依赖Mac绝对路径。
设备协议不依赖ROS，便于离线测试；ROS节点负责话题、时间和坐标适配。
常用启动/停止入口放 `scripts/`，算法和设备实现不要继续堆到该目录。
RViz控件放 `robowheels_rviz_plugins`，仿真专用代码放 `robowheels_sim`。

`tools/hardware/base/`、`arms/`、`cameras/` 存放设备诊断，可能驱动实体。
`tools/validation/` 存放现有回归脚本；阅读文件头确认是离线、ROS集成还是实体运动检查，
不要把整个目录无差别自动执行。`tools/cad/` 存放STEP处理与模型生成工具。
不访问设备的自动化测试放 `tests/`。

ROS依赖使用系统ROS环境；RDK额外Python依赖由根目录 `pyproject.toml` / `uv.lock` 管理。
Mac USB环境独立安装，避免把GPIO和TROS依赖装到Mac：

```bash
python3 -m venv .local/mac-hardware
.local/mac-hardware/bin/pip install -r requirements/mac-hardware.txt
```

## 应提交与不应提交

| 应提交 | 留在本机，不提交 |
|---|---|
| 源代码、launch、通用配置和 `*.example.json` | 串口路径、零位快照、令牌、X11 cookie |
| 运行必需的URDF/SDF/STL、模型报告 | 原始STEP、临时CAD导出、临时渲染 |
| 文档中引用的截图、网页图标 | 调试截图、录屏、逐次通信和运动采样 |
| 前端运行依赖及其许可证 | Python环境、colcon build/install/log、缓存 |
| 可复现步骤及实测结论 | rosbag、数据库、验证输出、进程句柄 |

统一使用 `.local/` 存本机内容：

```text
.local/config/arms/      # servo_bridge.json / servo_zero_offsets.json
.local/config/rdk-host  # 可选，SSH目标，如 root@设备地址
.local/physical-arms/   # 令牌、PID、串口桥日志
.local/mac-hardware/    # Mac Python虚拟环境
.local/notes/           # 历史调试流水与退役工具备份
.local/validation/      # 验证记录
```

`.gitignore`和`.dockerignore`同时排除本机数据；顶层 `calibration/`、`recordings/`、
`reports/` 也作为本机输出目录忽略。不要使用 `git add -f` 提交这些数据。
迁移到其他机器时使用模板重新填写端口和标定，不复制这台机器的零位作为通用默认值。

当前两张仓库图片都被文档或网页引用，保留；STL和前端vendor文件也均为运行资源，
清理时不能仅按扩展名或体积删除。

## 验证

离线几何/导航测试额外需要SciPy，可安装 `requirements/development.txt`；无需连接硬件即可运行：

```bash
python3 tests/test_physical_arm_bridge.py
.local/mac-hardware/bin/python tools/validation/validate_drive_model.py
.local/mac-hardware/bin/python tools/validation/validate_encoder_events.py
.local/mac-hardware/bin/python tools/validation/validate_navigation_grid.py
.local/mac-hardware/bin/python tools/validation/validate_orb_esdf.py
.local/mac-hardware/bin/python tools/validation/validate_control_geometry.py
git diff --check
```

ROS集成检查需进入已构建容器并source工作区。实体运动检查必须按对应硬件文档操作。
更新设备路径或零位后，重启Mac串口桥；不要同时运行扫描器与常驻桥接。

## 文档维护

README提供入口，ARCHITECTURE描述模块边界，硬件文档保留当前接线和实测限制。
逐轮扫描/调试流水放 `.local/notes/`，避免把相互覆盖的临时结论堆进公共文档。
当移动工具或模块时，同时更新Compose挂载、Docker入口、测试导入、文档和ROS安装清单。
