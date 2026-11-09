# 工具索引

- `cad/`：STEP导出、几何校验辅助与八关节实体显示模型生成。
- `hardware/base/`：电机、编码器、引脚与圈数标定；部分命令会驱动车轮。
- `hardware/arms/`：德晟总线只读扫描、UART回环；扫描前停止常驻串口桥。
- `hardware/cameras/`：Astra IR安装/兼容排查。
- `validation/`：既有回归检查，运行前阅读文件头。`validate_control*`需要虚拟ROS环境，
  `validate_joy`、`validate_motor_teleop`、`validate_x5_wiring`等需要对应实体环境。

纯软件保护测试位于 `tests/`。常用离线命令见 [开发说明](../docs/DEVELOPMENT.md)。
旧 `tools/*.py` 路径已按上述目录迁移；新的运行实现应进入ROS包对应子系统，
不要把工具脚本当作运行库导入。
