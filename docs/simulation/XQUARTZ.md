# XQuartz + VirtualGL 原生 RViz2

在 Mac 上直接显示 Docker 内的 RViz2 窗口，并在窗口内通过 `RoboWheels Control` 面板控制双臂和差速底盘。原浏览器/noVNC 模式保留。

## 启动

先安装并打开 [XQuartz](https://www.xquartz.org)，同时启动 Docker Desktop。在仓库根目录执行：

```bash
# 首次构建；依次准备基础、控制及 XQuartz 镜像
python3 scripts/xquartz.py build

# 已有镜像时直接启动
python3 scripts/xquartz.py start
```

RViz2 会作为 Mac 上的 XQuartz 窗口出现。双臂按钮复用容器内的本地轨迹桥接，由它调用真实 `FollowJointTrajectory` action；底盘指令、关节反馈和里程计使用 ROS 话题。窗口内左侧 `RoboWheels Control` 面板包含：

- **Forward / Reverse / Left / Right**：按住驾驶差速底盘，松开停止；`STOP` 停车。窗口失焦也会发送停车指令，控制器另有 0.5 秒超时停车。
- **Speed**：底盘线速度，范围 0.05–0.25 m/s。
- **R / L 关节目标**：左右臂各 5 个关节，单位 rad；旁边显示实际虚拟关节反馈。
- **Execute targets**：按 Duration 执行当前目标。
- **Demo trajectory**：执行三段、9 秒的双臂演示。
- **Home (CAD zero)**：双臂回到 STEP 装配姿态，底盘位置不重置。
- **Cancel / hold**：取消双臂轨迹并保持当前位置。

如果面板高度不足，可以在面板内滚动。模型视角仍由 RViz 的鼠标交互控制，绿色路径表示底盘行驶轨迹。

```bash
python3 scripts/xquartz.py status
docker compose -f compose.xquartz.yaml logs -f control
python3 scripts/xquartz.py stop
```

停止只管理本项目的控制容器及本项目的本机转发进程，不退出 XQuartz。

## 显示链路

```text
Docker Linux
  controller_manager → 双臂轨迹控制器 / 差速控制器
           ↓ 关节反馈、里程计、TF
  RViz2 + 原生控制面板
           ↓ VirtualGL 3.1.5
  Xvfb :1 / Mesa llvmpipe 进行 OpenGL 渲染
           ↓ X11 图像传输（-c proxy）
Mac 127.0.0.1:6099 → XQuartz UNIX socket → 原生窗口
```

该方案使用 CPU 软件渲染，并未将 Apple GPU 传入 Docker。已在本机确认 Mesa llvmpipe、OpenGL 4.5；VirtualGL 的作用是把容器内渲染结果传给 XQuartz，绕过直接使用 XQuartz 间接 GLX 渲染现代 OpenGL 的限制。

RViz 使用 `/opt/VirtualGL/bin/vglrun -nodl -d :1 -c proxy` 启动。`-c proxy` 使用 X11 图像传输，因此不需要额外安装 Mac VirtualGL 接收客户端；`-nodl` 避免不必要的 dlopen 拦截。[VirtualGL 参数说明](https://github.com/VirtualGL/virtualgl/blob/main/server/vglrun.in)

镜像从官方发布源安装固定版本 VirtualGL 3.1.5，并校验 amd64/arm64 安装包的 SHA-256。RViz 插件源码位于 `ros2_ws/src/robowheels_rviz_plugins/`。

## XQuartz 连接与认证

启动脚本使用当前 XQuartz UNIX socket，不修改 `xhost` 授权规则，也不修改 XQuartz 的 TCP 或间接 GLX 设置。本机已有实际 `/tmp/.X11-unix/X0` 时优先使用它，避免 launchd 显示入口失效时连接阻塞。

只将当前显示的认证 cookie 复制到 `.local/x11/Xauthority`，文件权限 0600、目录权限 0700，再只读挂载到控制容器。转发端口 6099 仅监听 `127.0.0.1`，不在局域网开放。容器访问 `host.docker.internal:99`，脚本会将认证记录的显示号对应到 :99。

`.local/x11/` 中的 cookie、PID 和日志已经由 Git/Docker 忽略，不应提交。不要将 cookie 内容粘贴到日志或问题报告中。

## 模式切换

原生模式与浏览器模式复用同一套控制器、ROS_DOMAIN_ID=43 和本项目 `control` 服务，切换会重建该控制容器，不应当把两套控制环境同时启动在同一个 ROS 域。

切回浏览器模式：

```bash
python3 scripts/xquartz.py stop
docker compose -f compose.control.yaml up -d
```

随后打开 [浏览器控制面板](http://localhost:18081)。原生模式不启动 noVNC，所以该页面的浏览器桌面 iframe 不是原生模式的查看入口；原生窗口中的控制面板不依赖浏览器即可操作。

## 检查与排查

```bash
# 查看真实的容器渲染器（不会打印认证内容）
docker compose -f compose.xquartz.yaml exec control cat /tmp/virtualgl-renderer.log

# 控制器应为 active
docker compose -f compose.xquartz.yaml exec control bash -lc 'source /ws/install/setup.bash; ros2 control list_controllers'

# 原生面板组件集成检查：在隐藏 Xvfb 中测试，不点击 Mac 上的窗口
# 会向运行中的虚拟硬件提交小幅轨迹和底盘运动
docker compose -f compose.xquartz.yaml exec control bash -lc 'source /ws/install/setup.bash; DISPLAY=:1 ros2 run robowheels_rviz_plugins control_panel_smoke_test'
```

若提示缺少 cookie 或 socket，先在 Mac 上打开 XQuartz 后重试 `prepare` / `start`；若 XQuartz 重启，重新执行 `start` 刷新认证文件。连接查询带有超时，失败原因会出现在 Compose 日志中。

机器人仍使用虚拟硬件：关节限位、零位和质量参数尚未完成实车标定，不包含负载动力学或碰撞规划。模型与控制接口细节见 [ros2_control 说明](ROS2_CONTROL.md)。
