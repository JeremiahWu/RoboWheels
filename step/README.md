# 本地机械模型源文件

将原始装配模型放在 `step/robot.STEP`。原始 CAD 不进入 Git，也不进入 Docker 构建上下文；请自行保留备份。

当前转换脚本所对应的 SHA-256：

```text
3312c97f8f197c85c103d2350e4d5f492de02710f13549223610624c20c9dcea
```

运行网页、Gazebo 和 RViz 不需要原始 STEP：仓库保留了描述包中的 URDF/SDF 和 STL 网格。

只有重建机械模型时才需要 STEP，步骤见 [仿真说明](../docs/simulation/README.md) 和 [ros2_control 说明](../docs/simulation/ROS2_CONTROL.md)。STEP 更换后需要重新检查零件编号与关节映射，不能直接沿用旧配置。
