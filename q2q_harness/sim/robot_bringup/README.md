# robot_bringup

当前入口：Gazebo Fortress + Jackal J100/Kinova + 四个控制器 + 夹爪动作适配 + 可选 Nav2。命名空间沿用a200_0000。

10-24导航、夹爪、机械臂回归证据与抓取剩余问题见[当前交接](../../../docs/operations/当前开发交接.md)和[修复报告](../../../docs/evaluations/2026-10-24-Jackal修复与回归.md)。

```bash
source ~/.q2q/step3_env.sh
ros2 launch robot_bringup bringup.launch.py gui:=false world:=q2q_warehouse nav:=true
```

Windows 启动、测试、停止及接口说明见 [第 3 步导航运行](../../../docs/operations/第3步导航运行.md)。

导航默认是真值定位基线：`map→odom` 恒等，`odom→base_footprint` 来自 Gazebo 物理真值；TF 话题为 `/a200_0000/tf` 和 `/a200_0000/tf_static`。`/platform/odom` 保留为轮式估计，不能据此认定真实到位。

模型配置唯一源为 `config/robot.yaml`；生成树 `~/.q2q/setup/a200_kinova` 由 prepare_setup 和 model_patches 维护。`nav:=true` 等四个控制器 active 后再启动 Nav2。当前新增轮胎摩擦方向/打滑参数和物理真值插件；旧补丁说明仅解释历史根化设计，不能代替最新运行口径。
