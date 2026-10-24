# robot_moveit —— 第 4 步的 MoveIt 2 配置

对应教程：`讲解/课程/10-MoveIt与机械臂执行.html`。只包含「手臂 → 规划 → 控制器 → Gazebo」这一条链，
不含感知、抓取流水线与调度。

## 它解决什么

第 3 步已经有导航，但手臂只有控制器在跑、没人会规划。MoveIt 负责：

- 用 URDF 建模型，用 **SRDF** 补语义（规划组、末端、被动关节、碰撞豁免）；
- 在关节空间里规划（OMPL），并检查自碰撞与场景障碍；
- 把轨迹交给 `FollowJointTrajectory`，也就是第 3 步已经验证过的 JTC action。

装好 MoveIt 不等于接好了。本包只做「接」这一步。

## 规划组（从 URDF 实测推出，不是抄的）

| 组 | 内容 | 端点 |
|---|---|---|
| `arm` | 6 个转动关节链 | base `arm_0_base_link` → tip `arm_0_end_effector_link` |
| `gripper` | 单关节 `arm_0_gripper_right_finger_bottom_joint` | 挂在 `arm_0_end_effector_link` 下 |

3个mimic关节不独立驱动，joint_states只发布主关节；运动学通过URDF mimic关系传播。10-24为仿真插件不支持offset做等价坐标变换，不能据被动关节未发布就断言“MoveIt看不见手指”。

## 怎么跑

```bash
# VM，先确认仿真栈已起（4 个控制器 active）
source ~/.q2q/step3_env.sh
ros2 launch robot_moveit move_group.launch.py
```

`robot_description_file` 默认指向 `~/.q2q/setup/a200_kinova/robot.patched.urdf`（prepare_setup 的生成产物）。
换模型后这个文件会重新生成，不需要改本包。

## executor 接口

`/a200_0000/arm_executor/move`，类型复用 `control_msgs/action/FollowJointTrajectory`（不新建 msg）：

- goal 只有 1 个点 → 当作目标关节位，先调 `plan_kinematic_path` 规划，再发给 JTC；
- goal 有多个点 → 直接透传给 JTC；
- 结果码 `0` 成功 / `1` 规划失败 / `2` 执行失败 / `3` 已取消；
- 取消原样转给 JTC（2026-10-22 实测 11 ms 生效）。

```bash
ros2 launch robot_moveit move_group.launch.py      # 规划
ros2 launch robot_moveit arm_executor.launch.py    # 规划 + 执行 + 取消
python3 scripts/executor_probe.py --target 0.4 0 0 0 0 0          # 到位
python3 scripts/executor_probe.py --target 1.2 0 0 0 0 0 --cancel-after 0.8   # 取消
```

## 已验证 / 未验证

- 10-24 Jackal回归通过：规划执行及实际关节收敛、并发目标拒绝、取消后物理停稳；夹爪两次独立8轮通过。见[修复报告](../../../docs/evaluations/2026-10-24-Jackal修复与回归.md)。
- 抓取采用MoveGroup仅规划、JTC有界执行；完整抓取仍须P5/P6/P7证据。RViz界面状态不能代替物理验收。

## 为什么参数写在 launch 里

2026-10-21 踩过的坑：命名空间被 `RewrittenYaml` 重写后，参数树与节点名对不上，ROS **静默忽略整份 YAML**，
现象是「配了等于没配」。本包把几十行参数直接写进 launch，节点拿到的就是那份 dict。

## 维护

- 改 SRDF 前先 `check_urdf`，确认新链/新关节名；
- 控制器名变化时同步launch中`moveit_simple_controller_manager`映射；
- URDF取生成产物，SRDF唯一源在本包config；不要再复制第二份。
