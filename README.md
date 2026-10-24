# Q2Q-FYP — 仓库移动抓取：一条命令走完整条链

> ROS 2 Humble + Gazebo Fortress 里的**移动抓取**仿真：Jackal J100 底盘 + Kinova Gen3 Lite 六轴臂，
> 由中层调度器 `orchestration_node` 用一条 `START` 命令推进 12 个状态，完成
> **导航 → 真值对接 → 锁底盘 → 抓取 → 收臂 → 出站 → 运回 → 投放 → DONE**。
> 全程 10 Hz 状态记录，关键步骤用物理真值判据，另有独立落地门禁复核。

```
IDLE → NAV_TO_PICK → WAIT_STILL → DOCKING → PICK_READY → PICKING
     → PICKED → STOWING → UNDOCKING → NAV_TO_PLACE → PLACE_READY → DONE
```

## 先说边界（比结论更重要）

- 物体是**运动学支撑**（`grasp_mode=suction_approx`，由 `kinematic_carry` 插件按末端位姿 `set_pose` 带着走），
  **不是**摩擦夹持、**不是**真空吸附。物理抓取 P7 仍不成立。
- 完成的是「**中层串行调度 + 低层仿真执行**」。顶层规划、`Subgoal` 消息、Q2Q-VLM、Km-Transformer 尚未实现。
- 4 次独立冷启动通过是**同一个固定场景、同一组参数**的结果，不能写成成功率。
- 取消/失联分支是离线事件表 + 记录回放，不是现场故障注入。投放后没有自动收臂，连续订单未做。

## 目录

| 目录 | 内容 |
|---|---|
| `q2q_harness/sim/` | 仿真层 ROS 包：`robot_bringup`（launch/控制器/工具脚本）、`robot_worlds`、`robot_moveit`、`robot_sim_control`（含运动学支撑插件）、`q2q_interfaces`（msg/action） |
| `q2q_harness/mid/` | 中层：`q2q_orchestration`（状态机、导航/对接/抓取客户端、就绪闸门） |
| `q2q_harness/low/` | 下层：`q2q_low`（抓取执行器 action 服务端、支撑 relay） |
| `q2q_harness/record/` | 记录层：`q2q_record`（`TaskState` → JSONL） |
| `docs/` | 决策（接口定稿）、验收报告、运行手册、证据（`evaluations/artifacts/`） |
| `.dsh/` | 运行入口（PowerShell / shell），见下节 |
| `archive/开发基线/` | 架构设计留档（规划文档，不代表已实现） |

> 教程（`讲解/`）与开发日志（`logs/`）是作者本地的学习与过程记录，**不在本仓库**；
> 少数文档里指向它们的相对链接在仓库内无效，属预期。

## 前置条件

Windows 侧：PowerShell 5.1+、VirtualBox、OpenSSH 客户端。
Ubuntu 22.04 虚拟机（本项目约定）：

| 项 | 约定值 | 说明 |
|---|---|---|
| VM 名 | `UBUNTU22.04` | VirtualBox 里的机器名；`sync.ps1` 用它判断开机 |
| SSH | `ros@127.0.0.1:2222` | NAT 端口转发 2222→22 |
| 私钥 | `%USERPROFILE%\.ssh\id_ed25519` | `ssh-keygen -t ed25519` 后把公钥放进 VM 的 `~/.ssh/authorized_keys` |
| 远程目录 | `~/ros2_ws/src` | 由 `sync.ps1` 从 Windows 推上去的镜像 |

VM 内需要：ROS 2 Humble、Gazebo Fortress 6.18（`ign gazebo`）、`ros_gz`、Nav2、MoveIt 2、
Clearpath 生成器（`clearpath_generator_common` / `_gz`、`clearpath_gz`、`clearpath_viz`）。
`.dsh/q2q_base_install.sh` 是当初装这套依赖的脚本，仅作参考（未在本仓库回归验证）。

**先体检**：

```powershell
Set-Location .\Q2Q-FYP
& .\.dsh\doctor.ps1
```

它逐项检查 VM 是否开机、端口与密钥、ROS/ Gazebo 版本、工作区八个包是否构建，缺什么直接打印修法。

## 三步跑起来

```powershell
# 1) 推源码到 VM 并构建（改过源码才需要；首次必须）
& .\.dsh\step3.ps1 -Action Deploy

# 2) 一键演示：起栈 + 挂 Gazebo 窗口 + RViz + 跑完整链 + 自动截关键帧
& .\.dsh\demo.ps1

# 3) 收尾
& .\.dsh\step3.ps1 -Action Stop
```

`demo.ps1` 退出码 0 才代表整链到 `DONE`（打印最后 14 条状态转移与 `SUMMARY`）。
自动截图落在 `docs/evaluations/artifacts/<日期>/demo-<时刻>/NN_<状态>.png`。

手动分步（调试用）：

```powershell
& .\.dsh\step3.ps1 -Action Status
& .\.dsh\step3.ps1 -Action Start -World q2q_warehouse_grasp50 -KinematicCarry
& .\.dsh\step7.ps1          # 只跑整链，不挂窗口
& .\.dsh\step3.ps1 -Action Stop
```

## 界面（答辩/演示）

- **Gazebo 窗口**必须走 Ogre 1.x：`--render-engine ogre`。默认 ogre2 在 VirtualBox SVGA3D 上
  **视口空白**（只有工具栏）。已写进 `bringup.launch.py`，`gui:=true` 时自动带上。
- **RViz** 能正常跑（GL 4.1 / 30+ fps），但 ssh 起窗口时要给 `XAUTHORITY=/home/ros/.Xauthority`，
  只给 `DISPLAY` 会报 `no Qt platform plugin could be initialized`。
- **推荐结构**：headless 起栈 + 独立 GUI 客户端（`ign gazebo -g --render-engine ogre`）。
  单进程 `gui:=true` 会把渲染和控制器挤在一起，冷启动时 `controller_manager` 可能被拖死。

## 结果（4 次独立冷启动，均 DONE）

| episode | START→DONE | 落点 xy 误差 | 落地中心 z | PICKING 漂移 | late_events |
|---|---:|---:|---:|---:|---:|
| `step7_1024_200714` | 97.7 s | 32.73 mm | 0.0205 m | 4.6 µm | 0 |
| `step7_1024_201108` | 255.2 s | 23.36 mm | 0.0239 m | 0.62 mm | 0 |
| `step7_1024_211150` | ≈120 s | 24 mm | 0.0250 m | — | 0 |
| `step7_1024_212902` | ≈155 s | 25.0 mm | 0.0250 m | — | 0 |

**一条真反例**：`step7_1024_195922` 状态机走到 `DONE`，但独立落地门禁判失败（投放四元数漏 `w=0`，
MoveIt 把非法 `(0,1,0,1)` 换成单位姿态）。所以「DONE」只是状态机的结论，**落地以独立门禁为准**。
样本保留在 `docs/evaluations/artifacts/2026-10-24/step7_chain/failed_delivery/`。

## 文档地图

| 想了解 | 读 |
|---|---|
| 一页纸答辩口径 | [docs/operations/第7步答辩一页.md](docs/operations/第7步答辩一页.md) |
| 演示怎么开窗口 / 怎么跑 | [docs/operations/第7步整链演示.md](docs/operations/第7步整链演示.md) |
| 接口与节点事实（唯一事实源） | [q2q_harness/项目节点总览.md](q2q_harness/项目节点总览.md) |
| 这一版修了什么、证据在哪 | [docs/evaluations/2026-10-24-第7步整链修复与验收.md](docs/evaluations/2026-10-24-第7步整链修复与验收.md) |
| 接手开发（含待办） | [docs/operations/当前开发交接.md](docs/operations/当前开发交接.md) |
| 接口定稿（历史决策） | `docs/decisions/` |
| 节点流图（交互式） | `docs/图-第7步整链节点流图.html`（若存在） |

## 常用排障

| 现象 | 真因 | 处置 |
|---|---|---|
| `controller_manager` 服务等不到 / spawner 三次超时 | `gz_ros2_control` 取 `robot_description` 超时，或渲染与控制器抢进程 | 先 `-Action Status`，`Stop` 后重 `Start`；演示改用 headless + GUI 客户端 |
| 导航 `nav_timeout` | 发 goal 时只等 action 可见，没等 Nav2 生命周期 ACTIVE | 闸门已改为等 ACTIVE；看 `orch.log` 的「还缺 …」 |
| 抓取段 `ROS response exceeded N wall seconds` | 墙钟预算按 RTF≈1 写的，仿真变慢后不够 | 设 `Q2Q_WALL_SCALE=3`（只放大墙钟，不改物理判据） |
| `DONE` 但落地门禁失败 | 投放位姿/坐标系错（历史: 四元数 `w`） | 看门禁 JSON 的 `xy`/`z`，再查 `place_plan.py` |
| Gazebo 窗口空白 | 默认 ogre2 在 SVGA3D 上不行 | 用 `--render-engine ogre` |
