# 一键演示自动截图（2026-10-24 21:28）

由 `.dsh\demo.ps1` 一键跑出：headless 起栈 → 挂 Gazebo GUI 客户端（ogre 引擎）+ RViz →
发一条 START 跑完整链，主机侧**每当状态变化自动截一张图**。

- episode：`step7_1024_212902`
- 结果：`DONE`、12 个状态、`exit_code=0`、`late_events=0`、记录 3269 行
- 落点：箱体真值 (0.3663, 0.1164, 0.0250) m；相对底盘前方 0.40 m 的误差 **25.0 mm**
  （限 80 mm），中心 z 25 mm（限 15–40 mm）
- 帧文件：`NN_<状态>.png`，按状态顺序命名，答辩时可直接按顺序讲

| 文件 | 状态 |
|---|---|
| `01_IDLE.png` | 起栈就绪、等命令 |
| `02_PICKING.png` | 已上锁、机械臂在取 |
| `03_STOWING.png` | 抓取成功、收臂中 |
| `04_UNDOCKING.png` | 出站后退 |
| `05_NAV_TO_PLACE.png` | 运回投放点 |
| `06_PLACE_READY.png` | 到达投放位、重新上锁 |
| `07_DONE.png` | 落地完成（Gazebo + RViz 同屏） |

> 口径：物体由运动学支撑插件带着走（`grasp_mode=suction_approx`），**不是物理夹持/吸附**。
> 详见 [第7步整链演示](../../../../../operations/第7步整链演示.md) 与
> [答辩一页](../../../../../operations/第7步答辩一页.md)。
