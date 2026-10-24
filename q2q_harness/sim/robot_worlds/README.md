# robot_worlds —— Gazebo 世界（纯数据包）

| 世界 | 用途 | 状态 |
|---|---|---|
| `q2q_empty` | 地面 + 光 + 物理/渲染插件，无 Fuel 依赖 | 第 1 步已用 |
| `q2q_warehouse` | 货架 / 料箱 / 操作台 / 通道 | 第 3 步（导航避障）起加 |

## 两条硬规则

1. **文件名 == world name**：`q2q_empty.sdf` 里必须是 `<world name="q2q_empty">`。
   启动器用文件名生成 `gz sim` 参数，用 name 拼传感器话题前缀 `/world/<name>/model/...`，
   两者不一致 → 激光/相机话题全断，且报错很不明显。
2. **不引 Fuel 远程模型**。`warehouse.sdf` 首次启动要联网下载模型，之前把 VM 拖死过一次。
   需要货架就用基本几何体（box/cylinder）在本地 `.sdf` 里拼。

## 怎么换世界

在 `robot_bringup/config/bringup.yaml` 里改 `world:`，或命令行 `world:=q2q_warehouse`。
`world:=empty` / `world:=warehouse` 会用 Clearpath 自带的世界。
