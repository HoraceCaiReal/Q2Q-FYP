# URDF `<gazebo reference>` 到物理引擎的通路核实（2026-10-24 第二轮）

> 起因：10-24 第一轮在「方案 A（指面 mu=2.0 + 关节 damping）改了等于没改」之后提出一条怀疑——
> 既然吸盘插件（模型级 `<gazebo><plugin>`）进不了 spawn 模型，
> 那么打在 URDF 上的 `<gazebo reference>`（指面摩擦、关节阻尼）**可能同样没生效**，
> 那之前参数调整就是白做。本轮只做核实，不写功能代码、不改仿真参数。

**一句话结论**：**这条通路是通的，怀疑不成立。**
`<gazebo reference>` 写的 `mu1/mu2` 与关节 `damping` 既进了 spawn 源，也进了转换后的 SDF，
并且在真物理里确实改变行为（mu=0.05 滑走、mu=1.5 停在原地）。
顺带推翻另一条结论：**模型级 `<gazebo><plugin>` 同样能进 spawn 模型**（`ign_ros2_control` 就是靠它工作的），
`DetachableJoint` 插件在本机也**已安装**，所以当时「吸附机制不可用」的归因也站不住。

## 证据链（按 hop 排序，每条都给原始命令与输出）

| # | 断言 | 手段 | 结论强度 |
|---|---|---|---|
| H1 | 补丁确实写进了 URDF | `grep` 生成物 | 实测 |
| H2 | spawn 源就是这份补丁后的 URDF | `cat robot.urdf.xacro` + `grep` 上游 launch | 实测 |
| H3 | URDF→SDF 转换保留了摩擦与阻尼 | `ign sdf -p`（sdformat 12.9.0） | 实测 |
| H4 | 模型级 `<plugin>` 也能过转换 | 同上，数 `<plugin` | 实测 |
| H5 | `mu` 确实改变物理行为 | 隔离世界对照实验（`ros_gz_sim create` 同一条路径） | 实测 |

### H1 补丁在 URDF 里

```
$ grep -n 'mu1\|damping' ~/.q2q/setup/a200_kinova/robot.patched.urdf | head
757:  <mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
765:  <dynamics damping="0.1" friction="0.0" /></joint>
791:  <mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
800:  <dynamics damping="0.1" friction="0.0" /></joint>
826/860: 同上（左右各 2 个指节 link，共 4 个）
```

raw 版只有 4 处 `mu1`（四个轮子，`<mu1 value="0.5"/>`），patched 版 8 处 + 4 处 `damping`，
差额正好是补丁加的那 4 个指面与 4 个指节关节。

### H2 spawn 源 = 补丁后的 URDF

```
$ cat ~/.q2q/setup/a200_kinova/robot.urdf.xacro
<robot ... name="j100-0000">
  <xacro:include filename="/home/ros/.q2q/setup/a200_kinova/robot.patched.urdf"/>
</robot>

$ grep -n "-topic" /opt/ros/humble/share/clearpath_gz/launch/robot_spawn.launch.py
127:                       '-topic', 'robot_description'],
```

即 `robot_description`（来自上面那个 xacro）→ `ros_gz_sim create -topic` → Gazebo。

### H3 转换后摩擦与阻尼都还在（关键）

```
$ ign sdf -p ~/.q2q/setup/a200_kinova/robot.patched.urdf > /tmp/conv_patched.sdf   # exit=0
$ grep -n -A4 '<friction>' /tmp/conv_patched.sdf | sed -n '8,12p'
624-          <friction>
625-            <ode>
626-              <mu>2</mu>
627-              <mu2>2</mu2>
628-            </ode>

$ grep -c '<mu>' : patched=8  raw=4        # 4 轮 + 4 指面
$ grep -c '<mu2>': patched=8  raw=4
$ grep -c 'damping' : 4 → <damping>0.10000000000000001</damping>
```

两点要注意（都是这次看清楚的）：

1. **SDF 里的标签名是 `<mu>`，不是 `<mu1>`**。sdformat 的 URDF 解析器把 URDF 的 `<mu1 value="x"/>`
   映射成 SDF 的 `<mu>x</mu>`。所以**用 `grep mu1` 去查转换结果永远查不到**——这很可能就是上一轮
   误判「补丁没生效」的地方之一。
2. 写进 URDF 的 `<gazebo reference>` 块即使写成 SDF 风格（`<mu1>2.0</mu1>` 没有 `value` 属性），
   转换器也照样收（因为 gazebo 扩展块是整块拷贝+归一化）。

### H4 模型级插件也能过转换

```
$ grep -n -A2 '<plugin' /tmp/conv_patched.sdf | head -20
1146:    <plugin name='ignition::gazebo::systems::WheelSlip' filename='libignition-gazebo-wheel-slip-system.so'>
1172:    <plugin name='ign_ros2_control::IgnitionROS2ControlPlugin' filename='libign_ros2_control-system.so'>
1187:    <plugin name='ignition::gazebo::systems::PosePublisher' ...>
1193:    <plugin name='ignition::gazebo::systems::OdometryPublisher' ...>
```

这 4 个都来自 Clearpath 的模型级 `<gazebo>` 块。**`ign_ros2_control` 显然在工作**（4 个控制器都能 active），
所以「`ros_gz_sim create` 从 URDF 转 SDF 不支持 `<gazebo><plugin>`」这个结论是错的。

### H5 隔离世界对照：mu 真的改变物理

做法：一个只有静态平台的世界，**重力改成 `3 0 -9.8`**（等价于把物体放在坡度 0.31 的斜面上），
用 `ros2 run ros_gz_sim create -file <box.urdf>` 生成两个只有 `mu` 不同的箱子（同一条 spawn/转换路径）。

```
$ # world: <gravity>3 0 -9.8</gravity> + 静态平台(4×4×0.2, 顶面 z=0.1)
$ # box.urdf: <gazebo reference="box_link"><mu1 value="__MU__"/><mu2 value="__MU__"/></gazebo>
=== after 1 more s ===
mu=0.05   [3.869790 0.000000 -0.972361]     # 已滑出平台、正在下坠
mu=1.5    [0.046200 0.000000 0.149999]      # 静止在 z=0.15（平台顶 0.1 + 半个箱高 0.05）
=== after 3 more s ===
mu=0.05   [85.752400 0.000000 -206.212000]
mu=1.5    [0.046201 0.000000 0.150000]      # 6 s 内一动不动
```

同一份代码路径、同一个引擎、只差 URDF 里那两个数：**一个滑走，一个钉住**。
`tanθ = 3/9.8 = 0.31`，所以 μ=0.05 < 0.31 必滑、μ=1.5 > 0.31 必停——结果与理论一致。

## 对被推翻的那两条结论

| 原结论（10-24 第一轮） | 本轮证据 | 现在的说法 |
|---|---|---|
| 「`<gazebo reference>` 的 mu/damping 可能同样没生效」 | H1–H5 | **不成立**。通道通，参数确实到了物理 |
| 「方案 C 吸附机制在本机不可用」 | H4 + 插件文件存在 | **不成立**。模型级插件能进模型；`libignition-gazebo6-detachable-joint-system.so`（6.18.0）已安装 |

第一轮那三条「证据」里，第 1 条（`/world/.../sdf` 查不到插件）**判据本身无效**——
那个查询根本不返回模型的 SDF，所以什么都查不到并不说明任何事。
第 2、3 条（插件写进世界文件、或隔离最小世界，`ign topic -l` 没有 attach/detach topic）是真的发生了，
但原因要从**插件配置**找，不是从「机制不可用」找。已知方向：`DetachableJoint` 的 topic 名由**插件所属模型名**决定、
父子 link 必须能按名字解析到；名字对不上时它不报错、只是不建话题。

## 对开发计划的影响

1. **方案 A 的参数没有被白费**：指面摩擦与关节阻尼确实进了模型。方案 A「改了等于没改」的解释要另找
   （更可能是楔形指面几何 + 位置控制无夹持力，而不是参数没生效）——这一点仍未证。
2. **真吸附（方案 C）没有被排除**：插件在、模型级插件能加载，剩下的是配置问题。
   如果要回到「真夹持/真吸附」，这是成本最低的方向（比重写 C++ 系统插件、或整栈升 Harmonic 便宜得多）。
3. **不要再用「`grep mu1` 查转换结果」当下结论**：SDF 侧的名字是 `<mu>`。

## 仍未证的部分（不许写成定论）

- 运行中的机器人模型里，指面 `<mu>` 到底是 2 还是被别的东西覆盖（例如 Clearpath 自己又写了一份 surface）。
  本轮只证明了「文件形态的转换 + 一条同路径的对照实验」，**没有从运行中的模型里读回 SDF**（本机没有这种只读接口）。
- `damping=0.1` 的实际效果没有单独做对照（它与摩擦同一个通道，但没单独测）。
- DetachableJoint 具体卡在哪一步，需要单独一轮带着 `-v4` 日志和正确 topic 名去验。

## 复现方法（探针脚本按排障协议已删，要点在此）

```bash
# H3/H4：转换结果
ign sdf -p ~/.q2q/setup/a200_kinova/robot.patched.urdf > /tmp/conv.sdf
grep -c '<mu>' /tmp/conv.sdf ; grep -c 'damping' /tmp/conv.sdf ; grep -c '<plugin' /tmp/conv.sdf

# H5：隔离对照（世界用 <gravity>3 0 -9.8</gravity> + 静态平台；箱子用 URDF + <gazebo reference>）
ign gazebo -s -r -v1 world.sdf &
ros2 run ros_gz_sim create -world <w> -file box_<mu>.urdf -name box_<mu> -z 0.30
ign model -m box_<mu> -p        # 读回位姿
```

**工具坑（我自己踩的，记下来防复发）**：
- URDF 的 box 几何是 `<box size="0.1 0.1 0.1"/>`（属性），不是 `<box><size>...</size></box>`；
  写成 SDF 风格时转换器**静默丢掉整个 collision/visual**，看起来就像「碰撞失效」——白跑了三轮。
- `ign gazebo -s` + 项目自带世界（含 Sensors 插件）在这台 VM 上会 abort；对照实验要用最小世界。
- `ros_gz_sim` 的可执行名是 `create`（`ros2 run ros_gz_sim create`），不是 `ros_gz_sim create`。
