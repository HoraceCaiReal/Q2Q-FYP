# 一次抓取观测：从「图里有个橙色块」到「手该怎么放」。
# 字段与 docs/decisions/2026-10-23-第5步观测契约.md 一一对应。

std_msgs/Header header          # stamp = 测量时刻（不是发布时刻）；frame_id = pose 所在坐标系
string target_id                # 多物体/多帧时不抓错目标
geometry_msgs/Pose pose         # 抓取位姿（TCP 朝向约定见契约文档）
float64 width_m                 # 所需开口，必须和夹爪行程核对
float32 confidence              # 过滤用；不等于位置误差
bool valid                      # 显式有效性，不用 (0,0,0) 表示失败
string reason                   # 无效原因，写进日志与复盘
string scene_version            # 避免旧物体位置配新规划场景；暂无生产者时为空串
