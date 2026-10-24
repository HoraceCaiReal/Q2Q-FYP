# 第 7 步：任务-世界状态的单一事实来源（第一片：只读）
# 设计依据 docs/decisions/2026-10-24-第7步接口定稿-第一片.md
std_msgs/Header header        # stamp=采样时刻（仿真钟）；frame_id="map"

# 身份：迟到结果必须能被识别，所以每次转移都要能追溯（教程 13 章）
string episode_id             # 本次运行编号
string task_id                # 一条取放任务的 id
uint32 generation             # 阶段代数，每次状态转移 +1

# IDLE/NAV_TO_PICK/WAIT_STILL/PICK/STOW/NAV_TO_PLACE/PLACE/DONE/CANCELING/FAILED
string state

# 不变量所需的四个条件；UNKNOWN 一律按「不满足」处理（stale 不当作安全）
bool base_fresh               # odom 年龄 <= freshness_window
bool base_stopped             # 连续 stopped_window 内 |v|<=v_eps 且 |w|<=w_eps
bool base_command_locked      # 控制权已锁定；第一片恒 false（尚未实现加锁）
bool arm_stowed               # 机械臂接近收拢位
bool arm_executing            # 有未完成的臂动作

# 观测（SI 单位，便于记录与作图）
float64 base_x
float64 base_y
float64 base_yaw
float64 base_speed            # |v| m/s
float64 base_yaw_rate         # rad/s
bool holding_object           # 第一片恒 false（没有持物传感器，不猜）

# 诊断：当前缺的条件名（空串=都满足）；给人看，不参与逻辑
string blocked_by

# 第二片新增：导航执行与迟到结果的可追溯信息
string nav_goal_id            # 当前/上一次导航目标 id（空=没有）
string nav_status             # NONE/ACTIVE/SUCCEEDED/CANCELED/ABORTED/TIMEOUT/REJECTED
float64 distance_remaining    # 来自 feedback；-1 表示未知
string last_request_id        # 最近一次被处理的请求 id
uint32 late_events            # 迟到结果计数（教程 13 章：迟到不能无迹可查）
