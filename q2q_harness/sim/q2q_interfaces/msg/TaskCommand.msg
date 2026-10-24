# 第 7 步第二片：上层 → 调度的命令事件（topic，不阻塞、可重放）
std_msgs/Header header
string request_id      # 请求 id：重复/迟到的请求靠它过滤，同一 id 只处理一次
uint8 command          # 1=START 2=CANCEL
uint8 START=1
uint8 CANCEL=2
string task_id
float64 target_x       # map 帧目标（工位）
float64 target_y
float64 target_yaw
float64 timeout_s      # 导航超时（秒）；<=0 用节点参数默认
