# 任务快照（调度器单向发布，供监控/回放使用；agent 不订阅此话题）
uint8 TYPE_APPROACH=1
uint8 TYPE_INSPECT=2

string task_id
uint8 type
uint8 target_color            # 1=红 2=绿 3=蓝，与 Detection 的常量一致
uint8 state                   # 取值同 TaskStatus 的 STATE_*（监控端不必再拼事件流）

geometry_msgs/Pose target_pose

# 分配归属：空字符串表示尚未分配
string assigned_robot_id
uint64 revision               # 每次状态变更 +1，供监控端判序与丢弃过期包

# 时间与重试
builtin_interfaces/Time detected_at    # 目标首次被检测到的时间
builtin_interfaces/Time assigned_at    # 最近一次分配时间，未分配时为 0
uint8 retry_count
uint8 max_retries
