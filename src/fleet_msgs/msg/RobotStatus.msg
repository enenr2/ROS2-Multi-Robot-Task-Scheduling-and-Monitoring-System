# 机器人状态心跳（1Hz 发布，调度器据此判活）
uint8 STATE_IDLE=0
uint8 STATE_BUSY=1
uint8 STATE_ERROR=2
uint8 STATE_OFFLINE=3       # 由调度器按心跳超时置位，agent 自身不会发布此状态

string robot_id
uint8 state

geometry_msgs/Pose pose      # map 系平面位姿（不是 odom）：z / roll / pitch 恒为 0
string current_task          # 空字符串表示无任务
uint16 tasks_done
uint16 tasks_failed
uint16 tasks_canceled    # 被取消的任务不计入 failed，单独统计
uint32 heartbeat_seq         # 递增序号，用于区分丢包与停顿

builtin_interfaces/Time stamp
