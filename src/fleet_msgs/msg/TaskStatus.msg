# 任务状态变更事件（agent -> 调度器/监控）
uint8 STATE_PENDING=0
uint8 STATE_ASSIGNED=1
uint8 STATE_NAVIGATING=2
uint8 STATE_DOCKING=3
uint8 STATE_DONE=4
uint8 STATE_FAILED=5
uint8 STATE_CANCELED=6

string task_id
string robot_id
uint8 state

float32 progress              # 0.0 ~ 1.0
string detail                 # 当前动作描述，供日志与 UI 展示

# 失败原因枚举（空闲时为 "NONE"）
# NAV_TIMEOUT / NAV_ABORTED / TARGET_LOST / DOCK_TIMEOUT
# / ROBOT_UNREACHABLE / CANCELED_BY_OPERATOR / NONE
string failure_reason
uint8 retry_count

builtin_interfaces/Time detected_at    # 任务来源时间（从 Task 透传）
builtin_interfaces/Time stamp
