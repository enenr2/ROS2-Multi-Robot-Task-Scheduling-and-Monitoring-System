#include "fleet_manager/watchdog.hpp"

namespace fleet {

Watchdog::Watchdog(WatchdogConfig cfg) : cfg_(cfg) {}

bool Watchdog::IsTaskTimedOut(const Task& t, TimeNs now, TimeoutEvent* out) const
{
  TimeNs deadline = 0;
  const char* reason = "";

  switch (t.state) {
    case TaskState::kAssigned:
      deadline = t.assigned_at + cfg_.assigned_timeout;
      reason = "GOAL_ACK_TIMEOUT";
      break;
    case TaskState::kNavigating:
      // NAVIGATING 的计时起点是分配到车的那一刻
      deadline = t.assigned_at + cfg_.nav_timeout;
      reason = "NAV_TIMEOUT";
      break;
    case TaskState::kDocking:
      // 对位阶段同样从分配时刻起算（assigned_at 在进入 ASSIGNED 时写入，
      // 不随阶段刷新），因此 Dock 的超时是"从派单到对位"的总时长上限
      deadline = t.assigned_at + cfg_.dock_timeout;
      reason = "DOCK_TIMEOUT";
      break;
    case TaskState::kPending: {
      // 一直没人可派：从检测到目标的时间起算
      const TimeNs pending_since = t.pending_since > 0 ? t.pending_since : t.detected_at;
      if (pending_since <= 0) {
        // 缺少创建时间说明任务数据不完整，不能把 Unix/稳态时钟的
        // 当前值当成已超时，避免产生无法收敛的重复告警。
        return false;
      }
      deadline = pending_since + cfg_.assign_timeout;
      reason = "ASSIGN_TIMEOUT";
      break;
    }
    default:
      return false;
  }

  if (t.assigned_at == 0 && t.state != TaskState::kPending) {
    // 没有分配时间戳说明数据不完整，不判定超时，避免误伤
    return false;
  }

  if (now <= deadline) {
    return false;
  }

  if (out != nullptr) {
    out->kind = t.state == TaskState::kPending ? TimeoutKind::kAssign
                                                : TimeoutKind::kTask;
    out->task_id = t.id;
    out->robot_id.clear();
    out->state_at_timeout = t.state;
    out->reason = reason;
    out->overdue = now - deadline;
  }
  return true;
}

bool Watchdog::IsRobotOffline(const Robot& r, TimeNs now, TimeoutEvent* out) const
{
  // 从未上报过心跳的机器人不参与判定：注册后到第一次心跳之间
  // 不该被误杀
  if (r.last_heartbeat == 0) {
    return false;
  }
  if (r.state == RobotState::kOffline) {
    return false;   // 已标记，避免重复上报
  }
  const TimeNs deadline = r.last_heartbeat + cfg_.heartbeat_timeout;
  if (now <= deadline) {
    return false;
  }

  if (out != nullptr) {
    out->kind = TimeoutKind::kRobotOffline;
    out->task_id.clear();
    out->robot_id = r.id;
    out->state_at_timeout = TaskState::kPending;   // 无任务语义，置默认值
    out->reason = "ROBOT_UNREACHABLE";
    out->overdue = now - deadline;
  }
  return true;
}

std::vector<TimeoutEvent> Watchdog::Check(const std::vector<Task>& tasks,
                                          const std::vector<Robot>& robots,
                                          TimeNs now) const
{
  std::vector<TimeoutEvent> events;

  for (const auto& t : tasks) {
    TimeoutEvent ev;
    if (IsTaskTimedOut(t, now, &ev)) {
      events.push_back(ev);
    }
  }

  for (const auto& r : robots) {
    TimeoutEvent ev;
    if (IsRobotOffline(r, now, &ev)) {
      events.push_back(ev);
    }
  }

  return events;
}

}  // namespace fleet
