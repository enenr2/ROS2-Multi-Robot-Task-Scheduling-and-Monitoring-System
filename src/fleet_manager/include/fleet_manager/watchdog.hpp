// 看门狗：只负责"判定"，不负责"执行"。
//
// 每次 Tick 返回一组待处理动作，由上层决定怎么落实（取消 action、
// 重排任务、把车标记离线）。这样超时规则可以脱离 ROS 单测，
// 也不会因为"在判定里顺手改数据"而出现跨模块的隐式副作用。
#pragma once

#include <string>
#include <vector>

#include "fleet_manager/task_types.hpp"

namespace fleet {

struct WatchdogConfig {
  TimeNs nav_timeout{60 * kSecond};      // NAVIGATING 阶段的超时
  TimeNs dock_timeout{30 * kSecond};     // DOCKING 阶段的超时
  TimeNs assign_timeout{30 * kSecond};   // PENDING 等待分配的超时
  TimeNs heartbeat_timeout{3 * kSecond}; // 心跳超过此值视为离线
};

enum class TimeoutKind {
  kTask,        // 任务阶段超时（携带 task_id 与所处阶段）
  kAssign,      // 任务长时间无人可派
  kRobotOffline // 机器人心跳超时
};

struct TimeoutEvent {
  TimeoutKind kind{TimeoutKind::kTask};
  TaskId task_id;            // kRobotOffline 时为空
  RobotId robot_id;          // kTask 时为空
  TaskState state_at_timeout{TaskState::kPending};
  std::string reason;        // 直接可写入 TaskStatus.failure_reason
  TimeNs overdue{0};         // 超出时限多久，便于监控排序
};

class Watchdog
{
public:
  explicit Watchdog(WatchdogConfig cfg = WatchdogConfig{});

  // now 由调用方传入，内部不取时钟
  std::vector<TimeoutEvent> Check(const std::vector<Task>& tasks,
                                  const std::vector<Robot>& robots,
                                  TimeNs now) const;

  // 单任务判定，便于测试逐条覆盖
  bool IsTaskTimedOut(const Task& t, TimeNs now, TimeoutEvent* out) const;

  // 单机器人判定
  bool IsRobotOffline(const Robot& r, TimeNs now, TimeoutEvent* out) const;

private:
  WatchdogConfig cfg_;
};

}  // namespace fleet
