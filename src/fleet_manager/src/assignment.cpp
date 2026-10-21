#include "fleet_manager/assignment.hpp"

#include <limits>

namespace fleet {

Assignment::Assignment(AssignConfig cfg) : cfg_(cfg) {}

bool Assignment::IsTargetBlocked(const Pose2D& task_target,
                                 const std::vector<Pose2D>& active_targets) const
{
  for (const auto& t : active_targets) {
    // 用 < 而不是 <=：正好等于阈值时不阻塞，边界取"更宽松"一侧，
    // 避免两个 1.0m 间距的任务被误判成冲突
    if (Distance2D(task_target, t) < cfg_.target_exclusion_radius) {
      return true;
    }
  }
  return false;
}

AssignDecision Assignment::SelectRobot(const Pose2D& task_target,
                                       const std::vector<Robot>& robots,
                                       const std::vector<Pose2D>& active_targets) const
{
  AssignDecision d;

  if (IsTargetBlocked(task_target, active_targets)) {
    d.blocked_by_target_exclusion = true;
    return d;
  }

  double best = std::numeric_limits<double>::infinity();
  for (const auto& r : robots) {
    if (r.state != RobotState::kIdle) {
      continue;   // BUSY / ERROR / OFFLINE 都不参与分配
    }
    const double dist = Distance2D(r.pose, task_target);

    // 距离更近者胜；距离完全相同时取 robot_id 字典序更小者。
    // 平局规则写在策略内部而不是依赖调用方先排序：否则"结果可复现"
    // 这个保证会散落到每个调用点，很容易漏。
    const bool better = (dist < best) ||
                        (dist == best && !d.robot.empty() && r.id < d.robot);
    if (better) {
      best = dist;
      d.robot = r.id;
    }
  }
  return d;
}

}  // namespace fleet
