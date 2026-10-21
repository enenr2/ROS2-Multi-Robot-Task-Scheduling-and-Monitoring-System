// 分配策略：纯函数，输入是任务与机器人快照，输出是"选谁"。
//
// 不做任何状态修改 —— 选人和改状态分开，好处是选人逻辑能被穷举测试，
// 也便于以后替换策略（比如按剩余电量、按负载）而不动任务表。
#pragma once

#include <optional>
#include <vector>

#include "fleet_manager/task_types.hpp"

namespace fleet {

struct AssignConfig {
  // 两个任务目标相距小于此值则不允许并行执行，避免两台车抢同一个物块
  double target_exclusion_radius{1.0};
};

struct AssignDecision {
  RobotId robot;     // 空表示没有可用的车
  bool blocked_by_target_exclusion{false};

  bool HasRobot() const { return !robot.empty(); }
};

class Assignment
{
public:
  explicit Assignment(AssignConfig cfg = AssignConfig{});

  // 最近空闲优先；平局按 robot_id 字典序，保证结果可复现。
  // active_targets 是当前已被占用的目标位置（来自 TaskQueue::ActiveTargets）。
  AssignDecision SelectRobot(const Pose2D& task_target,
                             const std::vector<Robot>& robots,
                             const std::vector<Pose2D>& active_targets) const;

  // 目标互斥判定单独暴露，便于单测覆盖边界（正好等于阈值等）
  bool IsTargetBlocked(const Pose2D& task_target,
                       const std::vector<Pose2D>& active_targets) const;

private:
  AssignConfig cfg_;
};

}  // namespace fleet
