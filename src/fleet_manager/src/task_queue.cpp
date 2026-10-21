#include "fleet_manager/task_queue.hpp"

#include <utility>

namespace fleet {

TaskQueue::TaskQueue(TaskQueueConfig cfg) : cfg_(cfg) {}

std::optional<TaskId> TaskQueue::FindDuplicate(Color color, const Pose2D& target) const
{
  for (const auto& [id, t] : tasks_) {
    if (t.state == TaskState::kCanceled) {
      continue;   // 已取消的目标允许重新发现
    }
    if (t.color == color && Distance2D(t.target, target) < cfg_.dedup_radius) {
      return id;
    }
  }
  return std::nullopt;
}

bool TaskQueue::AddTask(const TaskId& id, Color color, const Pose2D& target, TimeNs detected_at)
{
  if (id.empty() || tasks_.count(id) > 0) {
    return false;
  }
  if (FindDuplicate(color, target).has_value()) {
    return false;
  }

  Task t;
  t.id = id;
  t.color = color;
  t.target = target;
  t.state = TaskState::kPending;
  t.detected_at = detected_at;
  t.max_retries = cfg_.default_max_retries;
  t.revision = 1;   // 创建即算一次变更，监控端可据此区分"新建"与"更新"
  tasks_.emplace(id, std::move(t));
  return true;
}

bool TaskQueue::Transition(const TaskId& id, TaskState to, TimeNs now,
                           const std::string& failure_reason)
{
  auto it = tasks_.find(id);
  if (it == tasks_.end()) {
    return false;
  }
  Task& t = it->second;
  if (!TaskStateMachine::IsLegal(t.state, to)) {
    return false;
  }

  t.state = to;
  ++t.revision;
  if (to == TaskState::kAssigned) {
    t.assigned_at = now;
  }
  if (to == TaskState::kFailed || to == TaskState::kCanceled) {
    t.failure_reason = failure_reason;
    t.assigned_robot.clear();
  }
  return true;
}

bool TaskQueue::AssignTo(const TaskId& id, const RobotId& robot, TimeNs now)
{
  auto it = tasks_.find(id);
  if (it == tasks_.end()) {
    return false;
  }
  Task& t = it->second;
  if (t.state != TaskState::kPending || robot.empty()) {
    return false;
  }

  t.state = TaskState::kAssigned;
  t.assigned_robot = robot;
  t.assigned_at = now;
  ++t.revision;
  return true;
}

bool TaskQueue::RetryOrFail(const TaskId& id, TimeNs now, const std::string& reason)
{
  auto it = tasks_.find(id);
  if (it == tasks_.end()) {
    return false;
  }
  Task& t = it->second;
  if (t.state != TaskState::kFailed && t.state != TaskState::kAssigned &&
      t.state != TaskState::kNavigating && t.state != TaskState::kDocking) {
    return false;
  }

  if (TaskStateMachine::CanRetry(t)) {
    t.retry_count += 1;
    t.state = TaskState::kPending;
    t.assigned_robot.clear();
    t.assigned_at = 0;
    t.failure_reason = reason;
    ++t.revision;
    (void)now;   // 重排不记 assigned_at，保留 detected_at 便于做端到端耗时统计
    return true;
  }

  t.state = TaskState::kFailed;
  t.assigned_robot.clear();
  t.failure_reason = reason;
  ++t.revision;
  return false;
}

bool TaskQueue::Cancel(const TaskId& id, TimeNs now, const std::string& reason)
{
  auto it = tasks_.find(id);
  if (it == tasks_.end()) {
    return false;
  }
  Task& t = it->second;
  if (TaskStateMachine::IsTerminal(t.state)) {
    return false;
  }
  t.state = TaskState::kCanceled;
  t.failure_reason = reason;
  t.assigned_robot.clear();
  ++t.revision;
  (void)now;
  return true;
}

const Task* TaskQueue::Find(const TaskId& id) const
{
  auto it = tasks_.find(id);
  return it == tasks_.end() ? nullptr : &it->second;
}

std::vector<Task> TaskQueue::All() const
{
  std::vector<Task> out;
  out.reserve(tasks_.size());
  for (const auto& kv : tasks_) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<Task> TaskQueue::ByState(TaskState s) const
{
  std::vector<Task> out;
  for (const auto& kv : tasks_) {
    if (kv.second.state == s) {
      out.push_back(kv.second);
    }
  }
  return out;
}

std::vector<Pose2D> TaskQueue::ActiveTargets() const
{
  std::vector<Pose2D> out;
  for (const auto& kv : tasks_) {
    const Task& t = kv.second;
    if (t.state == TaskState::kAssigned || t.state == TaskState::kNavigating ||
        t.state == TaskState::kDocking) {
      out.push_back(t.target);
    }
  }
  return out;
}

}  // namespace fleet
