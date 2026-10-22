#include "fleet_manager/task_queue.hpp"

#include <algorithm>
#include <utility>

namespace fleet {

namespace {

// "不会再自己继续走"的任务：DONE/CANCELED，或已落定（finished_at>0）的 FAILED。
// 不能用 CanRetry 判断：结构性合法的 FAILED→PENDING 与"这条任务是否还会被重排"
// 是两回事，直接 Transition 到 FAILED（如派单超时）不会再重排。
bool IsClosedTask(const Task& task)
{
  return TaskStateMachine::IsTerminal(task.state) ||
         (task.state == TaskState::kFailed && task.finished_at > 0);
}

}  // namespace

TaskQueue::TaskQueue(TaskQueueConfig cfg) : cfg_(cfg) {}

std::optional<TaskId> TaskQueue::FindDuplicate(Color color, const Pose2D& target) const
{
  for (const auto& [id, t] : tasks_) {
    // 保留期内的任务（含已结束的）都参与去重：
    // 物块是静止的，感知 4~5 Hz 会持续上报同一位置；若只比较活跃任务，
    // 任务一完成就会被立刻重新建单，形成无限任务环。
    // 重新建单的出口有两个：物块移动超过 dedup_radius，或旧任务被保留策略清理。
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
    t.finished_at = now;
  } else if (to == TaskState::kDone) {
    t.finished_at = now;
  } else if (to == TaskState::kPending) {
    t.finished_at = 0;   // 重排回队列，不再算终态
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
    t.finished_at = 0;
    ++t.revision;
    (void)now;   // 重排不记 assigned_at，保留 detected_at 便于做端到端耗时统计
    return true;
  }

  t.state = TaskState::kFailed;
  t.assigned_robot.clear();
  t.failure_reason = reason;
  t.finished_at = now;
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
  t.finished_at = now;
  ++t.revision;
  return true;
}

std::vector<TaskId> TaskQueue::Prune(TimeNs now)
{
  std::vector<TaskId> removed;
  std::vector<std::pair<TimeNs, TaskId>> terminals;

  for (const auto& [id, task] : tasks_) {
    if (!IsClosedTask(task)) {
      continue;   // 活跃、待重排的任务永不清理
    }
    if (cfg_.retention_age > 0 && task.finished_at > 0 &&
        now - task.finished_at > cfg_.retention_age)
    {
      removed.push_back(id);   // 太旧
    } else {
      terminals.emplace_back(task.finished_at, id);
    }
  }

  // 数量上限：保留 finished_at 最新的 max_terminal 个；同一时间戳按 id 稳定排序
  if (terminals.size() > cfg_.max_terminal) {
    std::sort(terminals.begin(), terminals.end());
    const std::size_t drop = terminals.size() - cfg_.max_terminal;
    for (std::size_t i = 0; i < drop; ++i) {
      removed.push_back(terminals[i].second);
    }
  }

  for (const auto& id : removed) {
    tasks_.erase(id);
  }
  return removed;
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
