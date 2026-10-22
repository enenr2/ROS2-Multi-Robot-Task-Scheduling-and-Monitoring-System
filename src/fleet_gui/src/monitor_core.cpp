#include "fleet_gui/monitor_core.hpp"

#include <algorithm>
#include <cstdio>
#include <ctime>

namespace fleet::monitor {

namespace {

std::string FormatPose(double x, double y)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "(%.2f, %.2f)", x, y);
  return buf;
}

int RankOf(int state)
{
  switch (state) {
    case kStateAssigned:
    case kStateNavigating:
    case kStateDocking:
      return 0;
    case kStatePending:
      return 1;
    case kStateFailed:
      return 2;
    case kStateCanceled:
      return 3;
    case kStateDone:
      return 4;
    default:
      return 5;
  }
}

}  // namespace

bool IsActiveState(int state)
{
  return state == kStateAssigned || state == kStateNavigating || state == kStateDocking;
}

bool IsTerminalState(int state)
{
  return state == kStateDone || state == kStateFailed || state == kStateCanceled;
}

std::string StateName(int state)
{
  switch (state) {
    case kStatePending: return "待分配";
    case kStateAssigned: return "已派单";
    case kStateNavigating: return "导航中";
    case kStateDocking: return "对位中";
    case kStateDone: return "已完成";
    case kStateFailed: return "失败";
    case kStateCanceled: return "已取消";
    default: return "状态" + std::to_string(state);
  }
}

std::string RobotStateName(int state)
{
  switch (state) {
    case kRobotIdle: return "空闲";
    case kRobotBusy: return "执行中";
    case kRobotError: return "异常";
    case kRobotOffline: return "离线";
    default: return "状态" + std::to_string(state);
  }
}

std::string ColorName(int color)
{
  switch (color) {
    case kColorRed: return "红";
    case kColorGreen: return "绿";
    case kColorBlue: return "蓝";
    default: return "未知色";
  }
}

std::string ReasonText(const std::string& reason)
{
  if (reason.empty() || reason == "NONE") {
    return "无";
  }
  static const std::map<std::string, std::string> kText = {
    {"NAV_TIMEOUT", "导航超时"},
    {"NAV_ABORTED", "导航被中止"},
    {"TARGET_LOST", "目标丢失"},
    {"DOCK_TIMEOUT", "对位超时"},
    {"ROBOT_UNREACHABLE", "机器人失联"},
    {"CANCELED_BY_OPERATOR", "操作员取消"},
    {"ASSIGN_TIMEOUT", "派单超时"},
    {"GOAL_REJECTED", "目标被拒绝"},
    {"AGENT_FAILED", "执行失败"},
    {"UNKNOWN_TASK", "未知任务"},
  };
  const auto it = kText.find(reason);
  return it == kText.end() ? reason : it->second;
}

std::string FormatClock(int64_t ms)
{
  std::time_t sec = static_cast<std::time_t>(ms / 1000);
  std::tm tm_buf{};
  localtime_r(&sec, &tm_buf);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm_buf);
  return buf;
}

std::string FormatAge(int64_t ms)
{
  if (ms < 0) {
    ms = 0;
  }
  char buf[32];
  if (ms < 1000) {
    std::snprintf(buf, sizeof(buf), "%lldms", static_cast<long long>(ms));
  } else if (ms < 60000) {
    std::snprintf(buf, sizeof(buf), "%.1fs", static_cast<double>(ms) / 1000.0);
  } else {
    std::snprintf(buf, sizeof(buf), "%lldm%02llds",
                  static_cast<long long>(ms / 60000),
                  static_cast<long long>((ms % 60000) / 1000));
  }
  return buf;
}

MonitorModel::MonitorModel(int64_t offline_timeout_ms)
: offline_timeout_ms_(offline_timeout_ms) {}

void MonitorModel::Clear()
{
  tasks_.clear();
  robots_.clear();
  last_state_.clear();
  progress_.clear();
  detail_.clear();
  retry_.clear();
  ever_seen_.clear();
  no_heartbeat_alarmed_.clear();
}

std::vector<Event> MonitorModel::EnsureRobot(const std::string& robot_id, int64_t now_ms)
{
  std::vector<Event> out;
  if (robot_id.empty() || robots_.count(robot_id) > 0) {
    return out;
  }
  RobotView robot;
  robot.robot_id = robot_id;
  robot.state = kRobotOffline;
  robot.online = false;
  robot.heartbeat_seen = false;
  robot.last_seen_ms = now_ms;   // 登记时刻，用于判断"多久还没心跳"
  robots_[robot_id] = robot;

  Event ev;
  ev.stamp_ms = now_ms;
  ev.robot_id = robot_id;
  ev.text = robot_id + " 已登记，等待心跳";
  out.push_back(ev);
  return out;
}

std::vector<Event> MonitorModel::Announce(const std::string& task_id, const std::string& robot,
                                          int state, const std::string& reason,
                                          const std::string& detail, int64_t now_ms)
{
  const auto it = last_state_.find(task_id);
  if (it != last_state_.end() && it->second == state) {
    return {};   // 同一状态重复上报，不刷屏
  }
  last_state_[task_id] = state;
  if (state == kStatePending) {
    return {};   // 建单时已经记过"新任务"，不再重复
  }

  Event ev;
  ev.stamp_ms = now_ms;
  ev.robot_id = robot;
  ev.severity = SeverityOf(state);
  ev.text = task_id + " " + StateName(state);

  switch (state) {
    case kStateAssigned:
      if (!robot.empty()) {
        ev.text += "，执行车 " + robot;
      }
      break;
    case kStateFailed:
      ev.text += "（" + ReasonText(reason) + "）";
      break;
    case kStateCanceled:
      ev.text += "（" + ReasonText(reason) + "）";
      break;
    default:
      break;
  }
  if (!detail.empty() && (state == kStateFailed || state == kStateCanceled)) {
    ev.text += "：" + detail;
  }
  return {ev};
}

Severity MonitorModel::SeverityOf(int state)
{
  if (state == kStateFailed) {
    return Severity::kError;
  }
  if (state == kStateCanceled) {
    return Severity::kWarn;
  }
  return Severity::kInfo;
}

std::vector<Event> MonitorModel::OnTaskSnapshot(const TaskView& task, int64_t now_ms)
{
  std::vector<Event> out;
  if (task.task_id.empty()) {
    return out;
  }

  const auto it = tasks_.find(task.task_id);
  if (it != tasks_.end() && task.revision <= it->second.revision) {
    return out;   // 旧快照晚到，丢弃
  }
  const bool is_new = it == tasks_.end();
  const std::string prev_robot = is_new ? std::string() : it->second.robot;
  TaskView stored = task;
  if (is_new || stored.first_seen_ms <= 0) {
    stored.first_seen_ms = now_ms;
  } else {
    stored.first_seen_ms = it->second.first_seen_ms;
  }
  tasks_[task.task_id] = stored;

  if (is_new) {
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = task.robot;
    ev.severity = Severity::kInfo;
    ev.text = "新任务 " + task.task_id + "：" + ColorName(task.color) + " 目标 " +
              FormatPose(task.x, task.y);
    out.push_back(ev);
  }

  // 重试次数由调度器维护，快照里最权威；状态事件里的 retry 可能落后半拍
  const auto rit = retry_.find(task.task_id);
  if (rit == retry_.end()) {
    retry_[task.task_id] = task.retry;
  } else if (task.retry > rit->second) {
    rit->second = task.retry;
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = task.robot;
    ev.severity = Severity::kWarn;
    ev.text = task.task_id + " 第 " + std::to_string(task.retry) + " 次重排（" +
              ReasonText(task.failure_reason) + "），等待改派";
    out.push_back(ev);
  }

  if (!is_new && !task.robot.empty() && !prev_robot.empty() && task.robot != prev_robot &&
      IsActiveState(task.state))
  {
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = task.robot;
    ev.severity = Severity::kInfo;
    ev.text = task.task_id + " 改派给 " + task.robot;
    out.push_back(ev);
  }

  const auto announced = Announce(task.task_id, task.robot, task.state, task.failure_reason,
                                  std::string(), now_ms);
  out.insert(out.end(), announced.begin(), announced.end());
  return out;
}

std::vector<Event> MonitorModel::OnRobotStatus(const RobotView& robot, int64_t now_ms)
{
  std::vector<Event> out;
  if (robot.robot_id.empty()) {
    return out;
  }

  RobotView view = robot;
  view.online = true;
  view.heartbeat_seen = true;
  view.last_seen_ms = now_ms;
  ever_seen_.insert(view.robot_id);
  no_heartbeat_alarmed_.erase(view.robot_id);

  const auto it = robots_.find(view.robot_id);
  if (it == robots_.end()) {
    robots_[view.robot_id] = view;
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = view.robot_id;
    ev.text = view.robot_id + " 上线";
    out.push_back(ev);
    return out;
  }

  const RobotView prev = it->second;
  robots_[view.robot_id] = view;

  if (!prev.online) {
    const bool first_heartbeat = !prev.heartbeat_seen;
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = view.robot_id;
    ev.text = view.robot_id + (first_heartbeat ? " 上线（首帧心跳）" : " 恢复在线");
    out.push_back(ev);
  }
  if (prev.state != view.state || prev.current_task != view.current_task) {
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = view.robot_id;
    if (view.state == kRobotError) {
      ev.severity = Severity::kError;
      ev.text = view.robot_id + " 上报异常状态";
    } else if (view.state == kRobotBusy && !view.current_task.empty()) {
      ev.text = view.robot_id + " 开始执行 " + view.current_task;
    } else if (view.current_task.empty()) {
      ev.text = view.robot_id + " " + RobotStateName(view.state);
    } else {
      ev.text = view.robot_id + " " + RobotStateName(view.state) + "：" + view.current_task;
    }
    out.push_back(ev);
  }
  return out;
}

std::vector<Event> MonitorModel::OnTaskStatus(const StatusEvent& status, int64_t now_ms)
{
  std::vector<Event> out;
  if (status.task_id.empty()) {
    return out;
  }

  progress_[status.task_id] = status.progress;
  detail_[status.task_id] = status.detail;

  // 状态事件是增量流，比 2 Hz 的快照更及时；任务表先用它把状态推上去
  const auto it = tasks_.find(status.task_id);
  if (it != tasks_.end()) {
    it->second.state = status.state;
    if (!status.robot_id.empty()) {
      it->second.robot = status.robot_id;
    }
    if (!status.failure_reason.empty() && status.failure_reason != "NONE") {
      it->second.failure_reason = status.failure_reason;
    }
  }

  return Announce(status.task_id, status.robot_id, status.state, status.failure_reason,
                  status.detail, now_ms);
}

std::vector<Event> MonitorModel::Sweep(int64_t now_ms)
{
  std::vector<Event> out;
  for (auto& [id, robot] : robots_) {
    if (!robot.online) {
      // 登记了却一直没心跳：2 倍超时后给一次告警，避免"看得见但没人管"
      if (!ever_seen_.count(id) && !no_heartbeat_alarmed_.count(id) &&
          robot.last_seen_ms > 0 && now_ms - robot.last_seen_ms > 2 * offline_timeout_ms_)
      {
        no_heartbeat_alarmed_.insert(id);
        Event ev;
        ev.stamp_ms = now_ms;
        ev.robot_id = id;
        ev.severity = Severity::kWarn;
        ev.text = id + " 已登记但一直没收到心跳，检查执行代理是否启动";
        out.push_back(ev);
      }
      continue;
    }
    const int64_t gap = now_ms - robot.last_seen_ms;
    if (gap <= offline_timeout_ms_) {
      continue;
    }
    robot.online = false;
    Event ev;
    ev.stamp_ms = now_ms;
    ev.robot_id = id;
    ev.severity = Severity::kWarn;
    ev.text = id + " 心跳中断 " + FormatAge(gap) + "，判定离线";
    out.push_back(ev);
  }
  return out;
}

std::vector<TaskView> MonitorModel::SortedTasks() const
{
  std::vector<TaskView> out;
  out.reserve(tasks_.size());
  for (const auto& [id, task] : tasks_) {
    out.push_back(task);
  }
  std::sort(out.begin(), out.end(), [](const TaskView& a, const TaskView& b) {
    const int ra = RankOf(a.state);
    const int rb = RankOf(b.state);
    if (ra != rb) {
      return ra < rb;
    }
    if (a.detected_ms != b.detected_ms) {
      return a.detected_ms > b.detected_ms;   // 同组内新的在上
    }
    return a.task_id < b.task_id;
  });
  return out;
}

double MonitorModel::Progress(const std::string& task_id) const
{
  const auto it = progress_.find(task_id);
  return it == progress_.end() ? 0.0 : it->second;
}

std::string MonitorModel::Detail(const std::string& task_id) const
{
  const auto it = detail_.find(task_id);
  return it == detail_.end() ? std::string() : it->second;
}

int MonitorModel::ActiveCount() const
{
  int n = 0;
  for (const auto& [id, task] : tasks_) {
    if (IsActiveState(task.state)) {
      ++n;
    }
  }
  return n;
}

int MonitorModel::DoneCount() const
{
  int n = 0;
  for (const auto& [id, task] : tasks_) {
    if (task.state == kStateDone) {
      ++n;
    }
  }
  return n;
}

int MonitorModel::FailedCount() const
{
  int n = 0;
  for (const auto& [id, task] : tasks_) {
    if (task.state == kStateFailed) {
      ++n;
    }
  }
  return n;
}

}  // namespace fleet::monitor