#include "fleet_manager_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace fleet {
namespace {

constexpr const char* kReasonAssignTimeout = "ASSIGN_TIMEOUT";
constexpr const char* kReasonAgentFailed = "AGENT_FAILED";
constexpr const char* kReasonGoalRejected = "GOAL_REJECTED";
constexpr const char* kReasonUnknownTask = "UNKNOWN_TASK";

bool IsActiveState(TaskState s)
{
  return s == TaskState::kAssigned || s == TaskState::kNavigating ||
         s == TaskState::kDocking;
}

bool IsTerminalState(TaskState s)
{
  return s == TaskState::kDone || s == TaskState::kFailed ||
         s == TaskState::kCanceled;
}

Color ToCoreColor(uint8_t c)
{
  switch (c) {
    case 1: return Color::kRed;
    case 2: return Color::kGreen;
    case 3: return Color::kBlue;
    default: return Color::kUnknown;
  }
}

uint8_t ToMsgColor(Color c)
{
  return static_cast<uint8_t>(c);
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

builtin_interfaces::msg::Time ToMsgTime(TimeNs t)
{
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(t / kSecond);
  msg.nanosec = static_cast<uint32_t>(t % kSecond);
  return msg;
}

void SetMarkerColor(visualization_msgs::msg::Marker& m, Color c)
{
  switch (c) {
    case Color::kRed:   m.color.r = 0.9f; m.color.g = 0.1f; m.color.b = 0.1f; break;
    case Color::kGreen: m.color.r = 0.1f; m.color.g = 0.8f; m.color.b = 0.2f; break;
    case Color::kBlue:  m.color.r = 0.1f; m.color.g = 0.3f; m.color.b = 0.9f; break;
    default:            m.color.r = 0.7f; m.color.g = 0.7f; m.color.b = 0.7f; break;
  }
  m.color.a = 0.85f;
}

}  // namespace

FleetManagerNode::FleetManagerNode()
: Node("fleet_manager"),
  queue_([] {
    TaskQueueConfig cfg;
    cfg.dedup_radius = 0.8;          // 同一物块被两台车从不同视角估计，位置会差几十厘米
    cfg.default_max_retries = 1;
    return cfg;
  }()),
  assignment_([] {
    AssignConfig cfg;
    cfg.target_exclusion_radius = 1.0;
    return cfg;
  }()),
  watchdog_([] {
    WatchdogConfig cfg;
    // 默认值都略大于 agent 自身的超时，作为"最后一道兜底"，
    // 避免和 agent 抢着判失败；运行时可用参数调小，供故障注入使用
    // dock_timeout 是"从派单到对位结束"的总预算（fleet_core 的实现口径），
    // 必须大于 nav_timeout，否则导航稍慢一点就永远没机会进入对位
    cfg.nav_timeout = 100 * kSecond;
    cfg.dock_timeout = 150 * kSecond;
    cfg.assign_timeout = 30 * kSecond;
    cfg.heartbeat_timeout = 3 * kSecond;
    return cfg;
  }())
{
  const auto robot_ids = declare_parameter<std::vector<std::string>>(
    "robot_ids", std::vector<std::string>{"robot1", "robot2"});
  dispatch_ids_ = declare_parameter<std::vector<std::string>>(
    "dispatch_robot_ids", std::vector<std::string>{"robot1"});
  const double requested_standoff = declare_parameter<double>("standoff", 0.6);
  standoff_ = std::clamp(requested_standoff, kMinStandoff, kMaxStandoff);
  if (std::abs(standoff_ - requested_standoff) > 1e-9) {
    RCLCPP_WARN(get_logger(), "standoff 超出 [%.2f, %.2f]，已限制到 %.2f",
                kMinStandoff, kMaxStandoff, standoff_);
  }
  publish_markers_ = declare_parameter<bool>("publish_markers", true);

  // 终态任务保留策略：不清理会让内存和监控快照无限增长，
  // 也会让"同色目标再次出现"被历史任务挡住
  TaskQueueConfig queue_cfg = queue_.config();
  queue_cfg.retention_age = static_cast<TimeNs>(
    declare_parameter<double>("retention_age", 600.0) * kSecond);
  queue_cfg.max_terminal = static_cast<std::size_t>(
    declare_parameter<int>("retention_max_tasks", 500));
  queue_.set_config(queue_cfg);

  // 看门狗时限做成参数：正常运行时是兜底，故障注入/调参时可以在线缩短
  WatchdogConfig wd;
  wd.nav_timeout = static_cast<TimeNs>(declare_parameter<double>("nav_timeout", 100.0) * kSecond);
  wd.dock_timeout = static_cast<TimeNs>(declare_parameter<double>("dock_timeout", 150.0) * kSecond);
  wd.assign_timeout =
    static_cast<TimeNs>(declare_parameter<double>("assign_timeout", 30.0) * kSecond);
  wd.heartbeat_timeout =
    static_cast<TimeNs>(declare_parameter<double>("heartbeat_timeout", 3.0) * kSecond);
  watchdog_cfg_ = wd;
  watchdog_ = Watchdog(wd);
  cooldown_ns_ = static_cast<TimeNs>(
    declare_parameter<double>("retry_cooldown", 10.0) * kSecond);

  // 允许在线调整超时：故障注入要能把"导航卡死多久算超时"缩短到十几秒，
  // 否则一次注入得等 100 s 才看得到看门狗动作
  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      WatchdogConfig cfg = watchdog_cfg_;
      for (const auto& p : params) {
        const auto name = p.get_name();
        if (name == "nav_timeout") {
          cfg.nav_timeout = static_cast<TimeNs>(p.as_double() * kSecond);
        } else if (name == "dock_timeout") {
          cfg.dock_timeout = static_cast<TimeNs>(p.as_double() * kSecond);
        } else if (name == "assign_timeout") {
          cfg.assign_timeout = static_cast<TimeNs>(p.as_double() * kSecond);
        } else if (name == "heartbeat_timeout") {
          cfg.heartbeat_timeout = static_cast<TimeNs>(p.as_double() * kSecond);
        } else if (name == "retry_cooldown") {
          cooldown_ns_ = static_cast<TimeNs>(p.as_double() * kSecond);
        }
      }
      watchdog_cfg_ = cfg;
      watchdog_ = Watchdog(cfg);
      return result;
    });

  client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  for (const auto& id : robot_ids) {
    // 未收到心跳前不派活：状态先置 OFFLINE，第一次心跳会把它拉回 IDLE
    Robot r;
    r.id = id;
    r.state = RobotState::kOffline;
    robots_[id] = r;

    auto opts = rclcpp::SubscriptionOptions{};
    opts.callback_group = client_group_;

    clients_[id] = rclcpp_action::create_client<ExecuteTask>(this, "/" + id + "/execute_task");

    robot_status_subs_.push_back(create_subscription<fleet_msgs::msg::RobotStatus>(
      "/" + id + "/robot_status", rclcpp::QoS(10),
      [this](fleet_msgs::msg::RobotStatus::SharedPtr msg) { OnRobotStatus(*msg); }, opts));
    task_status_subs_.push_back(create_subscription<fleet_msgs::msg::TaskStatus>(
      "/" + id + "/task_status", rclcpp::QoS(20),
      [this](fleet_msgs::msg::TaskStatus::SharedPtr msg) { OnTaskStatus(*msg); }, opts));
    detection_subs_.push_back(create_subscription<fleet_msgs::msg::Detection>(
      "/" + id + "/detections", rclcpp::QoS(20),
      [this](fleet_msgs::msg::Detection::SharedPtr msg) { OnDetection(*msg); }, opts));
  }

  task_pub_ = create_publisher<fleet_msgs::msg::Task>("/fleet/tasks", rclcpp::QoS(20));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/fleet/task_markers", rclcpp::QoS(10));

  submit_srv_ = create_service<fleet_msgs::srv::SubmitTask>(
    "/fleet/submit_task",
    [this](const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> req,
           std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> res) { OnSubmit(req, res); });
  cancel_srv_ = create_service<fleet_msgs::srv::CancelTask>(
    "/fleet/cancel_task",
    [this](const std::shared_ptr<fleet_msgs::srv::CancelTask::Request> req,
           std::shared_ptr<fleet_msgs::srv::CancelTask::Response> res) { OnCancel(req, res); });

  // 0.5 s 一拍：既做超时判定，也做待分配任务的推进
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { Tick(); });

  RCLCPP_INFO(get_logger(), "调度中心就绪: 车辆 %zu 台，去重半径 0.8m，最大重试 1 次",
              robot_ids.size());
}

// ---------------- 事件接入 ----------------

void FleetManagerNode::OnDetection(const fleet_msgs::msg::Detection& msg)
{
  if (msg.lost || msg.range <= 0.0f) {
    return;   // 丢失事件由 agent 的失败原因体现，这里不据此撤任务
  }
  const Color color = ToCoreColor(msg.color);
  if (color == Color::kUnknown) {
    return;
  }

  const Pose2D target{msg.world_pose.x, msg.world_pose.y, 0.0};
  if (queue_.FindDuplicate(color, target).has_value()) {
    return;   // 同色同位置已有任务（含已完成），不重复建单
  }

  const TaskId id = "T" + std::to_string(++task_seq_);
  if (!queue_.AddTask(id, color, target, NowNs())) {
    return;
  }
  const Task* t = queue_.Find(id);
  RCLCPP_INFO(get_logger(), "发现目标 %s: %s @(%.2f, %.2f)，%s 报告",
              id.c_str(), ToString(color), target.x, target.y, msg.source_robot_id.c_str());
  if (t != nullptr) {
    PublishTaskSnapshot();
  }
}

void FleetManagerNode::OnTaskStatus(const fleet_msgs::msg::TaskStatus& msg)
{
  const Task* t = queue_.Find(msg.task_id);
  if (t == nullptr) {
    return;
  }
  // 只接受"当前归属车"的状态上报。故障恢复后，被冻结又恢复的旧车会把
  // 迟到的 NAVIGATING/DONE 发上来，若不校验归属就会把已改派的任务改乱
  if (t->assigned_robot.empty() || t->assigned_robot != msg.robot_id) {
    RCLCPP_DEBUG(get_logger(), "忽略 %s 对任务 %s 的迟到状态 %u（归属 %s）",
                 msg.robot_id.c_str(), msg.task_id.c_str(), msg.state,
                 t->assigned_robot.c_str());
    return;
  }
  const TimeNs now = NowNs();

  switch (msg.state) {
    case fleet_msgs::msg::TaskStatus::STATE_ASSIGNED:
    case fleet_msgs::msg::TaskStatus::STATE_NAVIGATING:
    case fleet_msgs::msg::TaskStatus::STATE_DOCKING: {
      const TaskState to = static_cast<TaskState>(msg.state);
      if (t->state != to) {
        queue_.Transition(msg.task_id, to, now);
      }
      break;
    }
    case fleet_msgs::msg::TaskStatus::STATE_DONE:
      // 只处理仍处于执行中的任务；重试后的 PENDING 收到迟到的 DONE 要忽略
      if (IsActiveState(t->state) && queue_.Transition(msg.task_id, TaskState::kDone, now)) {
        RCLCPP_INFO(get_logger(), "任务 %s 完成", msg.task_id.c_str());
        ReleaseRobot(msg.robot_id);
      }
      break;
    case fleet_msgs::msg::TaskStatus::STATE_FAILED: {
      if (!IsActiveState(t->state)) {
        break;
      }
      const std::string reason = msg.failure_reason.empty() ? kReasonAgentFailed
                                                            : msg.failure_reason;
      const bool retried = queue_.RetryOrFail(msg.task_id, now, reason);
      MarkCooldown(msg.robot_id);
      RCLCPP_WARN(get_logger(), "任务 %s 失败(%s)，%s", msg.task_id.c_str(), reason.c_str(),
                  retried ? "已重排等待改派" : "重试配额用尽，置为 FAILED");
      ReleaseRobot(msg.robot_id);
      break;
    }
    case fleet_msgs::msg::TaskStatus::STATE_CANCELED: {
      if (!IsTerminalState(t->state)) {
        queue_.Cancel(msg.task_id, now, msg.failure_reason.empty()
                                         ? "CANCELED_BY_OPERATOR" : msg.failure_reason);
      }
      ReleaseRobot(msg.robot_id);
      break;
    }
    default:
      break;
  }
}

void FleetManagerNode::OnRobotStatus(const fleet_msgs::msg::RobotStatus& msg)
{
  auto it = robots_.find(msg.robot_id);
  if (it == robots_.end()) {
    return;
  }
  Robot& r = it->second;
  const bool was_offline = r.state == RobotState::kOffline;
  const bool had_heartbeat = r.last_heartbeat != 0;
  r.pose = Pose2D{msg.pose.position.x, msg.pose.position.y,
                  YawFromQuaternion(msg.pose.orientation)};
  r.last_heartbeat = NowNs();
  r.heartbeat_seq = msg.heartbeat_seq;
  r.tasks_done = msg.tasks_done;
  r.tasks_failed = msg.tasks_failed;

  const auto busy_it = robot_task_.find(r.id);
  const bool executing = busy_it != robot_task_.end() && !busy_it->second.empty();

  if (msg.state == fleet_msgs::msg::RobotStatus::STATE_ERROR) {
    r.state = RobotState::kError;
  } else if (executing) {
    // 本中心还在等该任务的结果时，不信心跳的 IDLE —— 防止一次错拍导致重复派活
    r.state = RobotState::kBusy;
  } else {
    r.state = RobotState::kIdle;
  }

  // 从离线恢复：撤销该车上可能残留的旧目标。
  // 进程被冻住期间收到的 action 目标，恢复后仍会继续执行 —— 如果那个任务
  // 已经改派给别的车，这辆车就会去抢同一个物块（实测发生过）。
  if (was_offline && had_heartbeat) {
    auto client_it = clients_.find(r.id);
    if (client_it != clients_.end()) {
      client_it->second->async_cancel_all_goals();
      RCLCPP_INFO(get_logger(), "机器人 %s 恢复上线，已撤销其残留的旧目标", r.id.c_str());
    }
  }
}

void FleetManagerNode::OnSubmit(
  const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> req,
  std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> res)
{
  const Color color = ToCoreColor(req->target_color);
  if (color == Color::kUnknown) {
    res->accepted = false;
    res->message = "target_color 必须是 1/2/3";
    return;
  }

  const TaskId id = "S" + std::to_string(++task_seq_);
  const Pose2D target{req->target_pose.position.x, req->target_pose.position.y, 0.0};
  const bool ok = queue_.AddTask(id, color, target, NowNs());
  res->accepted = ok;
  res->task_id = ok ? id : "";
  res->message = ok ? "任务已创建" : "同色且位置过近的任务已存在";
  if (ok) {
    RCLCPP_INFO(get_logger(), "外部注入任务 %s: %s @(%.2f, %.2f)", id.c_str(),
                ToString(color), target.x, target.y);
    PublishTaskSnapshot();
  }
}

void FleetManagerNode::OnCancel(
  const std::shared_ptr<fleet_msgs::srv::CancelTask::Request> req,
  std::shared_ptr<fleet_msgs::srv::CancelTask::Response> res)
{
  const std::string reason = req->reason.empty() ? "CANCELED_BY_OPERATOR" : req->reason;
  const RobotId* robot = RobotOfTask(req->task_id);
  const bool ok = queue_.Cancel(req->task_id, NowNs(), reason);

  if (ok && robot != nullptr) {
    CancelGoal(*robot);
    ReleaseRobot(*robot);
  }

  res->canceled = ok;
  res->message = ok ? "任务已取消" : "任务不存在或已处于终态";
  if (ok) {
    RCLCPP_INFO(get_logger(), "任务 %s 已取消: %s", req->task_id.c_str(), reason.c_str());
    PublishTaskSnapshot();
  }
}

// ---------------- 周期推进 ----------------

void FleetManagerNode::Tick()
{
  const TimeNs now = NowNs();
  const auto pruned = queue_.Prune(now);
  if (!pruned.empty()) {
    RCLCPP_INFO(get_logger(), "清理 %zu 个超过保留策略的终态任务", pruned.size());
  }
  HandleWatchdog(now);
  AssignPending(now);
  PublishTaskSnapshot();
  if (publish_markers_) {
    PublishMarkers();
  }
}

void FleetManagerNode::HandleWatchdog(TimeNs now)
{
  const auto events = watchdog_.Check(queue_.All(), RobotList(), now);
  for (const auto& ev : events) {
    if (ev.kind == TimeoutKind::kRobotOffline) {
      auto& robot = robots_[ev.robot_id];
      if (robot.state != RobotState::kOffline) {
        RCLCPP_WARN(get_logger(), "机器人 %s 心跳超时，标记离线", ev.robot_id.c_str());
        robot.state = RobotState::kOffline;
      }
      // 用看门狗给的 reason（ROBOT_UNREACHABLE），与 TaskStatus 的失败原因枚举保持一致
      FailActiveTask(ev.robot_id, ev.reason);
      continue;
    }

    const Task* t = queue_.Find(ev.task_id);
    if (t == nullptr) {
      continue;
    }
    if (ev.kind == TimeoutKind::kAssign) {
      // 长时间没有车能接：直接判失败（PENDING->FAILED 是合法迁移），
      // 重试对它没有意义 —— 再排一次仍然没车可派
      if (queue_.Transition(ev.task_id, TaskState::kFailed, now, kReasonAssignTimeout)) {
        RCLCPP_WARN(get_logger(), "任务 %s 等待分配超时，置为 FAILED", ev.task_id.c_str());
      }
      continue;
    }

    const RobotId robot = t->assigned_robot;
    if (!robot.empty()) {
      CancelGoal(robot);
    }
    const bool retried = queue_.RetryOrFail(ev.task_id, now, ev.reason);
    RCLCPP_WARN(get_logger(), "任务 %s 超时(%s, 超 %.0fs)，%s", ev.task_id.c_str(),
                ev.reason.c_str(), static_cast<double>(ev.overdue) / kSecond,
                retried ? "已重排" : "重试配额用尽");
    if (!robot.empty()) {
      MarkCooldown(robot);
      ReleaseRobot(robot, robots_[robot].state == RobotState::kOffline);
    }
  }
}

void FleetManagerNode::AssignPending(TimeNs now)
{
  for (const auto& task : queue_.ByState(TaskState::kPending)) {
    const auto idle = IdleRobots();
    if (idle.empty()) {
      break;
    }

    const AssignDecision decision =
      assignment_.SelectRobot(task.target, idle, queue_.ActiveTargets());
    if (!decision.HasRobot()) {
      continue;   // 目标被占用或暂时无车可用，下一拍再试
    }

    const auto client_it = clients_.find(decision.robot);
    if (client_it == clients_.end() || !client_it->second->action_server_is_ready()) {
      continue;   // agent 还没上线，保持 PENDING
    }

    if (!queue_.AssignTo(task.id, decision.robot, now)) {
      continue;
    }
    robot_task_[decision.robot] = task.id;
    robots_[decision.robot].state = RobotState::kBusy;

    const Task* assigned = queue_.Find(task.id);
    if (assigned == nullptr || !SendGoal(decision.robot, *assigned)) {
      queue_.RetryOrFail(task.id, now, kReasonGoalRejected);
      ReleaseRobot(decision.robot);
      continue;
    }
    RCLCPP_INFO(get_logger(), "任务 %s 派给 %s", task.id.c_str(), decision.robot.c_str());
  }
}

// ---------------- action ----------------

bool FleetManagerNode::SendGoal(const RobotId& robot, const Task& task)
{
  const auto it = clients_.find(robot);
  if (it == clients_.end()) {
    return false;
  }

  ExecuteTask::Goal goal;
  goal.task_id = task.id;
  goal.target_color = ToMsgColor(task.color);
  goal.target_pose.position.x = task.target.x;
  goal.target_pose.position.y = task.target.y;
  goal.target_pose.position.z = 0.0;
  goal.target_pose.orientation = QuaternionFromYaw(task.target.yaw);
  goal.standoff = static_cast<float>(standoff_);

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
  const TaskId task_id = task.id;
  options.goal_response_callback =
    [this, robot, task_id](const GoalHandle::SharedPtr& handle) {
      OnGoalResponse(robot, task_id, handle);
    };
  options.result_callback =
    [this, robot, task_id](const GoalHandle::WrappedResult& wrapped) {
      OnActionResult(robot, task_id, wrapped);
    };

  it->second->async_send_goal(goal, options);
  return true;
}

void FleetManagerNode::OnGoalResponse(const RobotId& robot, const TaskId& task_id,
                                      const GoalHandle::SharedPtr& handle)
{
  // 归属校验：故障恢复后旧车可能补发 goal 响应，此时任务早已改派
  const auto owned = robot_task_.find(robot);
  if (owned == robot_task_.end() || owned->second != task_id) {
    RCLCPP_DEBUG(get_logger(), "忽略 %s 对任务 %s 的迟到 goal 响应", robot.c_str(),
                 task_id.c_str());
    return;
  }

  if (handle != nullptr) {
    active_goals_[robot] = handle;
    return;
  }

  // 被拒绝：多半是该车正忙（并发下发的保护），重排等下一拍
  const Task* t = queue_.Find(task_id);
  if (t != nullptr && IsActiveState(t->state)) {
    queue_.RetryOrFail(task_id, NowNs(), kReasonGoalRejected);
  }
  RCLCPP_WARN(get_logger(), "任务 %s 被 %s 拒绝", task_id.c_str(), robot.c_str());
  ReleaseRobot(robot);
}

void FleetManagerNode::OnActionResult(const RobotId& robot, const TaskId& task_id,
                                      const GoalHandle::WrappedResult& wrapped)
{
  // 只有"当前归属车"的结果才算数：被冻结后恢复的旧车会补发旧任务的失败结果，
  // 若不校验就会把已改派、正在别的车上执行的任务误判为失败（实测踩过）
  const auto owned = robot_task_.find(robot);
  if (owned == robot_task_.end() || owned->second != task_id) {
    RCLCPP_DEBUG(get_logger(), "忽略 %s 对任务 %s 的迟到结果", robot.c_str(),
                 task_id.c_str());
    return;
  }
  active_goals_.erase(robot);

  const Task* t = queue_.Find(task_id);
  if (t == nullptr) {
    ReleaseRobot(robot);
    return;
  }
  // task_status 可能已经把终态处理过了，这里只兜底"结果先到"的情况
  if (!IsActiveState(t->state)) {
    ReleaseRobot(robot);
    return;
  }

  const TimeNs now = NowNs();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result != nullptr &&
      wrapped.result->success) {
    if (queue_.Transition(task_id, TaskState::kDone, now)) {
      RCLCPP_INFO(get_logger(), "任务 %s 由 %s 完成", task_id.c_str(), robot.c_str());
    }
  } else {
    std::string reason = wrapped.result != nullptr ? wrapped.result->failure_reason
                                                   : kReasonAgentFailed;
    if (reason.empty() || reason == "NONE") {
      reason = kReasonAgentFailed;
    }
    const bool retried = queue_.RetryOrFail(task_id, now, reason);
    RCLCPP_WARN(get_logger(), "任务 %s 由 %s 执行失败(%s)，%s", task_id.c_str(),
                robot.c_str(), reason.c_str(), retried ? "已重排" : "置为 FAILED");
    MarkCooldown(robot);   // 只设一次：失败车冷却，避免重试又派回同一台
  }
  ReleaseRobot(robot);
}

void FleetManagerNode::CancelGoal(const RobotId& robot)
{
  const auto it = active_goals_.find(robot);
  if (it != active_goals_.end() && it->second != nullptr) {
    clients_[robot]->async_cancel_goal(it->second);
    active_goals_.erase(it);
  }
}

// ---------------- 状态工具 ----------------

void FleetManagerNode::ReleaseRobot(const RobotId& robot, bool offline)
{
  const auto it = robot_task_.find(robot);
  if (it != robot_task_.end()) {
    robot_task_.erase(it);
  }
  const auto goal_it = active_goals_.find(robot);
  if (goal_it != active_goals_.end()) {
    active_goals_.erase(goal_it);
  }

  auto robot_it = robots_.find(robot);
  if (robot_it != robots_.end()) {
    robot_it->second.state = offline ? RobotState::kOffline : RobotState::kIdle;
  }
}

void FleetManagerNode::FailActiveTask(const RobotId& robot, const std::string& reason)
{
  const auto it = robot_task_.find(robot);
  if (it == robot_task_.end() || it->second.empty()) {
    return;
  }
  const TaskId task_id = it->second;
  CancelGoal(robot);
  const bool retried = queue_.RetryOrFail(task_id, NowNs(), reason);
  RCLCPP_WARN(get_logger(), "机器人 %s 上的任务 %s 中止(%s)，%s", robot.c_str(),
              task_id.c_str(), reason.c_str(), retried ? "已重排" : "置为 FAILED");
  const auto goal_it = active_goals_.find(robot);
  if (goal_it != active_goals_.end()) {
    active_goals_.erase(goal_it);
  }
  const auto task_it = robot_task_.find(robot);
  if (task_it != robot_task_.end()) {
    robot_task_.erase(task_it);
  }
}

void FleetManagerNode::PublishTaskSnapshot()
{
  for (const auto& task : queue_.All()) {
    fleet_msgs::msg::Task msg;
    msg.task_id = task.id;
    msg.type = fleet_msgs::msg::Task::TYPE_APPROACH;
    msg.target_color = ToMsgColor(task.color);
    msg.state = static_cast<uint8_t>(task.state);
    msg.target_pose.position.x = task.target.x;
    msg.target_pose.position.y = task.target.y;
    msg.target_pose.position.z = 0.0;
    msg.target_pose.orientation = QuaternionFromYaw(task.target.yaw);
    msg.assigned_robot_id = task.assigned_robot;
    msg.revision = task.revision;
    msg.detected_at = ToMsgTime(task.detected_at);
    msg.assigned_at = ToMsgTime(task.assigned_at);
    msg.retry_count = task.retry_count;
    msg.max_retries = task.max_retries;
    msg.failure_reason = task.failure_reason.empty() ? "NONE" : task.failure_reason;
    task_pub_->publish(msg);
  }
}

void FleetManagerNode::PublishMarkers()
{
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = now();

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  int id = 0;
  for (const auto& task : queue_.All()) {
    if (IsTerminalState(task.state)) {
      continue;   // 终态不在 RViz 里长期堆着
    }
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = "map";
    cube.header.stamp = stamp;
    cube.ns = "fleet_tasks";
    cube.id = id;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.position.x = task.target.x;
    cube.pose.position.y = task.target.y;
    cube.pose.position.z = 0.12;
    cube.scale.x = 0.22;
    cube.scale.y = 0.22;
    cube.scale.z = 0.22;
    SetMarkerColor(cube, task.color);
    array.markers.push_back(cube);

    visualization_msgs::msg::Marker text;
    text.header = cube.header;
    text.ns = "fleet_task_labels";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = task.target.x;
    text.pose.position.y = task.target.y;
    text.pose.position.z = 0.5;
    text.scale.z = 0.14;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 0.9f;
    text.text = task.id + " " + ToString(task.state);
    if (!task.assigned_robot.empty()) {
      text.text += " @" + task.assigned_robot;
    }
    array.markers.push_back(text);
    ++id;
  }

  marker_pub_->publish(array);
}

std::vector<Robot> FleetManagerNode::RobotList() const
{
  std::vector<Robot> out;
  out.reserve(robots_.size());
  for (const auto& kv : robots_) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<Robot> FleetManagerNode::IdleRobots() const
{
  std::vector<Robot> out;
  for (const auto& kv : robots_) {
    if (kv.second.state != RobotState::kIdle) {
      continue;
    }
    const bool dispatchable =
      std::find(dispatch_ids_.begin(), dispatch_ids_.end(), kv.first) != dispatch_ids_.end();
    const auto cool = cooldown_until_.find(kv.first);
    const bool cooling = cool != cooldown_until_.end() && cool->second > NowNs();
    if (dispatchable && !cooling) {
      out.push_back(kv.second);
    }
  }
  return out;
}

void FleetManagerNode::MarkCooldown(const RobotId& robot)
{
  if (cooldown_ns_ <= 0) {
    return;
  }
  cooldown_until_[robot] = NowNs() + cooldown_ns_;
  RCLCPP_INFO(get_logger(), "机器人 %s 进入 %.0fs 冷却，期间不接新任务",
              robot.c_str(), static_cast<double>(cooldown_ns_) / kSecond);
}

const RobotId* FleetManagerNode::RobotOfTask(const TaskId& id) const
{
  for (const auto& kv : robot_task_) {
    if (kv.second == id) {
      return &kv.first;
    }
  }
  return nullptr;
}

TimeNs FleetManagerNode::NowNs() const
{
  return get_clock()->now().nanoseconds();
}

}  // namespace fleet

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fleet::FleetManagerNode>();
  // 单线程：内核契约"单线程推进"，本节点所有回调均为非阻塞
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
