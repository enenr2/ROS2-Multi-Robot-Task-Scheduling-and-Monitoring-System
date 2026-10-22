// 机器人执行代理：一台车一个节点。
//
// 职责边界刻意收窄：只负责"把这一辆车开到目标前并停稳"，不做任务分配、
// 不做去重、不做重试 —— 那些属于调度层（M6）。这样这个节点可以被单机
// 直接驱动，也能在 M6 里被调度器按车直调，天然互斥。
//
// 一次任务三段：
//   1. 导航：把目标点换算成"退后 approach_distance 且车头对准目标"的终点，
//      交给 Nav2。Nav2 负责全局路径与避障，我们只等结果。
//   2. 对位：目标已进视野，用视觉闭环把车头转正、距离收到 standoff。
//   3. 收尾：停车、发终态。
//
// 线程模型：action 的 execute 回调会阻塞等待导航结果与对位循环，所以用
// MultiThreadedExecutor，并把订阅/定时器放在独立的 callback group，
// 否则回调饿死、反馈发不出去。
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "fleet_agent/agent_geometry.hpp"
#include "fleet_agent/docking_controller.hpp"
#include "fleet_manager/task_types.hpp"
#include "fleet_msgs/action/execute_task.hpp"
#include "fleet_msgs/msg/detection.hpp"
#include "fleet_msgs/msg/robot_status.hpp"
#include "fleet_msgs/msg/task_status.hpp"

namespace fleet {
namespace {

using ExecuteTask = fleet_msgs::action::ExecuteTask;
using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteTask>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// 失败原因枚举，与 TaskStatus.failure_reason 的注释保持一致
constexpr const char* kReasonNone = "NONE";
constexpr const char* kReasonNavTimeout = "NAV_TIMEOUT";
constexpr const char* kReasonNavAborted = "NAV_ABORTED";
constexpr const char* kReasonTargetLost = "TARGET_LOST";
constexpr const char* kReasonDockTimeout = "DOCK_TIMEOUT";
constexpr const char* kReasonUnreachable = "ROBOT_UNREACHABLE";
constexpr const char* kReasonCanceled = "CANCELED_BY_OPERATOR";

// 到达对位点后没看到目标时的搜索参数：
// 先 0.12 m/s 倒车 4 s（退出"贴脸"盲区），再 0.45 rad/s 转一圈（约 14 s）
constexpr double kSearchYawRate = 0.45;
constexpr double kSearchBudgetSec = 18.0;
constexpr double kSearchBackoffVel = 0.12;
constexpr double kSearchBackoffSec = 4.0;

double Clamp01(double v)
{
  return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
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

uint8_t ToMsgState(TaskState s)
{
  return static_cast<uint8_t>(s);
}

}  // namespace

class RobotAgent : public rclcpp::Node
{
public:
  RobotAgent()
  : Node("robot_agent")
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "");
    if (robot_id_.empty()) {
      const std::string ns = get_namespace();
      robot_id_ = ns.size() > 1 ? ns.substr(1) : "robot";
    }

    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    const double standoff = declare_parameter<double>("standoff", 0.6);
    approach_distance_ = standoff + declare_parameter<double>("pre_dock_margin", 0.8);
    nav_timeout_s_ = declare_parameter<double>("nav_timeout", 90.0);
    const auto detection_stale_ms = declare_parameter<int>("detection_stale_ms", 1000);

    DockingConfig dock_cfg;
    dock_cfg.standoff = standoff;
    dock_cfg.distance_tolerance = declare_parameter<double>("distance_tolerance", 0.10);
    dock_cfg.bearing_tolerance = declare_parameter<double>("bearing_tolerance", 0.05);
    dock_cfg.max_linear_vel = declare_parameter<double>("max_linear_vel", 0.15);
    dock_cfg.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.5);
    dock_cfg.target_lost_timeout_s = declare_parameter<double>("target_lost_timeout", 3.0);
    dock_cfg.timeout_s = declare_parameter<double>("dock_timeout", 20.0);
    dock_ = std::make_unique<DockingController>(dock_cfg);

    detection_stale_s_ = static_cast<double>(detection_stale_ms) / 1000.0;

    // 订阅与定时器放独立组：execute 回调阻塞时它们仍要能被调度
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto sensor_opts = rclcpp::SubscriptionOptions{};
    auto opts = sensor_opts;
    opts.callback_group = sensor_group_;

    // 仿真里图像/里程计是传感器流，best effort 兼容性更好
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnOdom(*msg); }, opts);

    detection_sub_ = create_subscription<fleet_msgs::msg::Detection>(
      "detections", rclcpp::QoS(20),
      [this](fleet_msgs::msg::Detection::SharedPtr msg) { OnDetection(*msg); }, opts);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    task_status_pub_ = create_publisher<fleet_msgs::msg::TaskStatus>(
      "task_status", rclcpp::QoS(20));
    robot_status_pub_ = create_publisher<fleet_msgs::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 必须是 Reentrant：execute 回调会阻塞几十秒，若用 MutuallyExclusive，
    // 新任务请求会排在它后面，直到当前任务结束才被处理 —— 表现出来就是
    // "并发下发时第二个任务没被立即拒绝，而是悄悄排到了后面"（实测踩过）。
    // 并发执行由 busy_ 的原子交换挡住，组类型只负责让请求能被及时处理。
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this, "execute_task",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTask::Goal> goal) {
        return HandleGoal(*goal);
      },
      [this](const std::shared_ptr<GoalHandleExecute>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleExecute> handle) { Execute(handle); },
      rcl_action_server_get_default_options(),
      action_group_);

    heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { PublishRobotStatus(); }, sensor_group_);

    RCLCPP_INFO(get_logger(),
                "执行代理就绪: robot=%s 对位停车距离=%.2fm 导航终点退让=%.2fm "
                "action=/%s/execute_task",
                robot_id_.c_str(), dock_->config().standoff, approach_distance_,
                robot_id_.c_str());
  }

private:
  // ---------------- 回调 ----------------

  void OnOdom(const nav_msgs::msg::Odometry& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_.x = msg.pose.pose.position.x;
    pose_.y = msg.pose.pose.position.y;
    pose_.yaw = YawFromQuaternion(msg.pose.pose.orientation);
    has_pose_ = true;
  }

  void OnDetection(const fleet_msgs::msg::Detection& msg)
  {
    if (msg.color == 0 || msg.color > 3) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = detections_[msg.color];
    if (msg.lost || msg.range <= 0.0f) {
      slot.has = false;
      return;
    }
    slot.has = true;
    slot.bearing = msg.bearing;
    slot.range = msg.range;
    slot.stamp_ns = NowNs();
  }

  rclcpp_action::GoalResponse HandleGoal(const ExecuteTask::Goal& goal)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "拒绝任务 %s：本车正在执行其他任务",
                  goal.task_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "接受任务 %s 目标色=%u", goal.task_id.c_str(), goal.target_color);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // ---------------- 任务主体 ----------------

  void Execute(const std::shared_ptr<GoalHandleExecute> handle)
  {
    const auto goal = handle->get_goal();
    task_id_ = goal->task_id;
    const uint8_t color = goal->target_color;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_task_ = task_id_;   // 心跳线程读它，必须走锁
    }

    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = false;
    result->failure_reason = kReasonNone;

    PublishTaskStatus(TaskState::kAssigned, 0.0, "任务已接受");
    // 状态的第一个点也必须出现在 feedback 里，否则只靠 topic 才能看到 ASSIGNED
    PublishTaskFeedback(handle, TaskState::kAssigned, -1.0f, 0.0f, "任务已接受");

    // ---- 起点检查 ----
    Pose2D robot_pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pose_) {
        Finish(handle, result, false, kReasonUnreachable, "收不到里程计，无法定位");
        return;
      }
      robot_pose = pose_;
    }

    Pose2D target;
    target.x = goal->target_pose.position.x;
    target.y = goal->target_pose.position.y;
    target.yaw = YawFromQuaternion(goal->target_pose.orientation);

    const ApproachResult approach = ComputeApproach(robot_pose, target, approach_distance_);

    if (!approach.already_close) {
      if (!Navigate(handle, approach)) {
        Finish(handle, result, false, failure_reason_, failure_detail_);
        return;
      }
    } else {
      RCLCPP_INFO(get_logger(), "距目标 %.2fm，已进入接近距离，跳过导航直接对位",
                  approach.distance_to_target);
      PublishTaskStatus(TaskState::kNavigating, 0.6, "目标已在近处，直接进入对位");
    }

    const bool docked = Dock(handle, color, result);
    if (docked) {
      Finish(handle, result, true, kReasonNone, "任务完成");
    } else {
      Finish(handle, result, false, failure_reason_, failure_detail_);
    }
  }

  bool Navigate(const std::shared_ptr<GoalHandleExecute>& handle, const ApproachResult& approach)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      failure_reason_ = kReasonUnreachable;
      failure_detail_ = "Nav2 navigate_to_pose 不可用";
      return false;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose.header.frame_id = world_frame_;
    nav_goal.pose.header.stamp = now();
    nav_goal.pose.pose.position.x = approach.pose.x;
    nav_goal.pose.pose.position.y = approach.pose.y;
    nav_goal.pose.pose.orientation = QuaternionFromYaw(approach.pose.yaw);

    auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        std::lock_guard<std::mutex> lock(mutex_);
        nav_distance_remaining_ = fb->distance_remaining;
      };

    auto goal_future = nav_client_->async_send_goal(nav_goal, goal_options);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      failure_reason_ = kReasonUnreachable;
      failure_detail_ = "Nav2 未响应导航目标";
      return false;
    }
    auto nav_handle = goal_future.get();
    if (!nav_handle) {
      failure_reason_ = kReasonNavAborted;
      failure_detail_ = "Nav2 拒绝了导航目标";
      return false;
    }

    PublishTaskStatus(TaskState::kNavigating, 0.1, "导航前往目标附近");

    const double start_distance = std::max(approach.distance_to_target - approach_distance_, 0.0);
    const rclcpp::Time nav_start = now();
    auto result_future = nav_client_->async_get_result(nav_handle);

    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (handle->is_canceling()) {
        nav_client_->async_cancel_goal(nav_handle);
        failure_reason_ = kReasonCanceled;
        failure_detail_ = "导航中被取消";
        return false;
      }
      const double elapsed = (now() - nav_start).seconds();
      if (elapsed > nav_timeout_s_) {
        nav_client_->async_cancel_goal(nav_handle);
        failure_reason_ = kReasonNavTimeout;
        failure_detail_ = "导航超时（" + std::to_string(static_cast<int>(nav_timeout_s_)) + "s）";
        return false;
      }

      double remaining = -1.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        remaining = nav_distance_remaining_;
      }
      // 进度：导航占总进度的 0~60%
      double progress = 0.1;
      if (remaining >= 0.0 && start_distance > 0.05) {
        progress = 0.1 + 0.5 * Clamp01(1.0 - remaining / start_distance);
      }
      PublishTaskFeedback(handle, TaskState::kNavigating, static_cast<float>(remaining),
                          static_cast<float>(progress), "导航中");
    }

    const auto nav_result = result_future.get();
    if (nav_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      failure_reason_ = nav_result.code == rclcpp_action::ResultCode::CANCELED
                          ? kReasonCanceled : kReasonNavAborted;
      failure_detail_ = nav_result.code == rclcpp_action::ResultCode::CANCELED
                          ? "导航被取消" : "Nav2 返回失败";
      return false;
    }

    PublishTaskStatus(TaskState::kNavigating, 0.6, "已到达目标附近，开始视觉对位");
    return true;
  }

  bool Dock(const std::shared_ptr<GoalHandleExecute>& handle, uint8_t color,
            const std::shared_ptr<ExecuteTask::Result>& result)
  {
    dock_->Reset(NowNs());
    PublishTaskStatus(TaskState::kDocking, 0.6, "视觉对位中");

    double start_range = -1.0;
    bool ever_seen = false;
    const auto period = std::chrono::milliseconds(100);
    const rclcpp::Time dock_start = now();
    // 进入对位前的检测不算数：导航途中看到过目标，不代表到了对位点还能看到
    const TimeNs dock_start_ns = NowNs();

    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        StopRobot();
        failure_reason_ = kReasonCanceled;
        failure_detail_ = "对位中被取消";
        return false;
      }

      DockingInput in;
      bool fresh_since_dock = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& slot = detections_[color];
        const double age = static_cast<double>(NowNs() - slot.stamp_ns) / static_cast<double>(kSec);
        in.target_visible = slot.has && slot.stamp_ns > 0 && age <= detection_stale_s_;
        in.bearing = slot.bearing;
        in.range = slot.range;
        fresh_since_dock = in.target_visible && slot.stamp_ns > dock_start_ns;
      }
      ever_seen = ever_seen || fresh_since_dock;

      // 到了预定位置却始终没看到目标：多半是任务改派时沿用了上一台车报的
      // 世界坐标，两台车的里程计漂移不同，落点会差出半米到一米。原地转一圈
      // 把目标找回来再对位，比直接判 TARGET_LOST 更符合实际系统
      if (!in.target_visible && !ever_seen && (now() - dock_start).seconds() > 1.5) {
        RCLCPP_INFO(get_logger(), "到达对位位置后未见目标，先倒车再原地搜索");
        PublishTaskStatus(TaskState::kDocking, 0.6, "目标不在视野，倒车搜索");
        auto visible = [this, color]() {
          std::lock_guard<std::mutex> lock(mutex_);
          const auto& slot = detections_[color];
          const double age =
            static_cast<double>(NowNs() - slot.stamp_ns) / static_cast<double>(kSec);
          return slot.has && slot.stamp_ns > 0 && age <= detection_stale_s_;
        };

        bool found = false;
        // ① 先倒车：跨车改派时落点可能偏差接近一米，车头会顶到目标上，
        //    贴得太近时 0.2 m 的物块落在相机视野外，怎么转都看不见
        const rclcpp::Time back_start = now();
        while (rclcpp::ok() && !found &&
               (now() - back_start).seconds() < kSearchBackoffSec) {
          if (handle->is_canceling()) {
            StopRobot();
            failure_reason_ = kReasonCanceled;
            failure_detail_ = "搜索中被取消";
            return false;
          }
          PublishCmd(-kSearchBackoffVel, 0.0);
          found = visible();
          rclcpp::sleep_for(period);
        }
        // ② 再原地转一圈：目标可能在车的侧后方
        const rclcpp::Time search_start = now();
        while (rclcpp::ok() && !found &&
               (now() - search_start).seconds() < kSearchBudgetSec) {
          if (handle->is_canceling()) {
            StopRobot();
            failure_reason_ = kReasonCanceled;
            failure_detail_ = "搜索中被取消";
            return false;
          }
          PublishCmd(0.0, kSearchYawRate);
          found = visible();
          rclcpp::sleep_for(period);
        }
        StopRobot();
        if (!found) {
          failure_reason_ = kReasonTargetLost;
          failure_detail_ = "到达对位位置后倒车+搜索一圈仍未发现目标";
          return false;
        }
        RCLCPP_INFO(get_logger(), "搜索到目标，重新开始对位");
        dock_->Reset(NowNs());
        ever_seen = true;
        continue;
      }

      const DockingOutput out = dock_->Update(in, NowNs());
      PublishCmd(out.linear, out.angular);

      if (in.target_visible) {
        if (start_range < 0.0) {
          start_range = in.range;
        }
        const double span = std::max(start_range - dock_->config().standoff, 0.05);
        const double progress = 0.6 + 0.4 * Clamp01(1.0 - (in.range - dock_->config().standoff) / span);
        PublishTaskFeedback(handle, TaskState::kDocking, static_cast<float>(in.range),
                            static_cast<float>(progress), "视觉对位中");
      } else {
        PublishTaskFeedback(handle, TaskState::kDocking, -1.0f, 0.6f, "目标暂不可见");
      }

      if (out.done) {
        StopRobot();
        result->final_distance = static_cast<float>(in.range);
        result->final_heading_error = static_cast<float>(in.bearing);
        RCLCPP_INFO(get_logger(), "对位完成: 距离 %.2fm 偏角 %.1f°",
                    in.range, in.bearing * 180.0 / M_PI);
        return true;
      }

      if (out.target_lost) {
        StopRobot();
        failure_reason_ = kReasonTargetLost;
        failure_detail_ = "目标丢失，无法继续对位";
        return false;
      }

      if (out.timed_out) {
        StopRobot();
        failure_reason_ = kReasonDockTimeout;
        failure_detail_ = "对位超时（" + std::to_string(static_cast<int>(dock_->config().timeout_s))
                          + "s）";
        return false;
      }

      // 用节点时钟统计超时更稳，顺便记录真实耗时
      if ((now() - dock_start).seconds() > dock_->config().timeout_s * 2.0) {
        StopRobot();
        failure_reason_ = kReasonDockTimeout;
        failure_detail_ = "对位超时";
        return false;
      }

      rclcpp::sleep_for(period);
    }
    StopRobot();
    failure_reason_ = kReasonDockTimeout;
    failure_detail_ = "节点已关闭";
    return false;
  }

  // ---------------- 收尾与发布 ----------------

  void Finish(const std::shared_ptr<GoalHandleExecute>& handle,
              const std::shared_ptr<ExecuteTask::Result>& result,
              bool success, const std::string& reason, const std::string& detail)
  {
    StopRobot();
    result->success = success;
    result->failure_reason = reason;

    // 收尾前的状态检查：若收到过取消请求，goal 已不在 executing，
    // abort()/succeed() 会抛异常，这里先记录一笔便于定位
    if (handle->is_canceling()) {
      RCLCPP_WARN(get_logger(), "任务 %s 收尾时处于 canceling 状态", task_id_.c_str());
    }

    if (success) {
      tasks_done_.fetch_add(1);
      PublishTaskStatus(TaskState::kDone, 1.0, detail);
      RCLCPP_INFO(get_logger(), "任务 %s 完成", task_id_.c_str());
      // 终态不再发 feedback：feedback 走话题、result 走服务响应，两者没有顺序
      // 保证，客户端拿到 result 就会停止处理反馈（实测会偶发丢掉 DONE 这一帧）。
      // 终态由 action result 承载，这是 action 的语义约定。
      handle->succeed(result);
    } else {
      tasks_failed_.fetch_add(1);
      PublishTaskStatus(TaskState::kFailed, 0.0, detail, reason);
      RCLCPP_WARN(get_logger(), "任务 %s 失败: %s (%s)", task_id_.c_str(), reason.c_str(),
                  detail.c_str());
      handle->abort(result);
    }

    task_id_.clear();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_task_.clear();
    }
    failure_reason_ = kReasonNone;
    failure_detail_.clear();
    busy_ = false;
  }

  void StopRobot()
  {
    PublishCmd(0.0, 0.0);
    PublishCmd(0.0, 0.0);   // 发两帧：避免丢一帧后车还在动
  }

  void PublishCmd(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  void PublishTaskStatus(TaskState state, float progress, const std::string& detail,
                         const std::string& failure = kReasonNone)
  {
    fleet_msgs::msg::TaskStatus msg;
    msg.task_id = task_id_;
    msg.robot_id = robot_id_;
    msg.state = ToMsgState(state);
    msg.progress = progress;
    msg.detail = detail;
    msg.failure_reason = failure;
    msg.retry_count = 0;
    msg.stamp = now();
    task_status_pub_->publish(msg);
  }

  void PublishTaskFeedback(const std::shared_ptr<GoalHandleExecute>& handle, TaskState state,
                           float distance_remaining, float progress, const std::string& detail)
  {
    ExecuteTask::Feedback fb;
    fb.state = ToMsgState(state);
    fb.distance_remaining = distance_remaining;
    fb.progress = progress;
    fb.detail = detail;
    handle->publish_feedback(std::make_shared<ExecuteTask::Feedback>(fb));
  }

  void PublishRobotStatus()
  {
    Pose2D pose;
    std::string current_task;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = pose_;
      current_task = current_task_;
    }

    fleet_msgs::msg::RobotStatus msg;
    msg.robot_id = robot_id_;
    msg.state = busy_ ? fleet_msgs::msg::RobotStatus::STATE_BUSY
                      : fleet_msgs::msg::RobotStatus::STATE_IDLE;
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = QuaternionFromYaw(pose.yaw);
    msg.current_task = current_task;
    msg.tasks_done = tasks_done_.load();
    msg.tasks_failed = tasks_failed_.load();
    msg.heartbeat_seq = heartbeat_seq_++;
    msg.stamp = now();
    robot_status_pub_->publish(msg);
  }

  TimeNs NowNs()
  {
    return now().nanoseconds();
  }

  struct LatestDetection {
    bool has{false};
    double bearing{0.0};
    double range{0.0};
    TimeNs stamp_ns{0};
  };

  std::string robot_id_;
  std::string world_frame_;
  double approach_distance_{1.4};
  double nav_timeout_s_{90.0};
  double detection_stale_s_{1.0};

  std::unique_ptr<DockingController> dock_;

  std::mutex mutex_;
  Pose2D pose_;
  bool has_pose_{false};
  std::array<LatestDetection, 4> detections_;
  double nav_distance_remaining_{-1.0};

  std::atomic<bool> busy_{false};
  std::string task_id_;
  std::string current_task_;   // 心跳线程用，受 mutex_ 保护
  std::string failure_reason_{kReasonNone};
  std::string failure_detail_;
  // 心跳线程要读，execute 线程要写，因此用原子量
  std::atomic<uint16_t> tasks_done_{0};
  std::atomic<uint16_t> tasks_failed_{0};
  uint32_t heartbeat_seq_{0};

  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<fleet_msgs::msg::Detection>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
  rclcpp::Publisher<fleet_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
};

}  // namespace fleet

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fleet::RobotAgent>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
