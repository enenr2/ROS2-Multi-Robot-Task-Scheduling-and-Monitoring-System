// 调度中心（ROS 适配层）：把 fleet_core 的纯逻辑接到话题、服务与 action 上。
//
// 分工：
//   - 事件来源：各车 detections / task_status / robot_status，外部 SubmitTask/CancelTask
//   - 决策：TaskQueue（去重、状态、重试）、Assignment（选车）、Watchdog（超时判定）
//   - 执行：对每台车直调 execute_task action，一车同一时刻只有一个任务
//
// 线程模型：用单线程执行器。fleet_core 的契约是"单线程推进、不加锁"，
// 而本节点所有回调都是非阻塞的（action 用异步接口），所以单线程即可，
// 内核状态不会被并发访问。将来若改成多线程执行器，这里的假设必须重新审。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fleet_manager/assignment.hpp"
#include "fleet_manager/task_queue.hpp"
#include "fleet_manager/watchdog.hpp"
#include "fleet_msgs/action/execute_task.hpp"
#include "fleet_msgs/msg/detection.hpp"
#include "fleet_msgs/msg/robot_status.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/task_status.hpp"
#include "fleet_msgs/srv/cancel_task.hpp"
#include "fleet_msgs/srv/submit_task.hpp"

namespace fleet {

class FleetManagerNode : public rclcpp::Node
{
public:
  FleetManagerNode();

private:
  using ExecuteTask = fleet_msgs::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  // ---- 事件接入 ----
  void OnDetection(const fleet_msgs::msg::Detection& msg);
  void OnTaskStatus(const fleet_msgs::msg::TaskStatus& msg);
  void OnRobotStatus(const fleet_msgs::msg::RobotStatus& msg);
  void OnSubmit(const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> req,
                std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> res);
  void OnCancel(const std::shared_ptr<fleet_msgs::srv::CancelTask::Request> req,
                std::shared_ptr<fleet_msgs::srv::CancelTask::Response> res);

  // ---- 周期性推进（单线程，全部状态变更集中在这里与上面的回调） ----
  void Tick();
  void HandleWatchdog(TimeNs now);
  void AssignPending(TimeNs now);

  // ---- action ----
  bool SendGoal(const RobotId& robot, const Task& task);
  void CancelGoal(const RobotId& robot);
  void OnGoalResponse(const RobotId& robot, const TaskId& task_id,
                      const GoalHandle::SharedPtr& handle);
  void OnActionResult(const RobotId& robot, const TaskId& task_id,
                      const GoalHandle::WrappedResult& wrapped);

  // ---- 状态工具 ----
  void ReleaseRobot(const RobotId& robot, bool offline = false);
  void FailActiveTask(const RobotId& robot, const std::string& reason);
  void MarkCooldown(const RobotId& robot);
  void PublishTaskSnapshot();
  void PublishMarkers();
  std::vector<Robot> RobotList() const;
  std::vector<Robot> IdleRobots() const;
  const RobotId* RobotOfTask(const TaskId& id) const;

  TimeNs NowNs() const;

  // ---- 配置 ----
  double standoff_{0.6};
  bool publish_markers_{true};
  // 只有这些车会接任务（dispatch_robot_ids 控制；两车都接 Nav2，默认都可派）
  std::vector<std::string> dispatch_ids_;
  // 失败冷却：刚失败的车短时间内不再接单，否则重试会被"最近优先"又派回同一台
  TimeNs cooldown_ns_{10 * kSecond};
  WatchdogConfig watchdog_cfg_;

  // ---- 内核 ----
  TaskQueue queue_;
  Assignment assignment_;
  Watchdog watchdog_;

  // ---- 运行时状态（单线程访问，无需锁）----
  std::map<RobotId, Robot> robots_;
  std::map<RobotId, std::shared_ptr<rclcpp_action::Client<ExecuteTask>>> clients_;
  std::map<RobotId, GoalHandle::SharedPtr> active_goals_;
  std::map<RobotId, TaskId> robot_task_;
  std::map<RobotId, TimeNs> cooldown_until_;
  uint64_t task_seq_{0};

  // ---- ROS 实体 ----
  rclcpp::CallbackGroup::SharedPtr client_group_;
  std::vector<rclcpp::Subscription<fleet_msgs::msg::Detection>::SharedPtr> detection_subs_;
  std::vector<rclcpp::Subscription<fleet_msgs::msg::TaskStatus>::SharedPtr> task_status_subs_;
  std::vector<rclcpp::Subscription<fleet_msgs::msg::RobotStatus>::SharedPtr> robot_status_subs_;
  rclcpp::Publisher<fleet_msgs::msg::Task>::SharedPtr task_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<fleet_msgs::srv::SubmitTask>::SharedPtr submit_srv_;
  rclcpp::Service<fleet_msgs::srv::CancelTask>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace fleet
