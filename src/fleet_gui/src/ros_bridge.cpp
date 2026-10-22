#include "fleet_gui/ros_bridge.hpp"

#include <cmath>
#include <utility>

namespace fleet::gui {

namespace {

int64_t ToMs(const builtin_interfaces::msg::Time& t)
{
  return static_cast<int64_t>(t.sec) * 1000 + static_cast<int64_t>(t.nanosec) / 1000000;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

fleet::monitor::TaskView ToTaskView(const fleet_msgs::msg::Task& msg)
{
  fleet::monitor::TaskView view;
  view.task_id = msg.task_id;
  view.color = msg.target_color;
  view.state = msg.state;
  view.x = msg.target_pose.position.x;
  view.y = msg.target_pose.position.y;
  view.robot = msg.assigned_robot_id;
  view.failure_reason = msg.failure_reason;
  view.retry = msg.retry_count;
  view.max_retry = msg.max_retries;
  view.detected_ms = ToMs(msg.detected_at);
  view.assigned_ms = ToMs(msg.assigned_at);
  view.revision = msg.revision;
  return view;
}

fleet::monitor::RobotView ToRobotView(const fleet_msgs::msg::RobotStatus& msg)
{
  fleet::monitor::RobotView view;
  view.robot_id = msg.robot_id;
  view.state = msg.state;
  view.x = msg.pose.position.x;
  view.y = msg.pose.position.y;
  view.yaw = YawFromQuaternion(msg.pose.orientation);
  view.current_task = msg.current_task;
  view.tasks_done = msg.tasks_done;
  view.tasks_failed = msg.tasks_failed;
  view.tasks_canceled = msg.tasks_canceled;
  view.heartbeat_seq = msg.heartbeat_seq;
  return view;
}

fleet::monitor::StatusEvent ToStatusEvent(const fleet_msgs::msg::TaskStatus& msg)
{
  fleet::monitor::StatusEvent ev;
  ev.task_id = msg.task_id;
  ev.robot_id = msg.robot_id;
  ev.state = msg.state;
  ev.progress = msg.progress;
  ev.detail = msg.detail;
  ev.failure_reason = msg.failure_reason;
  ev.retry = msg.retry_count;
  ev.stamp_ms = ToMs(msg.stamp);
  return ev;
}

// gz 相机经桥接后常见 rgb8 / bgr8 / rgba8 / bgra8 / mono8
QImage ToQImage(const sensor_msgs::msg::Image& msg)
{
  if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return QImage();
  }
  const int w = static_cast<int>(msg.width);
  const int h = static_cast<int>(msg.height);
  const int step = static_cast<int>(msg.step);
  const uchar* bits = reinterpret_cast<const uchar*>(msg.data.data());

  if (msg.encoding == "rgb8") {
    return QImage(bits, w, h, step, QImage::Format_RGB888).copy();
  }
  if (msg.encoding == "bgr8") {
    return QImage(bits, w, h, step, QImage::Format_RGB888).rgbSwapped();
  }
  if (msg.encoding == "rgba8") {
    return QImage(bits, w, h, step, QImage::Format_RGBA8888).copy();
  }
  if (msg.encoding == "bgra8") {
    return QImage(bits, w, h, step, QImage::Format_ARGB32).copy();
  }
  if (msg.encoding == "mono8") {
    return QImage(bits, w, h, step, QImage::Format_Grayscale8).copy();
  }
  return QImage();
}

// /robot1/robot_status -> robot="robot1", kind="robot_status"
bool SplitRobotTopic(const std::string& topic, std::string* robot, std::string* kind)
{
  if (topic.size() < 4 || topic.front() != '/') {
    return false;
  }
  const size_t second = topic.find('/', 1);
  if (second == std::string::npos || second <= 1) {
    return false;
  }
  if (topic.find('/', second + 1) != std::string::npos) {
    return false;   // 只认两级：/<robot>/<kind>
  }
  *robot = topic.substr(1, second - 1);
  *kind = topic.substr(second + 1);
  return true;
}

}  // namespace

SignalHub::SignalHub(QObject* parent)
: QObject(parent), camera_robot_("robot1") {}

QString SignalHub::cameraRobot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return QString::fromStdString(camera_robot_);
}

void SignalHub::setCameraRobot(const QString& robot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  camera_robot_ = robot.toStdString();
}

MonitorNode::MonitorNode(SignalHub* hub)
: Node("fleet_monitor"), hub_(hub)
{
  task_sub_ = create_subscription<fleet_msgs::msg::Task>(
    "/fleet/tasks", rclcpp::QoS(20),
    [this](fleet_msgs::msg::Task::SharedPtr msg) {
      emit hub_->taskSnapshot(ToTaskView(*msg));
    });

  // 地图是 transient_local：监视器晚启动也能拿到最后一张
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      fleet::monitor::MapData data;
      data.width = static_cast<int>(msg->info.width);
      data.height = static_cast<int>(msg->info.height);
      data.resolution = msg->info.resolution;
      data.origin_x = msg->info.origin.position.x;
      data.origin_y = msg->info.origin.position.y;
      data.cells = msg->data;
      emit hub_->mapData(data);
    });

  submit_client_ = create_client<fleet_msgs::srv::SubmitTask>("/fleet/submit_task");
  cancel_client_ = create_client<fleet_msgs::srv::CancelTask>("/fleet/cancel_task");

  discovery_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { DiscoverRobots(); });

  RCLCPP_INFO(get_logger(), "监控端就绪（只读订阅 + 注入/取消服务）");
}

MonitorNode::~MonitorNode()
{
  Stop();
}

void MonitorNode::Start()
{
  if (spinning_) {
    return;
  }
  exec_.add_node(shared_from_this());
  spinning_ = true;
  thread_ = std::thread([this]() { exec_.spin(); });
}

void MonitorNode::Stop()
{
  if (!spinning_) {
    return;
  }
  exec_.cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  spinning_ = false;
}

size_t MonitorNode::robotCount() const
{
  return robots_.size();
}

void MonitorNode::DiscoverRobots()
{
  std::map<std::string, std::vector<std::string>> topics;
  try {
    topics = get_topic_names_and_types();
  } catch (const std::exception& ex) {
    RCLCPP_WARN(get_logger(), "枚举话题失败: %s", ex.what());
    return;
  }

  for (const auto& [topic, types] : topics) {
    std::string robot;
    std::string kind;
    if (!SplitRobotTopic(topic, &robot, &kind)) {
      continue;
    }
    if (kind != "robot_status" && kind != "task_status" && kind != "camera/image") {
      continue;
    }
    if (!robots_.insert(robot).second) {
      continue;
    }
    RCLCPP_INFO(get_logger(), "发现机器人 %s（来自话题 %s）", robot.c_str(), topic.c_str());
    SubscribeRobot(robot);
    emit hub_->robotDiscovered(QString::fromStdString(robot));
  }
}

void MonitorNode::SubscribeRobot(const std::string& robot)
{
  RobotSubs subs;

  subs.status = create_subscription<fleet_msgs::msg::RobotStatus>(
    "/" + robot + "/robot_status", rclcpp::QoS(10),
    [this](fleet_msgs::msg::RobotStatus::SharedPtr msg) {
      emit hub_->robotStatus(ToRobotView(*msg));
    });

  subs.task_status = create_subscription<fleet_msgs::msg::TaskStatus>(
    "/" + robot + "/task_status", rclcpp::QoS(20),
    [this](fleet_msgs::msg::TaskStatus::SharedPtr msg) {
      emit hub_->taskStatus(ToStatusEvent(*msg));
    });

  subs.camera = create_subscription<sensor_msgs::msg::Image>(
    "/" + robot + "/camera/image", rclcpp::SensorDataQoS(),
    [this, robot](sensor_msgs::msg::Image::SharedPtr msg) {
      if (hub_->cameraRobot().toStdString() != robot) {
        return;   // 只解码当前选中的相机
      }
      const QImage image = ToQImage(*msg);
      if (!image.isNull()) {
        emit hub_->cameraFrame(QString::fromStdString(robot), image);
      }
    });

  subs_[robot] = subs;
}

void MonitorNode::SubmitTask(uint8_t color, double x, double y)
{
  if (!submit_client_->service_is_ready()) {
    emit hub_->notice("调度中心未就绪：/fleet/submit_task 不可用", 1);
    return;
  }
  auto request = std::make_shared<fleet_msgs::srv::SubmitTask::Request>();
  request->target_color = color;
  request->target_pose.position.x = x;
  request->target_pose.position.y = y;
  request->target_pose.orientation.w = 1.0;

  submit_client_->async_send_request(
    request,
    [this, color](rclcpp::Client<fleet_msgs::srv::SubmitTask>::SharedFuture future) {
      const auto response = future.get();
      emit hub_->notice(
        QString("注入%1色目标：%2")
          .arg(QString::fromStdString(fleet::monitor::ColorName(color)))
          .arg(QString::fromStdString(response->message)),
        response->accepted ? 0 : 1);
    });
}

void MonitorNode::CancelTask(const QString& task_id)
{
  if (task_id.isEmpty()) {
    return;
  }
  if (!cancel_client_->service_is_ready()) {
    emit hub_->notice("调度中心未就绪：/fleet/cancel_task 不可用", 1);
    return;
  }
  auto request = std::make_shared<fleet_msgs::srv::CancelTask::Request>();
  request->task_id = task_id.toStdString();
  request->reason = "CANCELED_BY_OPERATOR";

  cancel_client_->async_send_request(
    request,
    [this, task_id](rclcpp::Client<fleet_msgs::srv::CancelTask>::SharedFuture future) {
      const auto response = future.get();
      emit hub_->notice(QString("取消 %1：%2").arg(task_id).arg(
                          QString::fromStdString(response->message)),
                        response->canceled ? 0 : 1);
    });
}

}  // namespace fleet::gui