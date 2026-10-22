// ROS 适配层：把 ROS 回调搬到 Qt 线程。
//
// 线程模型：节点在独立 std::thread 里跑单线程执行器，所有回调只做"消息 ->
// 纯结构体 / QImage"的转换，然后发 Qt 信号。SignalHub 活在 GUI 线程，
// 跨线程的自动连接会排队到 GUI 事件循环，界面不需要加锁。
#pragma once

#include <QImage>
#include <QObject>
#include <QString>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fleet_gui/monitor_core.hpp"
#include "fleet_gui/qt_metatypes.hpp"
#include "fleet_msgs/msg/robot_status.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/task_status.hpp"
#include "fleet_msgs/srv/cancel_task.hpp"
#include "fleet_msgs/srv/submit_task.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace fleet::gui {

class SignalHub : public QObject
{
  Q_OBJECT

public:
  explicit SignalHub(QObject* parent = nullptr);

  // 相机只解码当前选中的车，避免两路 640x480 同时转 QImage
  QString cameraRobot() const;
  void setCameraRobot(const QString& robot);

signals:
  void taskSnapshot(const fleet::monitor::TaskView& task);
  void robotStatus(const fleet::monitor::RobotView& robot);
  void taskStatus(const fleet::monitor::StatusEvent& status);
  void mapData(const fleet::monitor::MapData& map);
  void cameraFrame(const QString& robot, const QImage& image);
  // 服务调用结果等一次性提示，severity: 0 普通 / 1 警告 / 2 错误
  void notice(const QString& text, int severity);
  void robotDiscovered(const QString& robot);

private:
  mutable std::mutex mutex_;
  std::string camera_robot_;
};

class MonitorNode : public rclcpp::Node
{
public:
  explicit MonitorNode(SignalHub* hub);
  ~MonitorNode() override;

  // 起 ROS 线程；不调用则只有信号连接、没有数据
  void Start();
  void Stop();

  void SubmitTask(uint8_t color, double x, double y);
  void CancelTask(const QString& task_id);
  size_t robotCount() const;

private:
  struct RobotSubs
  {
    rclcpp::Subscription<fleet_msgs::msg::RobotStatus>::SharedPtr status;
    rclcpp::Subscription<fleet_msgs::msg::TaskStatus>::SharedPtr task_status;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera;
  };

  // 每 2s 扫一遍话题表，发现新上线的 /<robot>/{robot_status,task_status,camera/image}
  void DiscoverRobots();
  void SubscribeRobot(const std::string& robot);

  SignalHub* hub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread thread_;
  bool spinning_{false};

  std::set<std::string> robots_;
  std::map<std::string, RobotSubs> subs_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  rclcpp::Subscription<fleet_msgs::msg::Task>::SharedPtr task_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Client<fleet_msgs::srv::SubmitTask>::SharedPtr submit_client_;
  rclcpp::Client<fleet_msgs::srv::CancelTask>::SharedPtr cancel_client_;
};

}  // namespace fleet::gui