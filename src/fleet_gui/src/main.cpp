// 监控端入口：先解析 ROS 参数，再把剩下的参数交给 Qt。
#include <QApplication>
#include <QCommandLineParser>
#include <QPixmap>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "fleet_gui/main_window.hpp"
#include "fleet_gui/ros_bridge.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // --ros-args 之类不能塞给 QApplication，否则 Qt 会把它们当成未知选项
  const std::vector<std::string> qt_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> qt_argv;
  qt_argv.reserve(qt_args.size());
  for (const auto& arg : qt_args) {
    qt_argv.push_back(const_cast<char*>(arg.c_str()));
  }

  int qt_argc = static_cast<int>(qt_argv.size());
  QApplication app(qt_argc, qt_argv.data());
  QApplication::setApplicationName(QStringLiteral("fleet_monitor"));
  QApplication::setOrganizationName(QStringLiteral("fleet"));

  QCommandLineParser parser;
  parser.setApplicationDescription(QStringLiteral("ROS2 多机调度监控台"));
  parser.addHelpOption();
  QCommandLineOption screenshot_option(
    QStringLiteral("screenshot"),
    QStringLiteral("渲染完成后截图到指定路径（验收/CI 用）"), QStringLiteral("path"));
  QCommandLineOption quit_option(
    QStringLiteral("quit-after"),
    QStringLiteral("N 秒后自动退出（配合 --screenshot 使用）"), QStringLiteral("seconds"),
    QStringLiteral("0"));
  QCommandLineOption replay_option(
    QStringLiteral("replay"),
    QStringLiteral("启动后直接载入 rosbag2 目录"), QStringLiteral("dir"));
  QCommandLineOption camera_option(
    QStringLiteral("camera"),
    QStringLiteral("初始显示的相机（默认 robot1）"), QStringLiteral("robot"),
    QStringLiteral("robot1"));
  QCommandLineOption tab_option(
    QStringLiteral("tab"),
    QStringLiteral("初始页：任务 / 车辆 / 告警 / 日志"), QStringLiteral("name"),
    QStringLiteral("任务"));
  parser.addOption(screenshot_option);
  parser.addOption(quit_option);
  parser.addOption(replay_option);
  parser.addOption(camera_option);
  parser.addOption(tab_option);
  parser.process(app);

  // 跨线程发结构体信号，必须先注册元类型
  qRegisterMetaType<fleet::monitor::TaskView>("fleet::monitor::TaskView");
  qRegisterMetaType<fleet::monitor::RobotView>("fleet::monitor::RobotView");
  qRegisterMetaType<fleet::monitor::StatusEvent>("fleet::monitor::StatusEvent");
  qRegisterMetaType<fleet::monitor::MapData>("fleet::monitor::MapData");

  fleet::gui::SignalHub hub;
  auto node = std::make_shared<fleet::gui::MonitorNode>(&hub);
  node->Start();

  fleet::gui::MainWindow window(node.get(), &hub);
  window.SelectCamera(parser.value(camera_option));
  window.SelectTab(parser.value(tab_option));
  window.show();

  if (parser.isSet(replay_option)) {
    window.OpenBag(parser.value(replay_option));
  }

  const int quit_after_s = parser.value(quit_option).toInt();
  const QString screenshot = parser.value(screenshot_option);
  if (!screenshot.isEmpty()) {
    const int delay_ms = quit_after_s > 0 ? quit_after_s * 1000 : 8000;
    QTimer::singleShot(delay_ms, &window, [&app, &window, screenshot]() {
      window.grab().save(screenshot);
      app.quit();
    });
  } else if (quit_after_s > 0) {
    QTimer::singleShot(quit_after_s * 1000, &app, &QCoreApplication::quit);
  }

  const int rc = app.exec();
  node->Stop();
  rclcpp::shutdown();
  return rc;
}