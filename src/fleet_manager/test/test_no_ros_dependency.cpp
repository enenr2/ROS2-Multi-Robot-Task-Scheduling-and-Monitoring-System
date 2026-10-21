// 依赖约束测试：业务逻辑库必须能在没有 ROS 的编译环境下构建。
//
// 这个包刻意不 find_package(rclcpp / fleet_msgs / geometry_msgs)。
// 如果哪天有人在 fleet_core 里 include 了 ROS 头文件，这个测试文件
// 会直接编译失败 —— 用编译期失败来固化架构约束，比写文档更管用。
#include <gtest/gtest.h>

#include "fleet_manager/assignment.hpp"
#include "fleet_manager/state_machine.hpp"
#include "fleet_manager/task_queue.hpp"
#include "fleet_manager/watchdog.hpp"

TEST(NoRosDependency, CoreHeadersAreSelfContained)
{
  // 能在此处构造并使用全部核心类型，即证明它们不依赖 ROS 上下文
  fleet::TaskQueue q;
  fleet::Assignment a;
  fleet::Watchdog w;

  EXPECT_TRUE(q.AddTask("t1", fleet::Color::kRed, fleet::Pose2D{1.0, 1.0, 0.0}, 0));
  EXPECT_EQ(q.Size(), 1u);

  auto d = a.SelectRobot(fleet::Pose2D{0.0, 0.0, 0.0},
                         {fleet::Robot{"robot1", fleet::RobotState::kIdle,
                                       fleet::Pose2D{0.0, 1.0, 0.0}, "", 0, 0, 0, 0}},
                         {});
  EXPECT_EQ(d.robot, "robot1");

  EXPECT_TRUE(w.Check(q.All(), {}, 0).empty());
}

TEST(NoRosDependency, TimeIsInjectedNotReadFromClock)
{
  // 逻辑层不取系统时钟：同样的输入与同样的 now 必须给出同样的结果，
  // 这是回放测试能够比对的前提
  fleet::WatchdogConfig cfg;
  cfg.nav_timeout = 10 * fleet::kSecond;
  fleet::Watchdog w(cfg);

  fleet::Task t;
  t.id = "t1";
  t.state = fleet::TaskState::kNavigating;
  t.assigned_at = 100 * fleet::kSecond;

  EXPECT_FALSE(w.IsTaskTimedOut(t, 105 * fleet::kSecond, nullptr));
  EXPECT_TRUE(w.IsTaskTimedOut(t, 111 * fleet::kSecond, nullptr));
  // 再判一次同一时刻，结果必须一致（无隐藏状态）
  EXPECT_TRUE(w.IsTaskTimedOut(t, 111 * fleet::kSecond, nullptr));
}
