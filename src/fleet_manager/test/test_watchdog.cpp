// 看门狗测试：超时边界、离线判定、判定与执行分离。
#include <gtest/gtest.h>

#include "fleet_manager/watchdog.hpp"

using namespace fleet;

namespace {

constexpr TimeNs kT0 = 1000 * kSecond;

Task MakeTask(const std::string& id, TaskState st, TimeNs assigned_at, TimeNs detected_at)
{
  Task t;
  t.id = id;
  t.color = Color::kRed;
  t.state = st;
  t.assigned_at = assigned_at;
  t.detected_at = detected_at;
  return t;
}

Robot MakeRobot(const std::string& id, TimeNs last_hb, RobotState st = RobotState::kIdle)
{
  Robot r;
  r.id = id;
  r.last_heartbeat = last_hb;
  r.state = st;
  return r;
}

}  // namespace

TEST(Watchdog, NavTimeoutBoundary)
{
  WatchdogConfig cfg;
  cfg.nav_timeout = 60 * kSecond;
  Watchdog w(cfg);

  auto t = MakeTask("t1", TaskState::kNavigating, kT0, kT0);
  TimeoutEvent ev;

  // 正好到时限：不算超时（判定用 now > deadline）
  EXPECT_FALSE(w.IsTaskTimedOut(t, kT0 + 60 * kSecond, &ev));
  // 超 1 纳秒：算超时
  EXPECT_TRUE(w.IsTaskTimedOut(t, kT0 + 60 * kSecond + 1, &ev));
  EXPECT_EQ(ev.kind, TimeoutKind::kTask);
  EXPECT_EQ(ev.task_id, "t1");
  EXPECT_EQ(ev.reason, "NAV_TIMEOUT");
  EXPECT_EQ(ev.state_at_timeout, TaskState::kNavigating);
  EXPECT_EQ(ev.overdue, 1);
}

TEST(Watchdog, DockTimeoutFromAssignment)
{
  WatchdogConfig cfg;
  cfg.dock_timeout = 30 * kSecond;
  Watchdog w(cfg);

  // assigned_at 是派单时刻，不随阶段刷新
  auto t = MakeTask("t1", TaskState::kDocking, kT0, kT0);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsTaskTimedOut(t, kT0 + 29 * kSecond, &ev));
  EXPECT_TRUE(w.IsTaskTimedOut(t, kT0 + 31 * kSecond, &ev));
  EXPECT_EQ(ev.reason, "DOCK_TIMEOUT");
}

TEST(Watchdog, PendingAssignTimeoutUsesDetectedAt)
{
  WatchdogConfig cfg;
  cfg.assign_timeout = 30 * kSecond;
  Watchdog w(cfg);

  auto t = MakeTask("t1", TaskState::kPending, 0, kT0);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsTaskTimedOut(t, kT0 + 30 * kSecond, &ev));
  EXPECT_TRUE(w.IsTaskTimedOut(t, kT0 + 30 * kSecond + 1, &ev));
  EXPECT_EQ(ev.reason, "ASSIGN_TIMEOUT");
}

TEST(Watchdog, TerminalAndAssignedStatesNotJudged)
{
  Watchdog w;
  TimeoutEvent ev;
  const TimeNs far_future = kT0 + 3600 * kSecond;

  for (auto st : {TaskState::kDone, TaskState::kFailed, TaskState::kCanceled,
                  TaskState::kAssigned}) {
    auto t = MakeTask("t1", st, kT0, kT0);
    EXPECT_FALSE(w.IsTaskTimedOut(t, far_future, &ev))
      << "状态 " << ToString(st) << " 不应被超时判定命中";
  }
}

TEST(Watchdog, MissingAssignedAtDoesNotTriggerNavTimeout)
{
  Watchdog w;
  auto t = MakeTask("t1", TaskState::kNavigating, 0, kT0);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsTaskTimedOut(t, kT0 + 999 * kSecond, &ev))
    << "缺少分配时间戳说明数据不完整，不该误判超时";
}

TEST(Watchdog, RobotOfflineBoundary)
{
  WatchdogConfig cfg;
  cfg.heartbeat_timeout = 3 * kSecond;
  Watchdog w(cfg);

  auto r = MakeRobot("robot1", kT0);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsRobotOffline(r, kT0 + 3 * kSecond, &ev));
  EXPECT_TRUE(w.IsRobotOffline(r, kT0 + 3 * kSecond + 1, &ev));
  EXPECT_EQ(ev.kind, TimeoutKind::kRobotOffline);
  EXPECT_EQ(ev.robot_id, "robot1");
  EXPECT_EQ(ev.reason, "ROBOT_UNREACHABLE");
}

TEST(Watchdog, NeverHeartbeatedRobotIsNotOffline)
{
  Watchdog w;
  auto r = MakeRobot("robot1", 0);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsRobotOffline(r, kT0 + 999 * kSecond, &ev))
    << "注册后到首次心跳之间不应被误杀";
}

TEST(Watchdog, AlreadyOfflineNotReportedTwice)
{
  Watchdog w;
  auto r = MakeRobot("robot1", kT0, RobotState::kOffline);
  TimeoutEvent ev;
  EXPECT_FALSE(w.IsRobotOffline(r, kT0 + 999 * kSecond, &ev))
    << "已标记离线的车重复上报会产生告警风暴";
}

TEST(Watchdog, CheckAggregatesTasksAndRobots)
{
  WatchdogConfig cfg;
  cfg.nav_timeout = 10 * kSecond;
  cfg.heartbeat_timeout = 3 * kSecond;
  Watchdog w(cfg);

  std::vector<Task> tasks = {
    MakeTask("t_nav", TaskState::kNavigating, kT0, kT0),   // 超时
    MakeTask("t_done", TaskState::kDone, kT0, kT0),        // 不超时
  };
  std::vector<Robot> robots = {
    MakeRobot("robot1", kT0),                    // 超时（now = +20s）
    MakeRobot("robot2", kT0 + 19 * kSecond),     // 正常心跳
  };

  auto events = w.Check(tasks, robots, kT0 + 20 * kSecond);
  ASSERT_EQ(events.size(), 2u);

  int task_events = 0, robot_events = 0;
  for (const auto& e : events) {
    if (e.kind == TimeoutKind::kTask) {
      ++task_events;
      EXPECT_EQ(e.task_id, "t_nav");
      EXPECT_EQ(e.reason, "NAV_TIMEOUT");
    } else {
      ++robot_events;
      EXPECT_EQ(e.robot_id, "robot1");
    }
  }
  EXPECT_EQ(task_events, 1);
  EXPECT_EQ(robot_events, 1);
}

TEST(Watchdog, CheckOnEmptyInputsReturnsNothing)
{
  Watchdog w;
  EXPECT_TRUE(w.Check({}, {}, kT0).empty());
}

TEST(Watchdog, WatchdogDoesNotMutateInputs)
{
  WatchdogConfig cfg;
  cfg.nav_timeout = 1 * kSecond;
  Watchdog w(cfg);

  auto task = MakeTask("t1", TaskState::kNavigating, kT0, kT0);
  auto robot = MakeRobot("robot1", kT0);
  const Task task_before = task;
  const Robot robot_before = robot;

  auto events = w.Check({task}, {robot}, kT0 + 100 * kSecond);
  EXPECT_FALSE(events.empty());

  // 判定与执行分离：看门狗只能报告，不能改数据
  EXPECT_EQ(task.state, task_before.state);
  EXPECT_EQ(task.revision, task_before.revision);
  EXPECT_EQ(robot.state, robot_before.state);
  EXPECT_EQ(robot.last_heartbeat, robot_before.last_heartbeat);
}
