#include <gtest/gtest.h>

#include <string>

#include "fleet_gui/monitor_core.hpp"

namespace fleet::monitor {
namespace {

TaskView MakeTask(const std::string& id, int state, uint64_t revision,
                  const std::string& robot = "")
{
  TaskView task;
  task.task_id = id;
  task.color = kColorRed;
  task.state = state;
  task.x = 3.0;
  task.y = 1.0;
  task.robot = robot;
  task.revision = revision;
  task.detected_ms = 1000;
  return task;
}

RobotView MakeRobot(int state = kRobotIdle, const std::string& task = "")
{
  RobotView robot;
  robot.robot_id = "robot1";
  robot.state = state;
  robot.x = 1.0;
  robot.y = 2.0;
  robot.yaw = 0.5;
  robot.current_task = task;
  return robot;
}

bool Contains(const std::string& text, const std::string& needle)
{
  return text.find(needle) != std::string::npos;
}

}  // namespace

TEST(MonitorCore, SnapshotIgnoresStaleRevision)
{
  MonitorModel model;
  const auto first = model.OnTaskSnapshot(MakeTask("T1", kStatePending, 2), 1000);
  ASSERT_EQ(first.size(), 1u);   // 新任务
  EXPECT_TRUE(Contains(first[0].text, "新任务"));

  // revision 回退的旧包不能把状态倒回去
  const auto stale = model.OnTaskSnapshot(MakeTask("T1", kStateDone, 1), 1100);
  EXPECT_TRUE(stale.empty());
  EXPECT_EQ(model.tasks().at("T1").state, kStatePending);

  // 相同 revision 的重复快照（2Hz 全量重发）不产生事件、不改状态
  const auto repeat = model.OnTaskSnapshot(MakeTask("T1", kStatePending, 2), 1200);
  EXPECT_TRUE(repeat.empty());
}

TEST(MonitorCore, StateTransitionAnnouncedOnce)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStatePending, 1), 1000);

  const auto assigned = model.OnTaskSnapshot(MakeTask("T1", kStateAssigned, 2, "robot1"), 1500);
  ASSERT_EQ(assigned.size(), 1u);
  EXPECT_EQ(assigned[0].severity, Severity::kInfo);
  EXPECT_TRUE(Contains(assigned[0].text, "已派单"));
  EXPECT_TRUE(Contains(assigned[0].text, "robot1"));

  // 同一状态重复上报不应该再记日志
  EXPECT_TRUE(model.OnTaskSnapshot(MakeTask("T1", kStateAssigned, 3, "robot1"), 1700).empty());
}

TEST(MonitorCore, ReassignmentIsLogged)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStateNavigating, 1, "robot1"), 1000);
  const auto events = model.OnTaskSnapshot(MakeTask("T1", kStateNavigating, 2, "robot2"), 2000);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_TRUE(Contains(events[0].text, "改派给 robot2"));
}

TEST(MonitorCore, RetryAlarmOnce)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStatePending, 1), 1000);

  TaskView retried = MakeTask("T1", kStatePending, 2);
  retried.retry = 1;
  retried.max_retry = 1;
  retried.failure_reason = "NAV_TIMEOUT";
  const auto events = model.OnTaskSnapshot(retried, 2000);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].severity, Severity::kWarn);
  EXPECT_TRUE(Contains(events[0].text, "第 1 次重排"));
  EXPECT_TRUE(Contains(events[0].text, "导航超时"));

  // 重试次数没变就不重复告警
  EXPECT_TRUE(model.OnTaskSnapshot(retried, 2200).empty());
}

TEST(MonitorCore, StatusDrivesStateAndProgress)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStateNavigating, 1, "robot1"), 1000);

  StatusEvent status;
  status.task_id = "T1";
  status.robot_id = "robot1";
  status.state = kStateDocking;
  status.progress = 0.65;
  status.detail = "视觉对位";

  const auto events = model.OnTaskStatus(status, 1200);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_TRUE(Contains(events[0].text, "对位中"));
  EXPECT_DOUBLE_EQ(model.Progress("T1"), 0.65);
  EXPECT_EQ(model.Detail("T1"), "视觉对位");
  EXPECT_EQ(model.tasks().at("T1").state, kStateDocking);

  // 同一状态的高频进度上报只更新进度，不进日志
  status.progress = 0.8;
  EXPECT_TRUE(model.OnTaskStatus(status, 1300).empty());
  EXPECT_DOUBLE_EQ(model.Progress("T1"), 0.8);
}

TEST(MonitorCore, FailureCarriesReasonAndSeverity)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStateDocking, 1, "robot1"), 1000);

  StatusEvent status;
  status.task_id = "T1";
  status.robot_id = "robot1";
  status.state = kStateFailed;
  status.failure_reason = "TARGET_LOST";
  status.detail = "倒车 + 转一圈仍未发现目标";

  const auto events = model.OnTaskStatus(status, 1500);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].severity, Severity::kError);
  EXPECT_TRUE(Contains(events[0].text, "目标丢失"));
  EXPECT_TRUE(Contains(events[0].text, "倒车"));
  EXPECT_EQ(model.tasks().at("T1").failure_reason, "TARGET_LOST");
}

TEST(MonitorCore, OfflineAfterTimeoutAndRecovery)
{
  MonitorModel model(2500);
  const auto online = model.OnRobotStatus(MakeRobot(), 1000);
  ASSERT_EQ(online.size(), 1u);
  EXPECT_TRUE(Contains(online[0].text, "上线"));
  EXPECT_TRUE(model.robots().at("robot1").online);

  // 1Hz 心跳，2.4s 内不算离线（容忍一次丢包）
  EXPECT_TRUE(model.Sweep(3400).empty());
  const auto offline = model.Sweep(3600);
  ASSERT_EQ(offline.size(), 1u);
  EXPECT_EQ(offline[0].severity, Severity::kWarn);
  EXPECT_TRUE(Contains(offline[0].text, "判定离线"));
  EXPECT_FALSE(model.robots().at("robot1").online);

  // 一直没心跳也只报一次
  EXPECT_TRUE(model.Sweep(9000).empty());

  const auto recovered = model.OnRobotStatus(MakeRobot(), 9500);
  ASSERT_FALSE(recovered.empty());
  EXPECT_TRUE(Contains(recovered[0].text, "恢复在线"));
  EXPECT_TRUE(model.robots().at("robot1").online);
}

TEST(MonitorCore, RobotBusyAndErrorEvents)
{
  MonitorModel model;
  model.OnRobotStatus(MakeRobot(), 1000);

  const auto busy = model.OnRobotStatus(MakeRobot(kRobotBusy, "T1"), 2000);
  ASSERT_EQ(busy.size(), 1u);
  EXPECT_TRUE(Contains(busy[0].text, "开始执行 T1"));

  const auto error = model.OnRobotStatus(MakeRobot(kRobotError, "T1"), 3000);
  ASSERT_EQ(error.size(), 1u);
  EXPECT_EQ(error[0].severity, Severity::kError);
  EXPECT_TRUE(Contains(error[0].text, "异常"));
}

TEST(MonitorCore, SortedTasksPutActiveFirst)
{
  MonitorModel model;
  model.OnTaskSnapshot(MakeTask("T1", kStatePending, 1), 1000);
  model.OnTaskSnapshot(MakeTask("T2", kStateNavigating, 1, "robot1"), 900);
  model.OnTaskSnapshot(MakeTask("T3", kStateDone, 1, "robot2"), 1200);

  const auto sorted = model.SortedTasks();
  ASSERT_EQ(sorted.size(), 3u);
  EXPECT_EQ(sorted[0].task_id, "T2");   // 执行中
  EXPECT_EQ(sorted[1].task_id, "T1");   // 待分配
  EXPECT_EQ(sorted[2].task_id, "T3");   // 已结束

  EXPECT_EQ(model.ActiveCount(), 1);
  EXPECT_EQ(model.DoneCount(), 1);
  EXPECT_EQ(model.FailedCount(), 0);
}

TEST(MonitorCore, RegisteredRobotWithoutHeartbeatStaysOffline)
{
  MonitorModel model(2500);
  const auto registered = model.EnsureRobot("robot1", 1000);
  ASSERT_EQ(registered.size(), 1u);
  EXPECT_TRUE(Contains(registered[0].text, "等待心跳"));
  const auto& robot = model.robots().at("robot1");
  EXPECT_FALSE(robot.online);
  EXPECT_FALSE(robot.heartbeat_seen);

  // 登记不算掉线，不重复播报
  EXPECT_TRUE(model.EnsureRobot("robot1", 1200).empty());
  EXPECT_TRUE(model.Sweep(3000).empty());

  // 超过 2 倍心跳超时仍无心跳：给一次告警
  const auto warned = model.Sweep(6200);
  ASSERT_EQ(warned.size(), 1u);
  EXPECT_EQ(warned[0].severity, Severity::kWarn);
  EXPECT_TRUE(Contains(warned[0].text, "一直没收到心跳"));
  EXPECT_TRUE(model.Sweep(9000).empty());

  // 首帧心跳到达：报"上线"而不是"恢复在线"
  const auto online = model.OnRobotStatus(MakeRobot(), 9500);
  ASSERT_FALSE(online.empty());
  EXPECT_TRUE(Contains(online[0].text, "首帧心跳"));
  EXPECT_TRUE(model.robots().at("robot1").online);
}

TEST(MonitorCore, TextHelpers)
{
  EXPECT_EQ(StateName(kStateDocking), "对位中");
  EXPECT_EQ(RobotStateName(kRobotBusy), "执行中");
  EXPECT_EQ(ColorName(kColorBlue), "蓝");
  EXPECT_EQ(ReasonText("ROBOT_UNREACHABLE"), "机器人失联");
  EXPECT_EQ(ReasonText("SOMETHING_NEW"), "SOMETHING_NEW");
  EXPECT_EQ(ReasonText("NONE"), "无");

  EXPECT_EQ(FormatAge(500), "500ms");
  EXPECT_EQ(FormatAge(1500), "1.5s");
  EXPECT_EQ(FormatAge(65000), "1m05s");

  const std::string clock = FormatClock(1000);
  EXPECT_EQ(clock.size(), 8u);
  EXPECT_EQ(clock[2], ':');
  EXPECT_EQ(clock[5], ':');
}

}  // namespace fleet::monitor