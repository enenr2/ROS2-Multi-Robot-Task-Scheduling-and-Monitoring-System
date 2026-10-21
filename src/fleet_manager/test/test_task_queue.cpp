// 任务表测试：去重、迁移、归属、重排配额、revision 递增。
#include <gtest/gtest.h>

#include "fleet_manager/task_queue.hpp"

using namespace fleet;

namespace {

constexpr TimeNs kT0 = 1000 * kSecond;

Pose2D P(double x, double y) { return Pose2D{x, y, 0.0}; }

}  // namespace

TEST(TaskQueue, AddNewTask)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(2.0, 1.0), kT0));

  const Task* t = q.Find("t1");
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->state, TaskState::kPending);
  EXPECT_EQ(t->color, Color::kRed);
  EXPECT_EQ(t->retry_count, 0);
  EXPECT_TRUE(t->assigned_robot.empty());
  EXPECT_EQ(t->revision, 1u) << "创建即算一次变更";
  EXPECT_EQ(q.Size(), 1u);
}

TEST(TaskQueue, RejectDuplicateId)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  EXPECT_FALSE(q.AddTask("t1", Color::kGreen, P(5.0, 5.0), kT0));
  EXPECT_EQ(q.Size(), 1u);
}

TEST(TaskQueue, DedupSameColorWithinRadius)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(2.0, 1.0), kT0));

  // 同色、距离 0.1m < 0.5m → 视为同一物块
  EXPECT_FALSE(q.AddTask("t2", Color::kRed, P(2.1, 1.0), kT0));
  EXPECT_EQ(q.Size(), 1u);
  EXPECT_TRUE(q.FindDuplicate(Color::kRed, P(2.05, 1.0)).has_value());
}

TEST(TaskQueue, DedupRadiusBoundary)
{
  TaskQueueConfig cfg;
  cfg.dedup_radius = 0.5;
  TaskQueue q(cfg);
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));

  // 正好等于阈值：判定用严格小于，因此不视为重复
  EXPECT_FALSE(q.FindDuplicate(Color::kRed, P(0.5, 0.0)).has_value());
  // 略小于阈值：视为重复
  EXPECT_TRUE(q.FindDuplicate(Color::kRed, P(0.49, 0.0)).has_value());
}

TEST(TaskQueue, SameColorFarApartIsNotDuplicate)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  EXPECT_TRUE(q.AddTask("t2", Color::kRed, P(3.0, 0.0), kT0));
  EXPECT_EQ(q.Size(), 2u);
}

TEST(TaskQueue, DifferentColorSamePlaceIsNotDuplicate)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(1.0, 1.0), kT0));
  EXPECT_TRUE(q.AddTask("t2", Color::kGreen, P(1.0, 1.0), kT0))
    << "去重必须同时匹配颜色，否则不同色物块会被吞掉";
  EXPECT_EQ(q.Size(), 2u);
}

TEST(TaskQueue, CanceledTaskAllowsRediscovery)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(1.0, 1.0), kT0));
  ASSERT_TRUE(q.Cancel("t1", kT0, "CANCELED_BY_OPERATOR"));
  // 人工取消后，同一物块应能被重新发现建任务
  EXPECT_TRUE(q.AddTask("t2", Color::kRed, P(1.0, 1.0), kT0));
}

TEST(TaskQueue, RevisionIncrementsOnEveryChange)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  EXPECT_EQ(q.Find("t1")->revision, 1u);

  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
  EXPECT_EQ(q.Find("t1")->revision, 2u);

  ASSERT_TRUE(q.Transition("t1", TaskState::kNavigating, kT0));
  EXPECT_EQ(q.Find("t1")->revision, 3u);
}

TEST(TaskQueue, RejectIllegalTransition)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));

  // PENDING -> NAVIGATING 非法：必须先 ASSIGNED
  EXPECT_FALSE(q.Transition("t1", TaskState::kNavigating, kT0));
  EXPECT_EQ(q.Find("t1")->state, TaskState::kPending);
  EXPECT_EQ(q.Find("t1")->revision, 1u) << "被拒绝的迁移不应改动 revision";
}

TEST(TaskQueue, TransitionOnMissingTaskFails)
{
  TaskQueue q;
  EXPECT_FALSE(q.Transition("nope", TaskState::kAssigned, kT0));
  EXPECT_FALSE(q.AssignTo("nope", "robot1", kT0));
  EXPECT_FALSE(q.Cancel("nope", kT0, "x"));
  EXPECT_FALSE(q.RetryOrFail("nope", kT0, "x"));
  EXPECT_EQ(q.Find("nope"), nullptr);
}

TEST(TaskQueue, AssignOnlyFromPending)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));

  // 未分配时直接推进到 NAVIGATING 再尝试分配，应被拒绝
  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
  EXPECT_FALSE(q.AssignTo("t1", "robot2", kT0)) << "一车一任务，不可重复分配";
  EXPECT_EQ(q.Find("t1")->assigned_robot, "robot1");

  EXPECT_FALSE(q.AssignTo("t1", "", kT0)) << "空 robot_id 非法";
}

TEST(TaskQueue, AssignRecordsTimestamp)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0 + 5 * kSecond));
  EXPECT_EQ(q.Find("t1")->assigned_at, kT0 + 5 * kSecond);
  EXPECT_EQ(q.Find("t1")->state, TaskState::kAssigned);
}

TEST(TaskQueue, RetryKeepsQuotaThenFails)
{
  TaskQueueConfig cfg;
  cfg.default_max_retries = 1;
  TaskQueue q(cfg);
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
  ASSERT_TRUE(q.Transition("t1", TaskState::kNavigating, kT0));

  // 第一次重排：有配额，回到 PENDING 且清空归属
  ASSERT_TRUE(q.RetryOrFail("t1", kT0, "NAV_TIMEOUT"));
  EXPECT_EQ(q.Find("t1")->state, TaskState::kPending);
  EXPECT_EQ(q.Find("t1")->retry_count, 1);
  EXPECT_TRUE(q.Find("t1")->assigned_robot.empty());
  EXPECT_EQ(q.Find("t1")->assigned_at, 0) << "重排后旧分配时间必须清掉";
  EXPECT_EQ(q.Find("t1")->failure_reason, "NAV_TIMEOUT");

  // 第二次重排：配额用尽，终态 FAILED
  ASSERT_TRUE(q.AssignTo("t1", "robot2", kT0));
  ASSERT_TRUE(q.Transition("t1", TaskState::kNavigating, kT0));
  EXPECT_FALSE(q.RetryOrFail("t1", kT0, "NAV_TIMEOUT"));
  EXPECT_EQ(q.Find("t1")->state, TaskState::kFailed);
  EXPECT_EQ(q.Find("t1")->retry_count, 1) << "终态失败不再增加重试计数";
  EXPECT_EQ(q.Find("t1")->failure_reason, "NAV_TIMEOUT");
}

TEST(TaskQueue, FailedTaskCanRetryBeforeQuotaExhausted)
{
  TaskQueueConfig cfg;
  cfg.default_max_retries = 2;
  TaskQueue q(cfg);
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
  ASSERT_TRUE(q.Transition("t1", TaskState::kNavigating, kT0));

  ASSERT_TRUE(q.RetryOrFail("t1", kT0, "NAV_ABORTED"));
  ASSERT_EQ(q.Find("t1")->state, TaskState::kPending);
}

TEST(TaskQueue, CancelFromEachNonTerminalState)
{
  for (auto st : {TaskState::kPending, TaskState::kAssigned,
                  TaskState::kNavigating, TaskState::kDocking}) {
    TaskQueue q;
    ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
    if (st != TaskState::kPending) {
      ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
    }
    if (st == TaskState::kNavigating || st == TaskState::kDocking) {
      ASSERT_TRUE(q.Transition("t1", TaskState::kNavigating, kT0));
    }
    if (st == TaskState::kDocking) {
      ASSERT_TRUE(q.Transition("t1", TaskState::kDocking, kT0));
    }
    ASSERT_EQ(q.Find("t1")->state, st);

    EXPECT_TRUE(q.Cancel("t1", kT0, "CANCELED_BY_OPERATOR"))
      << "状态 " << ToString(st) << " 应可取消";
    EXPECT_EQ(q.Find("t1")->state, TaskState::kCanceled);
    EXPECT_EQ(q.Find("t1")->failure_reason, "CANCELED_BY_OPERATOR");
  }
}

TEST(TaskQueue, CancelTerminalStateRejected)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(0.0, 0.0), kT0));
  ASSERT_TRUE(q.Cancel("t1", kT0, "CANCELED_BY_OPERATOR"));
  EXPECT_FALSE(q.Cancel("t1", kT0, "again")) << "终态任务不可重复取消";
}

TEST(TaskQueue, ActiveTargetsOnlyIncludesInFlightTasks)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("t1", Color::kRed, P(1.0, 0.0), kT0));
  ASSERT_TRUE(q.AddTask("t2", Color::kGreen, P(5.0, 0.0), kT0));
  ASSERT_TRUE(q.AddTask("t3", Color::kBlue, P(9.0, 0.0), kT0));

  EXPECT_TRUE(q.ActiveTargets().empty()) << "PENDING 尚未占用目标";

  ASSERT_TRUE(q.AssignTo("t1", "robot1", kT0));
  ASSERT_TRUE(q.AssignTo("t2", "robot2", kT0));
  ASSERT_TRUE(q.Cancel("t2", kT0, "CANCELED_BY_OPERATOR"));
  ASSERT_TRUE(q.Transition("t3", TaskState::kFailed, kT0, "ASSIGN_TIMEOUT"));

  auto active = q.ActiveTargets();
  ASSERT_EQ(active.size(), 1u);
  EXPECT_DOUBLE_EQ(active[0].x, 1.0) << "只有 t1 在飞行中";
}

TEST(TaskQueue, ByStateFiltersAndKeepsDeterministicOrder)
{
  TaskQueue q;
  ASSERT_TRUE(q.AddTask("tb", Color::kRed, P(0.0, 0.0), kT0));
  ASSERT_TRUE(q.AddTask("ta", Color::kGreen, P(3.0, 0.0), kT0));
  ASSERT_TRUE(q.AddTask("tc", Color::kBlue, P(6.0, 0.0), kT0));
  ASSERT_TRUE(q.AssignTo("tc", "robot1", kT0));

  auto pending = q.ByState(TaskState::kPending);
  ASSERT_EQ(pending.size(), 2u);
  // map 有序，输出按 id 字典序 —— 回放比对依赖这一点
  EXPECT_EQ(pending[0].id, "ta");
  EXPECT_EQ(pending[1].id, "tb");

  auto all = q.All();
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0].id, "ta");
  EXPECT_EQ(all[1].id, "tb");
  EXPECT_EQ(all[2].id, "tc");
}
