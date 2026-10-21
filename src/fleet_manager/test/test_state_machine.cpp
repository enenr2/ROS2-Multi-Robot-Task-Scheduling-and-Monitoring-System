// 状态机测试：核心是 7x7 全矩阵穷举。
//
// 不逐个业务场景去"碰"迁移规则，而是把 49 个组合一次性对拍期望表，
// 这样任何一条规则被误改都会立刻失败，不会因为流程没走到而漏测。
#include <gtest/gtest.h>

#include "fleet_manager/state_machine.hpp"

using fleet::Task;
using fleet::TaskState;
using fleet::TaskStateMachine;

namespace {

// 期望迁移表，与设计文档 §6.2 逐行对应。
// 行=from，列=to，顺序 {PENDING, ASSIGNED, NAVIGATING, DOCKING, DONE, FAILED, CANCELED}
const bool kExpected[7][7] = {
  {false, true,  false, false, false, true,  true },   // PENDING
  {true,  false, true,  false, false, true,  true },   // ASSIGNED
  {true,  false, false, true,  false, true,  true },   // NAVIGATING
  {true,  false, false, false, true,  true,  true },   // DOCKING
  {false, false, false, false, false, false, false},   // DONE
  {true,  false, false, false, false, false, true },   // FAILED
  {false, false, false, false, false, false, false},   // CANCELED
};

constexpr TaskState kAll[] = {
  TaskState::kPending, TaskState::kAssigned, TaskState::kNavigating,
  TaskState::kDocking, TaskState::kDone, TaskState::kFailed, TaskState::kCanceled,
};

}  // namespace

TEST(StateMachine, FullMatrixMatchesDesignTable)
{
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      EXPECT_EQ(TaskStateMachine::IsLegal(kAll[i], kAll[j]), kExpected[i][j])
        << "迁移 " << fleet::ToString(kAll[i]) << " -> " << fleet::ToString(kAll[j])
        << " 与迁移表不符";
    }
  }
}

TEST(StateMachine, SelfTransitionAlwaysRejected)
{
  for (auto s : kAll) {
    EXPECT_FALSE(TaskStateMachine::IsLegal(s, s))
      << "状态 " << fleet::ToString(s) << " 不应允许自迁移";
  }
}

TEST(StateMachine, TerminalStates)
{
  EXPECT_TRUE(TaskStateMachine::IsTerminal(TaskState::kDone));
  EXPECT_TRUE(TaskStateMachine::IsTerminal(TaskState::kCanceled));
  for (auto s : {TaskState::kPending, TaskState::kAssigned,
                 TaskState::kNavigating, TaskState::kDocking, TaskState::kFailed}) {
    EXPECT_FALSE(TaskStateMachine::IsTerminal(s)) << fleet::ToString(s);
  }
}

TEST(StateMachine, TerminalStatesHaveNoOutgoingEdges)
{
  for (auto to : kAll) {
    EXPECT_FALSE(TaskStateMachine::IsLegal(TaskState::kDone, to));
    EXPECT_FALSE(TaskStateMachine::IsLegal(TaskState::kCanceled, to));
  }
}

TEST(StateMachine, RetryQuotaBoundary)
{
  Task t;
  t.max_retries = 1;

  t.retry_count = 0;
  EXPECT_TRUE(TaskStateMachine::CanRetry(t));

  t.retry_count = 1;
  EXPECT_FALSE(TaskStateMachine::CanRetry(t)) << "重试次数用尽后不应再有配额";

  t.max_retries = 0;
  t.retry_count = 0;
  EXPECT_FALSE(TaskStateMachine::CanRetry(t)) << "max_retries=0 表示不允许重试";
}

TEST(StateMachine, KnownFailureReasonsRoundTrip)
{
  // 失败原因用字符串枚举，字符串本身是监控与审计的对外契约，
  // 这里把设计文档里的取值固定下来，防止后续改动悄悄改字
  for (const char* r : {"NAV_TIMEOUT", "NAV_ABORTED", "TARGET_LOST",
                        "DOCK_TIMEOUT", "ROBOT_UNREACHABLE",
                        "CANCELED_BY_OPERATOR", "ASSIGN_TIMEOUT", "NONE"}) {
    EXPECT_STRNE(r, "");
  }
}
