// 分配策略测试：最近空闲优先、平局字典序、目标互斥、无车可用。
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "fleet_manager/assignment.hpp"

using namespace fleet;

namespace {

Pose2D P(double x, double y) { return Pose2D{x, y, 0.0}; }

Robot MakeRobot(const std::string& id, double x, double y,
                RobotState st = RobotState::kIdle)
{
  Robot r;
  r.id = id;
  r.pose = P(x, y);
  r.state = st;
  return r;
}

}  // namespace

TEST(Assignment, PicksNearestIdleRobot)
{
  Assignment a;
  std::vector<Robot> robots = {
    MakeRobot("robot1", 1.0, 0.0),   // 距目标 3.0
    MakeRobot("robot2", 3.0, 0.0),   // 距目标 1.0  ← 最近
    MakeRobot("robot3", 8.0, 0.0),   // 距目标 4.0
  };
  auto d = a.SelectRobot(P(4.0, 0.0), robots, {});
  EXPECT_EQ(d.robot, "robot2");
  EXPECT_FALSE(d.blocked_by_target_exclusion);
}

TEST(Assignment, SkipsBusyErrorAndOfflineRobots)
{
  Assignment a;
  std::vector<Robot> robots = {
    MakeRobot("robot1", 3.1, 0.0, RobotState::kBusy),     // 最近但在忙
    MakeRobot("robot2", 3.5, 0.0, RobotState::kError),
    MakeRobot("robot3", 3.6, 0.0, RobotState::kOffline),
    MakeRobot("robot4", 6.0, 0.0),                        // 唯一空闲
  };
  auto d = a.SelectRobot(P(3.0, 0.0), robots, {});
  EXPECT_EQ(d.robot, "robot4");
}

TEST(Assignment, NoIdleRobotReturnsEmpty)
{
  Assignment a;
  std::vector<Robot> robots = {
    MakeRobot("robot1", 1.0, 0.0, RobotState::kBusy),
    MakeRobot("robot2", 2.0, 0.0, RobotState::kError),
  };
  auto d = a.SelectRobot(P(0.0, 0.0), robots, {});
  EXPECT_FALSE(d.HasRobot());
  EXPECT_FALSE(d.blocked_by_target_exclusion)
    << "无车可用与目标互斥是两种不同原因，不能混淆";
}

TEST(Assignment, EmptyRobotListReturnsEmpty)
{
  Assignment a;
  auto d = a.SelectRobot(P(0.0, 0.0), {}, {});
  EXPECT_FALSE(d.HasRobot());
}

TEST(Assignment, TieBreakByRobotIdLexicographic)
{
  Assignment a;
  // 两台车与目标等距；故意把 robot2 放在前面，验证结果不受输入顺序影响
  std::vector<Robot> order1 = {
    MakeRobot("robot2", 0.0, 1.0),
    MakeRobot("robot1", 0.0, -1.0),
  };
  std::vector<Robot> order2 = {
    MakeRobot("robot1", 0.0, -1.0),
    MakeRobot("robot2", 0.0, 1.0),
  };
  EXPECT_EQ(a.SelectRobot(P(0.0, 0.0), order1, {}).robot, "robot1");
  EXPECT_EQ(a.SelectRobot(P(0.0, 0.0), order2, {}).robot, "robot1");

  // 三位数 id 的字典序陷阱：robot10 < robot9（字符串比较，不是数值比较）
  std::vector<Robot> multi = {
    MakeRobot("robot9", 0.0, 1.0),
    MakeRobot("robot10", 0.0, -1.0),
  };
  EXPECT_EQ(a.SelectRobot(P(0.0, 0.0), multi, {}).robot, "robot10")
    << "id 是字符串，robot10 字典序小于 robot9";
}

TEST(Assignment, TargetExclusionWithinRadius)
{
  Assignment a;   // 默认 exclusion=1.0
  std::vector<Robot> robots = {MakeRobot("robot1", 1.0, 1.0)};

  // 已有任务目标在 (3.0,0.0)，新目标 (3.4,0.0) 距离 0.4 < 1.0 → 阻塞
  auto d = a.SelectRobot(P(3.4, 0.0), robots, {P(3.0, 0.0)});
  EXPECT_FALSE(d.HasRobot());
  EXPECT_TRUE(d.blocked_by_target_exclusion);
}

TEST(Assignment, TargetExclusionBoundaryNotBlocked)
{
  Assignment a;
  std::vector<Robot> robots = {MakeRobot("robot1", 1.0, 1.0)};

  // 正好 1.0m：判定用严格小于，因此不阻塞
  auto d = a.SelectRobot(P(2.0, 0.0), robots, {P(1.0, 0.0)});
  EXPECT_TRUE(d.HasRobot());
  EXPECT_FALSE(d.blocked_by_target_exclusion);
}

TEST(Assignment, TargetExclusionBeyondRadiusProceeds)
{
  Assignment a;
  std::vector<Robot> robots = {MakeRobot("robot1", 1.0, 1.0)};
  auto d = a.SelectRobot(P(0.0, 5.0), robots, {P(1.0, 0.0)});
  EXPECT_EQ(d.robot, "robot1");
}

TEST(Assignment, TargetExclusionChecksAllActiveTargets)
{
  Assignment a;
  std::vector<Robot> robots = {MakeRobot("robot1", 0.0, 0.0)};
  // 与第一个目标相距很远，但与第二个目标很近
  auto d = a.SelectRobot(P(6.0, 0.0), robots, {P(0.0, 0.0), P(6.2, 0.0)});
  EXPECT_TRUE(d.blocked_by_target_exclusion);
}

TEST(Assignment, CustomExclusionRadius)
{
  AssignConfig cfg;
  cfg.target_exclusion_radius = 2.0;
  Assignment a(cfg);
  std::vector<Robot> robots = {MakeRobot("robot1", 0.0, 0.0)};

  // 1.5m 在默认阈值下不阻塞，2.0 阈值下阻塞
  auto d = a.SelectRobot(P(1.5, 0.0), robots, {P(0.0, 0.0)});
  EXPECT_TRUE(d.blocked_by_target_exclusion);
}

TEST(Assignment, IsolationCheckMatchesSelectRobot)
{
  Assignment a;
  EXPECT_TRUE(a.IsTargetBlocked(P(0.5, 0.0), {P(0.0, 0.0)}));
  EXPECT_FALSE(a.IsTargetBlocked(P(1.5, 0.0), {P(0.0, 0.0)}));
  EXPECT_FALSE(a.IsTargetBlocked(P(0.0, 0.0), {}));
}

TEST(Assignment, DeterministicAcrossRepeatedCalls)
{
  Assignment a;
  std::vector<Robot> robots = {
    MakeRobot("robot3", 2.0, 0.0),
    MakeRobot("robot1", 2.0, 0.0),
    MakeRobot("robot2", 2.0, 0.0),
  };
  const std::string first = a.SelectRobot(P(0.0, 0.0), robots, {}).robot;
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(a.SelectRobot(P(0.0, 0.0), robots, {}).robot, first)
      << "同输入必须同输出，否则回放测试无法比对";
  }
  EXPECT_EQ(first, "robot1");
}
