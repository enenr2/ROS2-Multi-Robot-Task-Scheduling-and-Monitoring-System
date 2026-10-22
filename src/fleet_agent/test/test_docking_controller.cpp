#include <gtest/gtest.h>

#include <cmath>

#include "fleet_agent/agent_geometry.hpp"
#include "fleet_agent/docking_controller.hpp"

using fleet::ApproachResult;
using fleet::ComputeApproach;
using fleet::DockingConfig;
using fleet::DockingController;
using fleet::DockingInput;
using fleet::Pose2D;
using fleet::kSec;

namespace {

DockingConfig MakeConfig()
{
  DockingConfig cfg;
  cfg.standoff = 0.6;
  cfg.distance_tolerance = 0.10;
  cfg.bearing_tolerance = 0.05;
  cfg.yaw_gain = 1.0;
  cfg.max_yaw_rate = 0.5;
  cfg.linear_gain = 0.8;
  cfg.max_linear_vel = 0.15;
  cfg.reverse_vel = 0.05;
  cfg.target_lost_timeout_s = 3.0;
  cfg.timeout_s = 20.0;
  return cfg;
}

DockingInput Visible(double bearing, double range)
{
  DockingInput in;
  in.target_visible = true;
  in.bearing = bearing;
  in.range = range;
  return in;
}

}  // namespace

// ---------------- 任务级 standoff ----------------

TEST(AgentGeometry, ResolveStandoffFallsBackAndClamps)
{
  // goal 未指定（0/负数/NaN）时用机器人默认值
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(0.0, 0.6), 0.6);
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(-0.3, 0.6), 0.6);
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(std::nan(""), 0.6), 0.6);

  // 任务级配置优先
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(0.4, 0.6), 0.4);

  // 越界收紧
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(9.0, 0.6), fleet::kMaxStandoff);
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(0.01, 0.6), fleet::kMinStandoff);

  // 默认值本身越界也要收
  EXPECT_DOUBLE_EQ(fleet::ResolveStandoff(0.0, 5.0), fleet::kMaxStandoff);
}

// ---------------- 接近点 ----------------

TEST(AgentGeometry, ApproachStopsShortAndFacesTarget)
{
  const Pose2D robot{0.0, 0.0, 0.0};
  const Pose2D target{3.0, 0.0, 0.0};

  const ApproachResult r = ComputeApproach(robot, target, 1.2);
  EXPECT_FALSE(r.already_close);
  EXPECT_NEAR(r.pose.x, 1.8, 1e-9);   // 3.0 - 1.2
  EXPECT_NEAR(r.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(r.pose.yaw, 0.0, 1e-9);
  EXPECT_NEAR(r.distance_to_target, 3.0, 1e-9);
}

TEST(AgentGeometry, ApproachKeepsHeadingTowardTargetOnDiagonal)
{
  const Pose2D robot{0.0, 0.0, 0.0};
  const Pose2D target{3.0, 3.0, 0.0};

  const ApproachResult r = ComputeApproach(robot, target, 1.0);
  const double d = std::hypot(3.0 - r.pose.x, 3.0 - r.pose.y);
  EXPECT_NEAR(d, 1.0, 1e-9);
  EXPECT_NEAR(r.pose.yaw, M_PI / 4.0, 1e-9);
}

TEST(AgentGeometry, AlreadyCloseDoesNotBackOff)
{
  const Pose2D robot{0.0, 0.0, 0.0};
  const Pose2D target{0.8, 0.0, 0.0};

  const ApproachResult r = ComputeApproach(robot, target, 1.2);
  EXPECT_TRUE(r.already_close);
  EXPECT_NEAR(r.pose.x, 0.0, 1e-9);   // 不后退
  EXPECT_NEAR(r.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(r.pose.yaw, 0.0, 1e-9);
}

TEST(AgentGeometry, AlreadyCloseStillTurnsTowardTarget)
{
  const Pose2D robot{0.0, 0.0, 3.0};   // 朝向 -x
  const Pose2D target{0.0, 0.8, 0.0};  // 目标在 +y

  const ApproachResult r = ComputeApproach(robot, target, 1.2);
  EXPECT_TRUE(r.already_close);
  EXPECT_NEAR(r.pose.yaw, M_PI / 2.0, 1e-9);
}

// ---------------- 入库控制 ----------------

TEST(Docking, TurnsInPlaceWhenBearingLarge)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  // 目标在右侧 20°，距离 2 m：应当只转不走
  const auto out = dock.Update(Visible(0.35, 2.0), kSec);
  EXPECT_LT(out.angular, 0.0);   // 右转 = 负角速度
  EXPECT_DOUBLE_EQ(out.linear, 0.0);
  EXPECT_FALSE(out.done);
}

TEST(Docking, YawRateIsClamped)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  const auto out = dock.Update(Visible(-1.2, 2.0), kSec);  // 左 68°
  EXPECT_GT(out.angular, 0.0);
  EXPECT_NEAR(out.angular, 0.5, 1e-9);   // max_yaw_rate
}

TEST(Docking, DrivesForwardWhenAlignedAndFar)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  // 距离误差 0.15 m：速度 = gain * err = 0.12，未被上限截断
  const auto out = dock.Update(Visible(0.0, 0.75), kSec);
  EXPECT_FALSE(out.done);
  EXPECT_DOUBLE_EQ(out.angular, 0.0);
  EXPECT_NEAR(out.linear, 0.8 * 0.15, 1e-9);
  EXPECT_NEAR(out.distance_error, 0.15, 1e-9);
}

TEST(Docking, ForwardSpeedIsClamped)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  const auto out = dock.Update(Visible(0.0, 5.0), kSec);
  EXPECT_NEAR(out.linear, 0.15, 1e-9);   // max_linear_vel
}

TEST(Docking, ReversesWhenTooClose)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  const auto out = dock.Update(Visible(0.0, 0.30), kSec);
  EXPECT_LT(out.linear, 0.0);
  EXPECT_NEAR(out.linear, -0.05, 1e-9);   // reverse_vel 上限
}

TEST(Docking, DoneWhenInsideTolerance)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  const auto out = dock.Update(Visible(0.01, 0.62), kSec);
  EXPECT_TRUE(out.done);
  EXPECT_DOUBLE_EQ(out.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.angular, 0.0);
}

TEST(Docking, NotDoneWhileBearingOutsideToleranceEvenIfRangeOk)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  const auto out = dock.Update(Visible(0.10, 0.60), kSec);
  EXPECT_FALSE(out.done);   // 距离够近但没对准，仍要转
  EXPECT_LT(out.angular, 0.0);
}

TEST(Docking, DoesNotReportLostBeforeEverSeen)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  DockingInput none;
  EXPECT_FALSE(dock.Update(none, 1 * kSec).target_lost);
  EXPECT_FALSE(dock.Update(none, 2 * kSec).target_lost);
  // 从任务开始计时，超过 3 s 才判丢失
  EXPECT_TRUE(dock.Update(none, 3 * kSec).target_lost);
}

TEST(Docking, LostTimerRestartsFromLastSeen)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);

  dock.Update(Visible(0.0, 1.0), 5 * kSec);   // 5 s 时还看得到

  DockingInput none;
  EXPECT_FALSE(dock.Update(none, 7 * kSec).target_lost);   // 丢 2 s
  EXPECT_TRUE(dock.Update(none, 9 * kSec).target_lost);    // 丢 4 s
}

TEST(Docking, TimeoutStopsCommand)
{
  DockingConfig cfg = MakeConfig();
  cfg.timeout_s = 5.0;
  DockingController dock(cfg);
  dock.Reset(0);

  EXPECT_FALSE(dock.Update(Visible(0.0, 3.0), 4 * kSec).timed_out);

  const auto out = dock.Update(Visible(0.0, 3.0), 6 * kSec);
  EXPECT_TRUE(out.timed_out);
  EXPECT_DOUBLE_EQ(out.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.angular, 0.0);
}

TEST(Docking, CompletionWinsOverTimeout)
{
  DockingConfig cfg = MakeConfig();
  cfg.timeout_s = 5.0;
  DockingController dock(cfg);
  dock.Reset(0);

  const auto out = dock.Update(Visible(0.0, 0.6), 6 * kSec);
  EXPECT_TRUE(out.done);
  EXPECT_FALSE(out.timed_out);
}

TEST(Docking, ResetClearsTimers)
{
  DockingController dock(MakeConfig());
  dock.Reset(0);
  EXPECT_TRUE(dock.Update(DockingInput{}, 10 * kSec).target_lost);

  dock.Reset(10 * kSec);
  EXPECT_FALSE(dock.Update(DockingInput{}, 11 * kSec).target_lost);
  EXPECT_FALSE(dock.Update(DockingInput{}, 12 * kSec).target_lost);
}
