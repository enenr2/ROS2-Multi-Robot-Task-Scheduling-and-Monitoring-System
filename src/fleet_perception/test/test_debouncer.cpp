#include <gtest/gtest.h>

#include "fleet_perception/debouncer.hpp"

using fleet::DebounceConfig;
using fleet::DetectionDebouncer;

namespace {

DebounceConfig MakeConfig()
{
  DebounceConfig cfg;
  cfg.required_frames = 3;
  cfg.centroid_tolerance_px = 10.0;
  cfg.range_tolerance_m = 0.15;
  cfg.lost_after_missing_frames = 15;
  return cfg;
}

}  // namespace

TEST(Debouncer, EmitsOnlyAfterRequiredFrames)
{
  DetectionDebouncer db(MakeConfig());

  EXPECT_FALSE(db.Update(true, 100.0, 100.0, 2.0).emit);
  EXPECT_FALSE(db.Update(true, 101.0, 100.0, 2.0).emit);

  const auto third = db.Update(true, 102.0, 101.0, 2.05);
  EXPECT_TRUE(third.emit);
  EXPECT_EQ(third.streak, 3);

  // 之后每帧都继续上报，不再重新累计
  const auto fourth = db.Update(true, 103.0, 101.0, 2.05);
  EXPECT_TRUE(fourth.emit);
  EXPECT_EQ(fourth.streak, 4);
}

TEST(Debouncer, JitterResetsStreak)
{
  DetectionDebouncer db(MakeConfig());

  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 101.0, 100.0, 2.0);

  // 质心跳 50px，超出容差：本帧作为新基准，streak 归 1
  const auto jumped = db.Update(true, 150.0, 100.0, 2.0);
  EXPECT_FALSE(jumped.emit);
  EXPECT_EQ(jumped.streak, 1);

  db.Update(true, 151.0, 100.0, 2.0);
  EXPECT_TRUE(db.Update(true, 152.0, 100.0, 2.0).emit);
}

TEST(Debouncer, RangeJitterResetsStreak)
{
  DetectionDebouncer db(MakeConfig());

  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 100.0, 100.0, 2.0);

  const auto jumped = db.Update(true, 100.0, 100.0, 2.5);  // 0.5m > 0.15m
  EXPECT_FALSE(jumped.emit);
  EXPECT_EQ(jumped.streak, 1);
}

TEST(Debouncer, MissingFramesResetStreak)
{
  DetectionDebouncer db(MakeConfig());

  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 100.0, 100.0, 2.0);

  const auto miss = db.Update(false, 0.0, 0.0, 0.0);
  EXPECT_EQ(miss.streak, 0);
  EXPECT_EQ(miss.missing, 1);
  EXPECT_FALSE(miss.lost);

  // 重新出现需要重新累计 3 帧
  EXPECT_FALSE(db.Update(true, 100.0, 100.0, 2.0).emit);
  EXPECT_FALSE(db.Update(true, 100.0, 100.0, 2.0).emit);
  EXPECT_TRUE(db.Update(true, 100.0, 100.0, 2.0).emit);
}

TEST(Debouncer, ReportsLostOnceAfterThreshold)
{
  DetectionDebouncer db(MakeConfig());

  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 100.0, 100.0, 2.0);

  int lost_count = 0;
  for (int i = 0; i < 20; ++i) {
    const auto st = db.Update(false, 0.0, 0.0, 0.0);
    if (st.lost) {
      lost_count++;
      EXPECT_EQ(st.missing, 15);
    }
  }
  EXPECT_EQ(lost_count, 1);
}

TEST(Debouncer, DoesNotReportLostBeforeFirstAcquisition)
{
  DetectionDebouncer db(MakeConfig());

  // 启动时相机里什么都没有：连丢 40 帧也不该冒出"目标丢失"
  bool any_lost = false;
  for (int i = 0; i < 40; ++i) {
    any_lost = db.Update(false, 0.0, 0.0, 0.0).lost || any_lost;
  }
  EXPECT_FALSE(any_lost);
}

TEST(Debouncer, RediscoveryAfterLostResetsFlag)
{
  DetectionDebouncer db(MakeConfig());

  for (int i = 0; i < 15; ++i) {
    db.Update(false, 0.0, 0.0, 0.0);
  }
  // 目标重新出现并稳定
  db.Update(true, 50.0, 50.0, 3.0);
  db.Update(true, 50.0, 50.0, 3.0);
  EXPECT_TRUE(db.Update(true, 50.0, 50.0, 3.0).emit);

  // 再次丢失时应重新上报一次
  bool lost_again = false;
  for (int i = 0; i < 15; ++i) {
    lost_again = db.Update(false, 0.0, 0.0, 0.0).lost || lost_again;
  }
  EXPECT_TRUE(lost_again);
}

TEST(Debouncer, ResetClearsState)
{
  DetectionDebouncer db(MakeConfig());

  db.Update(true, 100.0, 100.0, 2.0);
  db.Update(true, 100.0, 100.0, 2.0);
  db.Reset();

  const auto st = db.Update(true, 100.0, 100.0, 2.0);
  EXPECT_EQ(st.streak, 1);
  EXPECT_FALSE(st.emit);
  EXPECT_EQ(st.missing, 0);
}
