#include <gtest/gtest.h>

#include <cmath>

#include "fleet_perception/ranging.hpp"

using fleet::BoundingBox;
using fleet::CameraModel;
using fleet::RangeConfig;
using fleet::Ranging;

namespace {

// 与仿真相机一致的等效内参：320x240、60° HFOV
// fx = (320/2) / tan(30°) ≈ 277.13
CameraModel MakeCamera()
{
  CameraModel cam;
  cam.fx = 277.128;
  cam.fy = 277.128;
  cam.cx = 160.0;
  cam.cy = 120.0;
  cam.width = 320;
  cam.height = 240;
  return cam;
}

}  // namespace

TEST(Ranging, RangeFromPixelWidth)
{
  Ranging ranging;
  const CameraModel cam = MakeCamera();

  // 0.2m 宽的物块在 2m 处的像素宽度 = fx * W / d = 277.128 * 0.2 / 2 = 27.7 px
  BoundingBox bbox;
  bbox.x = 146;
  bbox.y = 100;
  bbox.width = 28;
  bbox.height = 20;

  const auto est = ranging.Compute(cam, bbox);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(est.range, 277.128 * 0.2 / 28.0, 1e-6);
  EXPECT_NEAR(est.range, 1.979, 0.01);
}

TEST(Ranging, RangeHalvesWhenWidthDoubles)
{
  Ranging ranging;
  const CameraModel cam = MakeCamera();

  BoundingBox near_box;
  near_box.width = 40;
  near_box.height = 30;
  near_box.x = 140;
  near_box.y = 100;

  BoundingBox far_box;
  far_box.width = 20;
  far_box.height = 15;
  far_box.x = 150;
  far_box.y = 110;

  const auto near_est = ranging.Compute(cam, near_box);
  const auto far_est = ranging.Compute(cam, far_box);
  ASSERT_TRUE(near_est.valid);
  ASSERT_TRUE(far_est.valid);
  EXPECT_NEAR(far_est.range, near_est.range * 2.0, 1e-6);
}

TEST(Ranging, RejectsTooSmallBBox)
{
  Ranging ranging;
  BoundingBox bbox;
  bbox.width = 5;
  bbox.height = 5;

  const auto est = ranging.Compute(MakeCamera(), bbox);
  EXPECT_FALSE(est.valid);
  EXPECT_EQ(est.invalid_reason, "TOO_SMALL");
}

TEST(Ranging, RejectsTooClose)
{
  RangeConfig cfg;
  cfg.min_range = 0.5;
  Ranging ranging(cfg);

  BoundingBox bbox;
  bbox.width = 200;  // 0.2*277/200 = 0.277m < 0.5
  bbox.height = 150;

  const auto est = ranging.Compute(MakeCamera(), bbox);
  EXPECT_FALSE(est.valid);
  EXPECT_EQ(est.invalid_reason, "TOO_CLOSE");
  EXPECT_NEAR(est.range, 277.128 * 0.2 / 200.0, 1e-6);
}

TEST(Ranging, RejectsTooFar)
{
  RangeConfig cfg;
  cfg.max_range = 5.0;
  cfg.min_bbox_width_px = 1;
  Ranging ranging(cfg);

  BoundingBox bbox;
  bbox.width = 8;  // 277.128*0.2/8 = 6.93m > 5
  bbox.height = 6;

  const auto est = ranging.Compute(MakeCamera(), bbox);
  EXPECT_FALSE(est.valid);
  EXPECT_EQ(est.invalid_reason, "TOO_FAR");
}

TEST(Ranging, RejectsInvalidCameraModel)
{
  Ranging ranging;
  CameraModel cam;  // fx=0
  BoundingBox bbox;
  bbox.width = 30;
  bbox.height = 20;

  const auto est = ranging.Compute(cam, bbox);
  EXPECT_FALSE(est.valid);
  EXPECT_EQ(est.invalid_reason, "INVALID_CAMERA_MODEL");
}

TEST(Ranging, SigmaGrowsWithRange)
{
  Ranging ranging;
  const CameraModel cam = MakeCamera();

  BoundingBox near_box;
  near_box.width = 40;
  near_box.height = 30;
  near_box.x = 140;
  near_box.y = 105;

  BoundingBox far_box;
  far_box.width = 20;
  far_box.height = 15;
  far_box.x = 150;
  far_box.y = 112;

  const auto near_est = ranging.Compute(cam, near_box);
  const auto far_est = ranging.Compute(cam, far_box);
  ASSERT_TRUE(near_est.valid);
  ASSERT_TRUE(far_est.valid);

  // sigma = d * Δw / w，距离翻倍、宽度减半 => sigma 变为 4 倍
  EXPECT_NEAR(far_est.sigma, near_est.sigma * 4.0, 1e-9);
  EXPECT_NEAR(near_est.sigma, near_est.range * 2.0 / 40.0, 1e-9);
}

TEST(Ranging, BearingIsZeroAtCenter)
{
  Ranging ranging;
  BoundingBox bbox;
  bbox.x = 150;  // 中心 150+10 = 160 = cx
  bbox.width = 20;
  bbox.height = 16;
  bbox.y = 112;  // 中心 112+8 = 120 = cy

  const auto est = ranging.Compute(MakeCamera(), bbox);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(est.bearing, 0.0, 1e-9);
  EXPECT_NEAR(est.point_cam.y, 0.0, 1e-6);
  EXPECT_NEAR(est.point_cam.z, 0.0, 1e-3);
}

TEST(Ranging, BearingPositiveToTheRight)
{
  Ranging ranging;
  BoundingBox bbox;
  bbox.x = 220;  // 中心 230，在 cx 右侧
  bbox.width = 20;
  bbox.height = 15;
  bbox.y = 112;

  const auto est = ranging.Compute(MakeCamera(), bbox);
  ASSERT_TRUE(est.valid);
  EXPECT_GT(est.bearing, 0.0);
  EXPECT_NEAR(est.bearing, std::atan((230.0 - 160.0) / 277.128), 1e-9);
  // 相机系 y 向左为正，所以右侧目标 y 为负
  EXPECT_LT(est.point_cam.y, 0.0);
}

TEST(Ranging, BackProjectionMatchesRange)
{
  Ranging ranging;
  BoundingBox bbox;
  bbox.x = 120;
  bbox.y = 90;
  bbox.width = 30;
  bbox.height = 24;

  const auto est = ranging.Compute(MakeCamera(), bbox);
  ASSERT_TRUE(est.valid);

  // 反投影点沿光轴的分量应等于测距距离
  EXPECT_NEAR(est.point_cam.x, est.range, 1e-9);
  EXPECT_NEAR(est.point_cam.y / est.point_cam.x, std::tan(-est.bearing), 1e-6);
}
