// 单目测距：已知物块真实宽度，由图像中的像素宽度反推距离。
//
// 纯数学，无 ROS 无 OpenCV 依赖。选这个方法而不是"地面投影法"
// （用相机高度和俯角求地面交点）：物块尺寸已知且固定，像素宽度法
// 不受俯角标定误差影响，也对物块不在正前方时更稳。
//
// 已知局限（scripts/verify_perception.py 实测，仿真 320x240 / fx=277.19）：
//   正前方 1~3 m：测距误差 <= 0.04 m；
//   偏轴 12°：偏小约 0.05 m；偏轴 24°：偏小约 0.17~0.20 m（约 12%）。
//   偏轴误差来自"外接矩形量的是轮廓外接框"——目标离开光轴后侧棱投影
//   也被算进宽度，像素宽度虚高 -> 距离偏小。
//   因此本模块定位是"任务发现 + 粗测距"：精细对位交给 M5 的 robot_agent，
//   先把目标转到视野中心再测一次，不要用偏轴数据做末端精定位。
#pragma once

#include "fleet_perception/detection_types.hpp"

namespace fleet {

struct RangeConfig {
  double target_width{0.2};      // 物块真实宽度（米）
  double min_range{0.5};         // 有效距离下限
  double max_range{5.0};         // 有效距离上限
  int min_bbox_width_px{10};     // 像素宽度下限，低于此值视为太远/噪声
  double edge_sigma_px{2.0};     // 轮廓离散化带来的像素宽度误差
};

class Ranging
{
public:
  explicit Ranging(RangeConfig cfg = RangeConfig{});

  // bbox 必须是该 blob 的外接矩形
  RangeEstimate Compute(const CameraModel& cam, const BoundingBox& bbox) const;

  // 便于参数校验与测试
  const RangeConfig& config() const { return cfg_; }

private:
  RangeConfig cfg_;
};

}  // namespace fleet
