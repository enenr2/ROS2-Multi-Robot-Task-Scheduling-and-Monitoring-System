#include "fleet_perception/ranging.hpp"

#include <cmath>

namespace fleet {

const char* ToString(Color c)
{
  switch (c) {
    case Color::kRed:   return "RED";
    case Color::kGreen: return "GREEN";
    case Color::kBlue:  return "BLUE";
    case Color::kUnknown: break;
  }
  return "UNKNOWN";
}

Ranging::Ranging(RangeConfig cfg) : cfg_(cfg) {}

RangeEstimate Ranging::Compute(const CameraModel& cam, const BoundingBox& bbox) const
{
  RangeEstimate out;

  if (!cam.IsValid()) {
    out.invalid_reason = "INVALID_CAMERA_MODEL";
    return out;
  }
  if (bbox.width < cfg_.min_bbox_width_px) {
    // 太窄：要么太远，要么只是噪点
    out.invalid_reason = "TOO_SMALL";
    return out;
  }
  if (cfg_.target_width <= 0.0) {
    out.invalid_reason = "INVALID_TARGET_WIDTH";
    return out;
  }

  // 像素宽度与距离成反比：w = fx * W / d  =>  d = fx * W / w
  const double w = static_cast<double>(bbox.width);
  const double range = cam.fx * cfg_.target_width / w;

  if (range < cfg_.min_range) {
    out.invalid_reason = "TOO_CLOSE";
    out.range = range;
    return out;
  }
  if (range > cfg_.max_range) {
    out.invalid_reason = "TOO_FAR";
    out.range = range;
    return out;
  }

  const double u = bbox.CenterU();
  const double v = bbox.CenterV();

  out.valid = true;
  out.range = range;
  // 目标在图像右侧（u > cx）时偏角为正
  out.bearing = std::atan((u - cam.cx) / cam.fx);

  // 像素宽度的相对误差直接线性传递到距离：sigma_d = d * (Δw / w)
  out.sigma = range * (cfg_.edge_sigma_px / w);

  // 反投影到相机系：x 沿光轴向前，y 向左，z 向上。
  // 图像 u 向右 => y 为负；图像 v 向下 => z 为负。
  out.point_cam.x = range;
  out.point_cam.y = -(u - cam.cx) / cam.fx * range;
  out.point_cam.z = -(v - cam.cy) / cam.fy * range;

  return out;
}

}  // namespace fleet
