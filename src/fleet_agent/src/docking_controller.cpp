#include "fleet_agent/docking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace fleet {
namespace {

double Clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace

DockingController::DockingController(DockingConfig cfg) : cfg_(cfg) {}

void DockingController::Reset(TimeNs now)
{
  start_ = now;
  last_seen_ = 0;
  ever_seen_ = false;
}

DockingOutput DockingController::Update(const DockingInput& in, TimeNs now)
{
  DockingOutput out;

  const double elapsed_s = static_cast<double>(now - start_) / static_cast<double>(kSec);

  if (!in.target_visible) {
    out.heading_error = 0.0;
    // 从"最后一次看到"开始计时；从未看到过则从任务开始时计时，
    // 否则目标一开始就不在视野里会永远等下去
    const TimeNs ref = ever_seen_ ? last_seen_ : start_;
    const double missing_s = static_cast<double>(now - ref) / static_cast<double>(kSec);
    if (missing_s >= cfg_.target_lost_timeout_s) {
      out.target_lost = true;
    }
    if (elapsed_s >= cfg_.timeout_s) {
      out.timed_out = true;
    }
    return out;
  }

  ever_seen_ = true;
  last_seen_ = now;

  out.heading_error = in.bearing;
  out.distance_error = in.range - cfg_.standoff;

  // 先对准：偏角超容差时只转不走，避免把目标甩出视野
  if (std::fabs(in.bearing) > cfg_.bearing_tolerance) {
    // bearing > 0 表示目标在右侧 -> 需要顺时针转 -> 角速度为负
    out.angular = Clamp(-cfg_.yaw_gain * in.bearing, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    out.linear = 0.0;
    return CheckTimeout(out, elapsed_s);
  }

  if (std::fabs(out.distance_error) <= cfg_.distance_tolerance) {
    out.done = true;
    return CheckTimeout(out, elapsed_s);
  }

  // 边靠近边修偏：偏角已在容差内，修正量很小
  out.angular = Clamp(-cfg_.yaw_gain * in.bearing, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);

  if (out.distance_error > 0.0) {
    out.linear = Clamp(cfg_.linear_gain * out.distance_error, 0.0, cfg_.max_linear_vel);
  } else {
    // 冲过头了：小幅倒车，不超过 reverse_vel
    out.linear = -std::min(cfg_.reverse_vel, cfg_.linear_gain * std::fabs(out.distance_error));
  }

  return CheckTimeout(out, elapsed_s);
}

DockingOutput DockingController::CheckTimeout(DockingOutput out, double elapsed_s) const
{
  // 已完成就不再判超时：完成是更强的结论
  if (!out.done && elapsed_s >= cfg_.timeout_s) {
    out.timed_out = true;
    out.linear = 0.0;
    out.angular = 0.0;
  }
  return out;
}

}  // namespace fleet
