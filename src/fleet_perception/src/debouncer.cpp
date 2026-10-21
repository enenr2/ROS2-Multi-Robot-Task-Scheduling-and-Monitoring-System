#include "fleet_perception/debouncer.hpp"

#include <cmath>

namespace fleet {

DetectionDebouncer::DetectionDebouncer(DebounceConfig cfg) : cfg_(cfg) {}

void DetectionDebouncer::Reset()
{
  streak_ = 0;
  missing_ = 0;
  lost_reported_ = false;
  acquired_ = false;
  last_u_ = last_v_ = last_range_ = 0.0;
}

DebounceState DetectionDebouncer::Update(bool has_blob, double centroid_u,
                                        double centroid_v, double range)
{
  DebounceState st;

  if (!has_blob) {
    streak_ = 0;
    missing_ += 1;
    st.streak = 0;
    st.missing = missing_;

    // 只上报一次"丢失"，避免连续丢帧时刷出一串同样的告警；
    // 从未捕获过目标时不上报（启动噪声）
    if (acquired_ && missing_ >= cfg_.lost_after_missing_frames && !lost_reported_) {
      st.lost = true;
      lost_reported_ = true;
    }
    return st;
  }

  missing_ = 0;
  lost_reported_ = false;
  acquired_ = true;

  const bool consistent =
    streak_ > 0 &&
    std::fabs(centroid_u - last_u_) <= cfg_.centroid_tolerance_px &&
    std::fabs(centroid_v - last_v_) <= cfg_.centroid_tolerance_px &&
    std::fabs(range - last_range_) <= cfg_.range_tolerance_m;

  if (consistent) {
    streak_ += 1;
  } else {
    // 首帧或跳动超限：以本帧为新基准重新累计
    streak_ = 1;
  }

  last_u_ = centroid_u;
  last_v_ = centroid_v;
  last_range_ = range;

  st.streak = streak_;
  st.missing = 0;
  st.emit = streak_ >= cfg_.required_frames;
  return st;
}

}  // namespace fleet
