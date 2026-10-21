// 时序去抖：同一颜色连续多帧位置一致才认为目标稳定，才允许上报。
//
// 不加这层的话，轮廓抖动会让调度中心在同一个物块上反复建任务、
// 又反复判定"目标移动"。判定逻辑与 ROS 无关，可逐帧驱动测试。
#pragma once

#include <cstdint>
#include <string>

namespace fleet {

struct DebounceConfig {
  int required_frames{3};            // 连续多少帧一致才上报
  double centroid_tolerance_px{10.0};// 质心允许的帧间跳动
  double range_tolerance_m{0.15};    // 距离允许的帧间跳动
  int lost_after_missing_frames{15}; // 连续丢帧超过此值判定目标丢失
};

struct DebounceState {
  bool emit{false};        // 本帧是否应上报
  bool lost{false};        // 本帧是否应判为目标丢失（仅上报一次）
  int streak{0};           // 当前连续一致帧数
  int missing{0};          // 当前连续丢帧数
};

class DetectionDebouncer
{
public:
  explicit DetectionDebouncer(DebounceConfig cfg = DebounceConfig{});

  // has_blob=false 表示本帧该颜色没有候选
  DebounceState Update(bool has_blob, double centroid_u, double centroid_v, double range);

  void Reset();

  const DebounceConfig& config() const { return cfg_; }

private:
  DebounceConfig cfg_;
  int streak_{0};
  int missing_{0};
  bool lost_reported_{false};
  // 从未见过目标时不该报"丢失"：节点刚启动、相机还没对准时每帧都是无目标，
  // 若不加这个门闩，上线头 15 帧就会刷三条假的丢失告警
  bool acquired_{false};
  double last_u_{0.0};
  double last_v_{0.0};
  double last_range_{0.0};
};

}  // namespace fleet
