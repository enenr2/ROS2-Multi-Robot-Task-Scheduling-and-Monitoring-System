// 视觉精对位控制：把"目标在画面里的偏角和距离"变成速度指令。
//
// 为什么不用 Nav2 直接开到物块前：Nav2 的目标是地图上的一个点，误差受定位
// 与代价地图分辨率限制；物块只有 0.2 m，且调度要求"停到 0.6 m 处正对目标"。
// 所以导航只负责把车送到目标附近并让目标进视野，最后这段用视觉闭环。
//
// 控制律分三段，顺序不能换：
//   1. 偏角超出容差 -> 原地转（先对准，避免边走边转把目标甩出视野）
//   2. 距离超出容差 -> 直行 + 小幅修偏
//   3. 都进容差 -> 停车完成
// 帧率只有约 4 Hz，所以限速压得很低（0.15 m/s），否则会冲过头振荡。
//
// 纯逻辑、无 ROS 依赖，用注入时间驱动，可直接跑单测。
#pragma once

#include <cstdint>

namespace fleet {

using TimeNs = int64_t;

inline constexpr TimeNs kSec = 1000000000LL;

struct DockingConfig {
  double standoff{0.6};              // 期望停车距离（米）
  double distance_tolerance{0.10};   // 距离容差（米）
  double bearing_tolerance{0.05};    // 偏角容差（弧度，约 2.9°）
  double yaw_gain{1.0};              // 每弧度偏角对应的角速度
  double max_yaw_rate{0.5};          // 角速度上限（rad/s）
  double linear_gain{0.8};           // 每米距离误差对应的线速度
  double max_linear_vel{0.15};       // 线速度上限（m/s），受相机帧率限制
  double reverse_vel{0.05};          // 过近时倒车速度
  double target_lost_timeout_s{3.0}; // 连续看不到目标多久判定丢失
  double timeout_s{20.0};            // 整个对位阶段超时
};

struct DockingInput {
  bool target_visible{false};
  double bearing{0.0};   // 相对光轴的水平偏角，目标在右为正（弧度）
  double range{0.0};     // 单目测距结果（米）
};

struct DockingOutput {
  double linear{0.0};    // m/s
  double angular{0.0};   // rad/s，逆时针为正
  bool done{false};
  bool target_lost{false};
  bool timed_out{false};
  double distance_error{0.0};   // range - standoff
  double heading_error{0.0};    // 等于输入的 bearing
};

class DockingController
{
public:
  explicit DockingController(DockingConfig cfg = DockingConfig{});

  // 每次任务开始时调用：清空计时与"是否见过目标"的标记
  void Reset(TimeNs now);

  DockingOutput Update(const DockingInput& in, TimeNs now);

  const DockingConfig& config() const { return cfg_; }
  void set_config(const DockingConfig& cfg) { cfg_ = cfg; }

private:
  DockingOutput CheckTimeout(DockingOutput out, double elapsed_s) const;

  DockingConfig cfg_;
  TimeNs start_{0};
  TimeNs last_seen_{0};
  bool ever_seen_{false};
};

}  // namespace fleet
