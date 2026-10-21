// 颜色检测：HSV 阈值分割 + 形态学去噪 + 轮廓筛选。
//
// 依赖 OpenCV，但不依赖 ROS：可以脱离仿真用合成图直接单测。
// 检测策略是"每色只报一个目标"——一个颜色在场内同时出现多块时会选面积最大者，
// 这对当前"三色各一块"的任务场景足够；真要多目标再扩展成多 blob 返回。
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "fleet_perception/detection_types.hpp"

namespace fleet {

// OpenCV 的 HSV 空间中 H ∈ [0,180]、S/V ∈ [0,255]。
// 红色跨越 0°，所以需要两段（primary + secondary）取并集。
struct HsvRange {
  int h_lo{0};
  int h_hi{180};
  int s_lo{0};
  int s_hi{255};
  int v_lo{0};
  int v_hi{255};
};

struct ColorRule {
  Color color{Color::kUnknown};
  HsvRange primary;
  bool use_secondary{false};
  HsvRange secondary;
};

struct DetectorConfig {
  std::vector<ColorRule> rules;
  int min_area_px{60};            // 轮廓面积下限，滤掉噪点
  int morph_kernel{3};            // 形态学核尺寸（开运算去噪、闭运算补洞）
  double reference_area_px{900.0};// 置信度参考面积：area / reference
};

class ColorDetector
{
public:
  explicit ColorDetector(DetectorConfig cfg = DefaultConfig());

  // bgr 必须是 CV_8UC3；空图或非法格式返回空结果
  std::vector<Blob> Detect(const cv::Mat& bgr) const;

  const DetectorConfig& config() const { return cfg_; }

  // 三个颜色的默认阈值：红色双段，绿/蓝单段
  static DetectorConfig DefaultConfig();

  // "RED"/"GREEN"/"BLUE" -> Color，大小写不敏感；无法识别返回 false
  static bool ParseColor(const std::string& name, Color* out);

private:
  DetectorConfig cfg_;
};

}  // namespace fleet
