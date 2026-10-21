#include "fleet_perception/color_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>

namespace fleet {
namespace {

cv::Scalar ToScalar(const HsvRange& r)
{
  return cv::Scalar(r.h_lo, r.s_lo, r.v_lo);
}

cv::Scalar ToScalarUpper(const HsvRange& r)
{
  return cv::Scalar(r.h_hi, r.s_hi, r.v_hi);
}

std::string ToUpper(const std::string& s)
{
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return out;
}

}  // namespace

ColorDetector::ColorDetector(DetectorConfig cfg) : cfg_(std::move(cfg)) {}

DetectorConfig ColorDetector::DefaultConfig()
{
  DetectorConfig cfg;

  // 室内白光下的实测阈值。饱和度下限设得较高（>=90）是为了
  // 避开灰白地面与墙壁——它们 H 值不稳定，但 S 很低。
  ColorRule red;
  red.color = Color::kRed;
  red.primary = HsvRange{0, 10, 90, 255, 60, 255};
  red.use_secondary = true;
  red.secondary = HsvRange{170, 180, 90, 255, 60, 255};

  ColorRule green;
  green.color = Color::kGreen;
  green.primary = HsvRange{40, 85, 80, 255, 50, 255};

  ColorRule blue;
  blue.color = Color::kBlue;
  blue.primary = HsvRange{95, 135, 90, 255, 50, 255};

  cfg.rules = {red, green, blue};
  return cfg;
}

std::vector<Blob> ColorDetector::Detect(const cv::Mat& bgr) const
{
  std::vector<Blob> out;
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return out;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  const int k = std::max(1, cfg_.morph_kernel);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));

  for (const ColorRule& rule : cfg_.rules) {
    cv::Mat mask;
    cv::inRange(hsv, ToScalar(rule.primary), ToScalarUpper(rule.primary), mask);
    if (rule.use_secondary) {
      cv::Mat mask2;
      cv::inRange(hsv, ToScalar(rule.secondary), ToScalarUpper(rule.secondary), mask2);
      cv::bitwise_or(mask, mask2, mask);
    }

    // 先开后闭：开运算打掉孤立噪点，闭运算把目标内部的孔洞补上
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int best_area = 0;
    cv::Rect best_rect;
    for (const auto& contour : contours) {
      const int area = static_cast<int>(cv::contourArea(contour));
      if (area < cfg_.min_area_px || area <= best_area) {
        continue;
      }
      best_area = area;
      best_rect = cv::boundingRect(contour);
    }

    if (best_area <= 0 || best_rect.width <= 0 || best_rect.height <= 0) {
      continue;
    }

    Blob blob;
    blob.color = rule.color;
    blob.bbox = BoundingBox{best_rect.x, best_rect.y, best_rect.width, best_rect.height};
    blob.area = best_area;
    blob.confidence = cfg_.reference_area_px > 0.0
                        ? static_cast<double>(best_area) / cfg_.reference_area_px
                        : 1.0;
    out.push_back(blob);
  }

  return out;
}

bool ColorDetector::ParseColor(const std::string& name, Color* out)
{
  if (out == nullptr) {
    return false;
  }
  const std::string upper = ToUpper(name);
  if (upper == "RED") {
    *out = Color::kRed;
    return true;
  }
  if (upper == "GREEN") {
    *out = Color::kGreen;
    return true;
  }
  if (upper == "BLUE") {
    *out = Color::kBlue;
    return true;
  }
  return false;
}

}  // namespace fleet
