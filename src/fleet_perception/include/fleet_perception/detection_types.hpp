// 感知域数据类型：不含 ROS 头文件，也不含 OpenCV 头文件。
//
// 这样测距与去抖逻辑可以被纯 C++ 测试覆盖，也能在 M5 的视觉对位里
// 复用而不必把 OpenCV 拖进 robot_agent。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fleet {

// 颜色编号与 fleet_msgs/Detection 的常量保持一致（1=红 2=绿 3=蓝）
enum class Color : uint8_t {
  kUnknown = 0,
  kRed = 1,
  kGreen = 2,
  kBlue = 3,
};

const char* ToString(Color c);

// 像素域包围盒，原点在图像左上角，u 向右 v 向下
struct BoundingBox {
  int x{0};
  int y{0};
  int width{0};
  int height{0};

  double CenterU() const { return x + width / 2.0; }
  double CenterV() const { return y + height / 2.0; }
  int Area() const { return width * height; }
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// 相机内参 + 图像尺寸
struct CameraModel {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool IsValid() const { return fx > 0.0 && fy > 0.0 && width > 0 && height > 0; }
};

// 单帧颜色检测结果
struct Blob {
  Color color{Color::kUnknown};
  BoundingBox bbox;
  int area{0};
  // 轮廓面积 / 参考面积，>1 表示目标比参考更近。仅供调度做趋势判断，
  // 不用于测距（测距用像素宽度，抗透视变化更好）
  double confidence{1.0};
};

// 测距结果
struct RangeEstimate {
  bool valid{false};
  double range{0.0};        // 沿相机光轴的距离（米）
  double bearing{0.0};      // 相对光轴的水平偏角（弧度，目标在右为正）
  double sigma{0.0};        // 测距不确定度（米），随距离线性增大
  Vec3 point_cam;           // 目标在相机系下的位置：x 前、y 左、z 上
  std::string invalid_reason;
};

}  // namespace fleet
