// 调度域的纯数据类型：不引用任何 ROS 头文件。
//
// 时间统一用纳秒整数，而不是 rclcpp::Time —— 这样逻辑层不绑定 ROS，
// 测试可以直接传常数时间点。与 builtin_interfaces/Time 的互转放在
// M6 的 ROS 适配层。
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace fleet {

using TaskId = std::string;
using RobotId = std::string;
using TimeNs = int64_t;

inline constexpr TimeNs kSecond = 1000000000LL;
inline constexpr TimeNs kMilliSecond = 1000000LL;

// 与 fleet_msgs/Detection 的颜色常量保持一致（1=红 2=绿 3=蓝）
enum class Color : uint8_t {
  kUnknown = 0,
  kRed = 1,
  kGreen = 2,
  kBlue = 3,
};

// 与 fleet_msgs/TaskStatus 的状态常量保持一致
enum class TaskState : uint8_t {
  kPending = 0,
  kAssigned = 1,
  kNavigating = 2,
  kDocking = 3,
  kDone = 4,
  kFailed = 5,
  kCanceled = 6,
};

enum class RobotState : uint8_t {
  kIdle = 0,
  kBusy = 1,
  kError = 2,
  kOffline = 3,
};

const char* ToString(TaskState s);
const char* ToString(RobotState s);
const char* ToString(Color c);

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double Distance2D(const Pose2D& a, const Pose2D& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

struct Task {
  TaskId id;
  Color color{Color::kUnknown};
  Pose2D target;

  TaskState state{TaskState::kPending};
  // 归属唯一：分配在通信层天然互斥（调度器直调该车的 action）
  RobotId assigned_robot;
  TimeNs assigned_at{0};

  TimeNs detected_at{0};
  uint8_t retry_count{0};
  uint8_t max_retries{1};

  // 每次字段变更 +1。用途是让监控端判断消息时序、丢弃过期包，
  // 不是并发控制手段。
  uint64_t revision{0};

  std::string failure_reason;
};

struct Robot {
  RobotId id;
  RobotState state{RobotState::kIdle};
  Pose2D pose;

  TaskId current_task;
  uint16_t tasks_done{0};
  uint16_t tasks_failed{0};

  TimeNs last_heartbeat{0};
  uint32_t heartbeat_seq{0};
};

}  // namespace fleet
