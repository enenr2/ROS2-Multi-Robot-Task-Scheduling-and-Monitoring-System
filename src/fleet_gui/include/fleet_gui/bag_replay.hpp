// rosbag 回放：把 bag 里的任务快照 / 心跳 / 状态事件读成内存时间线。
//
// 监控端只看三类话题，而且是低频小消息，所以整包读进内存、按时间戳重放，
// 比一边 seek 一边反序列化简单得多（一个任务全流程也就几百条）。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fleet_gui/monitor_core.hpp"

namespace fleet::gui {

struct ReplayEntry
{
  enum class Kind { kTaskSnapshot, kRobotStatus, kTaskStatus };
  Kind kind = Kind::kTaskSnapshot;
  int64_t stamp_ms = 0;
  fleet::monitor::TaskView task;
  fleet::monitor::RobotView robot;
  fleet::monitor::StatusEvent status;
};

struct ReplayData
{
  std::vector<ReplayEntry> entries;
  std::vector<fleet::monitor::Event> events;   // 预生成的日志/告警，按时间顺序
  int64_t start_ms = 0;
  int64_t end_ms = 0;

  bool Empty() const { return entries.empty(); }
};

// uri 为 rosbag2 目录。失败时返回 false 并写入 error
bool LoadBag(const std::string& uri, ReplayData* out, std::string* error);

}  // namespace fleet::gui