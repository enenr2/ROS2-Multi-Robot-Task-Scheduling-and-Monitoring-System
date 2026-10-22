// 监控端纯逻辑：任务表合并、心跳判活、日志与告警生成。
//
// 不依赖 ROS / Qt —— 这里的边界情况最容易在真机上出错：
//   * 任务快照 2 Hz 全量重发，旧包晚到不能覆盖新状态（用 revision 判序）
//   * 重复的状态上报不能刷屏（同一任务同一状态只记一次）
//   * 心跳 1 Hz，抖动时不能来回报"离线/上线"（超过超时才判离线）
// 用注入时间戳的单测覆盖这些点，比开仿真点鼠标快得多。
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace fleet::monitor {

// 与 TaskStatus.msg 的 STATE_* 保持一致
constexpr int kStatePending = 0;
constexpr int kStateAssigned = 1;
constexpr int kStateNavigating = 2;
constexpr int kStateDocking = 3;
constexpr int kStateDone = 4;
constexpr int kStateFailed = 5;
constexpr int kStateCanceled = 6;

// 与 RobotStatus.msg 的 STATE_* 保持一致
constexpr int kRobotIdle = 0;
constexpr int kRobotBusy = 1;
constexpr int kRobotError = 2;
constexpr int kRobotOffline = 3;

// 与 Detection/Task 的 target_color 一致
constexpr int kColorRed = 1;
constexpr int kColorGreen = 2;
constexpr int kColorBlue = 3;

enum class Severity { kInfo = 0, kWarn = 1, kError = 2 };

// 任务快照（由 fleet_msgs/Task 转换）
struct TaskView
{
  std::string task_id;
  int color = 0;
  int state = kStatePending;
  double x = 0.0;
  double y = 0.0;
  std::string robot;             // assigned_robot_id，空串表示未分配
  std::string failure_reason;    // NONE / NAV_TIMEOUT / ...
  int retry = 0;
  int max_retry = 0;
  int64_t detected_ms = 0;
  int64_t assigned_ms = 0;
  // 监控端第一次看到该任务的时间（本端时钟）。仿真里 detected_at 是 sim time，
  // 与界面上的墙钟日志不同源，表格统一用这个字段展示
  int64_t first_seen_ms = 0;
  // 进入终态的本端时间；用于清理历史任务，避免表格与内存无限增长
  int64_t terminal_ms = 0;
  uint64_t revision = 0;
};

// 机器人心跳（由 fleet_msgs/RobotStatus 转换）
struct RobotView
{
  std::string robot_id;
  int state = kRobotIdle;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string current_task;
  int tasks_done = 0;
  int tasks_failed = 0;
  int tasks_canceled = 0;
  uint32_t heartbeat_seq = 0;
  int64_t last_seen_ms = 0;   // 本端收到心跳的本地时间
  bool online = false;        // 由 MonitorModel 按心跳间隔判定
  bool heartbeat_seen = false;  // 是否至少收到过一帧心跳（区分"未上线"与"掉线"）
};

// 任务状态事件（由 fleet_msgs/TaskStatus 转换）
struct StatusEvent
{
  std::string task_id;
  std::string robot_id;
  int state = kStatePending;
  double progress = 0.0;
  std::string detail;
  std::string failure_reason;
  int retry = 0;
  int64_t stamp_ms = 0;
};

// 日志/告警条目。severity >= kWarn 的同时进告警页
struct Event
{
  int64_t stamp_ms = 0;
  std::string robot_id;
  std::string text;
  Severity severity = Severity::kInfo;
};

// 栅格地图快照（由 nav_msgs/OccupancyGrid 转换），供地图视图绘制
struct MapData
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> cells;   // -1 未知，0 空闲，100 占用
  bool Empty() const { return width <= 0 || height <= 0 || cells.empty(); }
};

bool IsActiveState(int state);
bool IsTerminalState(int state);

std::string StateName(int state);
std::string RobotStateName(int state);
std::string ColorName(int color);
std::string ReasonText(const std::string& reason);
std::string FormatClock(int64_t ms);   // 本地时区 HH:MM:SS
std::string FormatAge(int64_t ms);     // 850ms / 3.2s / 2m05s

// 监控数据模型：所有输入都是"已转换好的快照"，输出是本次需要记的日志/告警。
class MonitorModel
{
public:
  explicit MonitorModel(int64_t offline_timeout_ms = 2500);

  // 返回本次产生的事件（可能为空）
  std::vector<Event> OnTaskSnapshot(const TaskView& task, int64_t now_ms);
  std::vector<Event> OnRobotStatus(const RobotView& robot, int64_t now_ms);
  std::vector<Event> OnTaskStatus(const StatusEvent& status, int64_t now_ms);
  // 发现车的话题后先登记：agent 从未发过心跳（进程没起/被冻住）时，
  // 界面上也要有一行"离线"，否则运维根本看不出少了一台车
  std::vector<Event> EnsureRobot(const std::string& robot_id, int64_t now_ms);
  // 周期调用：把心跳超时的车标离线。返回"刚判定离线"的事件
  std::vector<Event> Sweep(int64_t now_ms);
  // 清理过期的终态任务：保留最近 max_terminal 个，且不超过 keep_ms。
  // 只删终态，活跃任务永远保留。返回删除条数。
  std::size_t PruneTasks(int64_t now_ms, std::size_t max_terminal = 300,
                         int64_t keep_ms = 600000);

  void Clear();

  const std::map<std::string, TaskView>& tasks() const { return tasks_; }
  const std::map<std::string, RobotView>& robots() const { return robots_; }
  // 活跃任务在前，终态在后；同组内新任务在前
  std::vector<TaskView> SortedTasks() const;
  double Progress(const std::string& task_id) const;
  std::string Detail(const std::string& task_id) const;

  int ActiveCount() const;
  int DoneCount() const;
  int FailedCount() const;

  int64_t offline_timeout_ms() const { return offline_timeout_ms_; }

private:
  // 同一任务的状态只在真正跳变时产出一条事件
  std::vector<Event> Announce(const std::string& task_id, const std::string& robot,
                              int state, const std::string& reason,
                              const std::string& detail, int64_t now_ms);
  static Severity SeverityOf(int state);

  int64_t offline_timeout_ms_;
  std::map<std::string, TaskView> tasks_;
  std::map<std::string, RobotView> robots_;
  std::map<std::string, int> last_state_;   // task_id -> 已播报过的状态
  std::map<std::string, double> progress_;  // task_id -> 0..1
  std::map<std::string, std::string> detail_;
  std::map<std::string, int> retry_;        // task_id -> 已播报过的重试次数
  std::set<std::string> ever_seen_;         // 收到过心跳的车
  std::set<std::string> no_heartbeat_alarmed_;   // 已为"从未上线"告警过的车
};

}  // namespace fleet::monitor