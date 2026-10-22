#include "fleet_gui/bag_replay.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "fleet_msgs/msg/robot_status.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/task_status.hpp"

namespace fleet::gui {

namespace {

int64_t BagStampMs(const rosbag2_storage::SerializedBagMessage& msg)
{
  // sqlite3 后端只存一列 timestamp（读出来在 recv_timestamp），send_timestamp
  // 恒为 0；mcap 两个都存。统一优先用 send，缺失时退回 recv，否则 sqlite3
  // 录的包时间轴会全是 0
  const rcutils_time_point_value_t stamp =
    msg.send_timestamp > 0 ? msg.send_timestamp : msg.recv_timestamp;
  return static_cast<int64_t>(stamp / 1000000);
}

int64_t ToMs(const builtin_interfaces::msg::Time& t)
{
  return static_cast<int64_t>(t.sec) * 1000 + static_cast<int64_t>(t.nanosec) / 1000000;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

template<typename T>
bool Deserialize(const rosbag2_storage::SerializedBagMessage& msg, T* out)
{
  const size_t size = msg.serialized_data->buffer_length;
  rclcpp::SerializedMessage serialized(size);
  auto& raw = serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer, msg.serialized_data->buffer, size);
  raw.buffer_length = size;
  try {
    rclcpp::Serialization<T> serialization;
    serialization.deserialize_message(&serialized, out);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

bool SplitRobotTopic(const std::string& topic, std::string* robot, std::string* kind)
{
  if (topic.size() < 4 || topic.front() != '/') {
    return false;
  }
  const size_t second = topic.find('/', 1);
  if (second == std::string::npos || second <= 1) {
    return false;
  }
  if (topic.find('/', second + 1) != std::string::npos) {
    return false;
  }
  *robot = topic.substr(1, second - 1);
  *kind = topic.substr(second + 1);
  return true;
}

}  // namespace

bool LoadBag(const std::string& uri, ReplayData* out, std::string* error)
{
  out->entries.clear();
  out->events.clear();
  out->start_ms = 0;
  out->end_ms = 0;

  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = "";   // 由 metadata.yaml 决定 sqlite3 / mcap
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception& ex) {
    *error = std::string("打开 bag 失败：") + ex.what();
    return false;
  }

  std::vector<std::string> topics{"/fleet/tasks"};
  std::ostringstream available;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    available << topic.name << " ";
    std::string robot;
    std::string kind;
    if (!SplitRobotTopic(topic.name, &robot, &kind)) {
      continue;
    }
    if (kind == "robot_status" || kind == "task_status") {
      topics.push_back(topic.name);
    }
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics = topics;
  reader.set_filter(filter);

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (!bag_msg) {
      break;
    }
    const int64_t stamp_ms = BagStampMs(*bag_msg);

    ReplayEntry entry;
    entry.stamp_ms = stamp_ms;

    if (bag_msg->topic_name == "/fleet/tasks") {
      fleet_msgs::msg::Task msg;
      if (!Deserialize(*bag_msg, &msg)) {
        continue;
      }
      entry.kind = ReplayEntry::Kind::kTaskSnapshot;
      entry.task.task_id = msg.task_id;
      entry.task.color = msg.target_color;
      entry.task.state = msg.state;
      entry.task.x = msg.target_pose.position.x;
      entry.task.y = msg.target_pose.position.y;
      entry.task.robot = msg.assigned_robot_id;
      entry.task.failure_reason = msg.failure_reason;
      entry.task.retry = msg.retry_count;
      entry.task.max_retry = msg.max_retries;
      entry.task.detected_ms = ToMs(msg.detected_at);
      entry.task.assigned_ms = ToMs(msg.assigned_at);
      // 回放时间轴与 bag 录制时间同源，表格时间列用录制时间才有意义
      entry.task.first_seen_ms = stamp_ms;
      entry.task.revision = msg.revision;
    } else {
      std::string robot;
      std::string kind;
      if (!SplitRobotTopic(bag_msg->topic_name, &robot, &kind)) {
        continue;
      }
      if (kind == "robot_status") {
        fleet_msgs::msg::RobotStatus msg;
        if (!Deserialize(*bag_msg, &msg)) {
          continue;
        }
        entry.kind = ReplayEntry::Kind::kRobotStatus;
        entry.robot.robot_id = msg.robot_id.empty() ? robot : msg.robot_id;
        entry.robot.state = msg.state;
        entry.robot.x = msg.pose.position.x;
        entry.robot.y = msg.pose.position.y;
        entry.robot.yaw = YawFromQuaternion(msg.pose.orientation);
        entry.robot.current_task = msg.current_task;
        entry.robot.tasks_done = msg.tasks_done;
        entry.robot.tasks_failed = msg.tasks_failed;
        entry.robot.tasks_canceled = msg.tasks_canceled;
        entry.robot.heartbeat_seq = msg.heartbeat_seq;
      } else if (kind == "task_status") {
        fleet_msgs::msg::TaskStatus msg;
        if (!Deserialize(*bag_msg, &msg)) {
          continue;
        }
        entry.kind = ReplayEntry::Kind::kTaskStatus;
        entry.status.task_id = msg.task_id;
        entry.status.robot_id = msg.robot_id.empty() ? robot : msg.robot_id;
        entry.status.state = msg.state;
        entry.status.progress = msg.progress;
        entry.status.detail = msg.detail;
        entry.status.failure_reason = msg.failure_reason;
        entry.status.retry = msg.retry_count;
        entry.status.stamp_ms = stamp_ms;
      } else {
        continue;
      }
    }
    out->entries.push_back(std::move(entry));
  }

  if (out->entries.empty()) {
    *error = "bag 中没有任务/心跳/状态话题。可用话题：" + available.str();
    return false;
  }

  // 不同存储后端的跨话题顺序不一定严格单调，统一按录制时间排序
  std::stable_sort(out->entries.begin(), out->entries.end(),
                   [](const ReplayEntry& a, const ReplayEntry& b) {
                     return a.stamp_ms < b.stamp_ms;
                   });
  out->start_ms = out->entries.front().stamp_ms;
  out->end_ms = out->entries.back().stamp_ms;

  // 用同一个纯逻辑模型把日志/告警预生成出来，回放时只按时间截断
  fleet::monitor::MonitorModel model;
  for (const auto& entry : out->entries) {
    std::vector<fleet::monitor::Event> events;
    if (entry.kind == ReplayEntry::Kind::kTaskSnapshot) {
      events = model.OnTaskSnapshot(entry.task, entry.stamp_ms);
    } else if (entry.kind == ReplayEntry::Kind::kRobotStatus) {
      events = model.OnRobotStatus(entry.robot, entry.stamp_ms);
    } else {
      events = model.OnTaskStatus(entry.status, entry.stamp_ms);
    }
    out->events.insert(out->events.end(), events.begin(), events.end());
  }
  return true;
}

}  // namespace fleet::gui