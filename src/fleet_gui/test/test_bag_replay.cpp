// 回放固化：真的写一个 sqlite3 bag，再用 LoadBag 读回来。
//
// 之前只有"拿一段手录 bag 点开界面"的验证，机器上没 bag 就测不到。
// 这个用例自己造数据，覆盖：跨话题排序、时间轴起止、日志/告警预生成。
#include <gtest/gtest.h>

#include <rcutils/allocator.h>
#include <rcutils/types/uint8_array.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "fleet_gui/bag_replay.hpp"
#include "fleet_msgs/msg/robot_status.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/task_status.hpp"

namespace {

namespace fs = std::filesystem;

// 每个用例一个独立临时目录，析构时清理
class TempDir
{
public:
  TempDir()
  {
    path_ = fs::temp_directory_path() /
            ("fleet_bag_test_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::remove_all(path_);
  }
  ~TempDir() { fs::remove_all(path_); }

  std::string Path() const { return path_.string(); }
  const fs::path & operator*() const { return path_; }

private:
  fs::path path_;
};

template<typename T>
void WriteMessage(rosbag2_cpp::writers::SequentialWriter& writer, const std::string& topic,
                  const T& message, int64_t stamp_ns)
{
  rclcpp::Serialization<T> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);

  const auto& raw = serialized.get_rcl_serialized_message();
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic;
  bag_message->send_timestamp = stamp_ns;
  bag_message->recv_timestamp = stamp_ns;
  bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  auto allocator = rcutils_get_default_allocator();
  if (rcutils_uint8_array_init(bag_message->serialized_data.get(), raw.buffer_length,
                               &allocator) != RCUTILS_RET_OK)
  {
    throw std::runtime_error("rcutils_uint8_array_init 失败");
  }
  std::memcpy(bag_message->serialized_data->buffer, raw.buffer, raw.buffer_length);
  bag_message->serialized_data->buffer_length = raw.buffer_length;
  writer.write(bag_message);
}

void CreateTopic(rosbag2_cpp::writers::SequentialWriter& writer, const std::string& name,
                 const std::string& type)
{
  rosbag2_storage::TopicMetadata metadata;
  metadata.name = name;
  metadata.type = type;
  metadata.serialization_format = "cdr";
  writer.create_topic(metadata);
}

int64_t Ms(int64_t ms) { return ms * 1000000; }

}  // namespace

TEST(BagReplay, SortsTopicsAndPrecomputesEvents)
{
  TempDir dir;

  {
    rosbag2_cpp::writers::SequentialWriter writer;
    rosbag2_storage::StorageOptions storage;
    storage.uri = dir.Path();
    storage.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter;
    converter.input_serialization_format = "cdr";
    converter.output_serialization_format = "cdr";
    writer.open(storage, converter);

    CreateTopic(writer, "/fleet/tasks", "fleet_msgs/msg/Task");
    CreateTopic(writer, "/robot1/robot_status", "fleet_msgs/msg/RobotStatus");
    CreateTopic(writer, "/robot1/task_status", "fleet_msgs/msg/TaskStatus");

    fleet_msgs::msg::Task task;
    task.task_id = "T1";
    task.type = fleet_msgs::msg::Task::TYPE_APPROACH;
    task.target_color = 1;
    task.state = 0;
    task.target_pose.position.x = 3.0;
    task.target_pose.position.y = 1.0;
    task.revision = 1;
    task.max_retries = 1;
    task.failure_reason = "NONE";
    WriteMessage(writer, "/fleet/tasks", task, Ms(1000));

    fleet_msgs::msg::RobotStatus robot;
    robot.robot_id = "robot1";
    robot.state = fleet_msgs::msg::RobotStatus::STATE_IDLE;
    robot.pose.orientation.w = 1.0;
    robot.heartbeat_seq = 1;
    WriteMessage(writer, "/robot1/robot_status", robot, Ms(1200));

    fleet_msgs::msg::TaskStatus status;
    status.task_id = "T1";
    status.robot_id = "robot1";
    status.state = fleet_msgs::msg::TaskStatus::STATE_ASSIGNED;
    status.detail = "任务已接受";
    status.failure_reason = "NONE";
    WriteMessage(writer, "/robot1/task_status", status, Ms(1400));

    task.revision = 3;
    task.state = fleet_msgs::msg::TaskStatus::STATE_DONE;
    task.assigned_robot_id = "robot1";
    WriteMessage(writer, "/fleet/tasks", task, Ms(2000));

    status.state = fleet_msgs::msg::TaskStatus::STATE_DONE;
    status.progress = 1.0f;
    status.detail = "任务完成";
    WriteMessage(writer, "/robot1/task_status", status, Ms(3000));

    writer.close();
  }

  fleet::gui::ReplayData data;
  std::string error;
  ASSERT_TRUE(fleet::gui::LoadBag(dir.Path(), &data, &error)) << error;

  ASSERT_EQ(data.entries.size(), 5u);
  EXPECT_EQ(data.start_ms, 1000);
  EXPECT_EQ(data.end_ms, 3000);

  // 跨话题按录制时间排序，时间轴拖动才能得到一致的状态
  for (size_t i = 1; i < data.entries.size(); ++i) {
    EXPECT_LE(data.entries[i - 1].stamp_ms, data.entries[i].stamp_ms);
  }

  EXPECT_FALSE(data.events.empty());
  bool saw_new_task = false;
  bool saw_robot_online = false;
  for (const auto& event : data.events) {
    saw_new_task = saw_new_task || event.text.find("新任务 T1") != std::string::npos;
    saw_robot_online = saw_robot_online || event.text.find("robot1 上线") != std::string::npos;
  }
  EXPECT_TRUE(saw_new_task);
  EXPECT_TRUE(saw_robot_online);

  EXPECT_EQ(data.entries.back().kind, fleet::gui::ReplayEntry::Kind::kTaskStatus);

  const fleet::gui::ReplayEntry* last_task = nullptr;
  for (const auto& entry : data.entries) {
    if (entry.kind == fleet::gui::ReplayEntry::Kind::kTaskSnapshot) {
      last_task = &entry;
    }
  }
  ASSERT_NE(last_task, nullptr);
  EXPECT_EQ(last_task->task.task_id, "T1");
  EXPECT_EQ(last_task->task.state, fleet::monitor::kStateDone);
}

TEST(BagReplay, EmptyDirectoryFailsWithMessage)
{
  TempDir dir;
  std::filesystem::create_directories(*dir);

  fleet::gui::ReplayData data;
  std::string error;
  EXPECT_FALSE(fleet::gui::LoadBag(dir.Path(), &data, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(data.Empty());
}