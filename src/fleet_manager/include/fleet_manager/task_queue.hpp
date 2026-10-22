// 任务表：去重、状态迁移、归属、重排。
//
// 不依赖 ROS，时间由调用方传入，内部不加锁 —— 单线程推进，
// 由上层保证"回调只投递事件、状态变更集中在单线程处理"这一约定。
#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "fleet_manager/state_machine.hpp"
#include "fleet_manager/task_types.hpp"

namespace fleet {

struct TaskQueueConfig {
  // 同色且目标相距小于此值 → 视为同一物块，不再重复建任务。
  // 只按颜色和空间距离判定，不看是哪台车发现的，避免两车分别
  // 看到同一物块后各建一个任务。
  double dedup_radius{0.5};
  uint8_t default_max_retries{1};
  // 终态任务保留策略：活跃任务永不清理；终态任务超过 age 或数量上限就删除。
  // 不清理会让去重误伤"同色目标再次出现"，内存与监控快照也会无限增长。
  TimeNs retention_age{600 * kSecond};
  std::size_t max_terminal{500};
};

class TaskQueue
{
public:
  explicit TaskQueue(TaskQueueConfig cfg = TaskQueueConfig{});

  // 命中已有目标时返回其 id；没有重复目标返回 nullopt
  std::optional<TaskId> FindDuplicate(Color color, const Pose2D& target) const;

  // 新建任务。若与已有目标重复则拒绝创建并返回 false。
  // 调用方需自行保证 id 唯一；重复 id 也会被拒绝。
  bool AddTask(const TaskId& id, Color color, const Pose2D& target, TimeNs detected_at);

  // 状态迁移。非法迁移或任务不存在都返回 false，任务保持不变。
  // 成功时 revision +1。
  bool Transition(const TaskId& id, TaskState to, TimeNs now,
                  const std::string& failure_reason = "");

  // 分配归属：只允许从 PENDING 出发。成功时状态转 ASSIGNED、revision +1。
  bool AssignTo(const TaskId& id, const RobotId& robot, TimeNs now);

  // 失败后重排：还有重试配额则清空归属、retry_count+1、回到 PENDING，
  // 返回 true；配额用尽则转 FAILED 并记录原因，返回 false。
  bool RetryOrFail(const TaskId& id, TimeNs now, const std::string& reason);

  // 取消：任何非终态都可取消，原因写入 failure_reason 便于审计
  bool Cancel(const TaskId& id, TimeNs now, const std::string& reason);

  const Task* Find(const TaskId& id) const;
  bool Contains(const TaskId& id) const { return Find(id) != nullptr; }

  std::vector<Task> All() const;
  std::vector<Task> ByState(TaskState s) const;

  // 当前被占用的目标位置（含未完成任务的归属目标），供分配策略做互斥判断
  std::vector<Pose2D> ActiveTargets() const;

  std::size_t Size() const { return tasks_.size(); }

  // 运行时参数化（manager 声明完参数后覆盖默认配置）
  void set_config(const TaskQueueConfig& cfg) { cfg_ = cfg; }
  const TaskQueueConfig& config() const { return cfg_; }

  // 清理超过保留策略的终态任务，返回被删除的 id。
  // 活跃任务永不清理；终态任务保留最近 max_terminal 个、且不超过 retention_age。
  std::vector<TaskId> Prune(TimeNs now);

private:
  TaskQueueConfig cfg_;
  // 用 map 而不是 unordered_map：All() / ByState() 的输出顺序稳定，
  // 回放比对和测试断言都不依赖插入顺序
  std::map<TaskId, Task> tasks_;
};

}  // namespace fleet
