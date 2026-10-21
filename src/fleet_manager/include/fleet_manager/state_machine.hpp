// 任务状态机：只回答"这条迁移合不合法"，不持有任何数据。
//
// 把迁移规则从任务表里拆出来的理由：规则是纯函数，可以 7x7 穷举对拍；
// 混在会改数据的类里就只能沿着业务流程零散验证，必然漏路径。
//
// 迁移表（设计文档 §6.2）：
//   PENDING    -> ASSIGNED | FAILED | CANCELED
//   ASSIGNED   -> NAVIGATING | PENDING | FAILED | CANCELED
//   NAVIGATING -> DOCKING | PENDING | FAILED | CANCELED
//   DOCKING    -> DONE | PENDING | FAILED | CANCELED
//   FAILED     -> PENDING | CANCELED
//   DONE       -> 终态
//   CANCELED   -> 终态
//
// 注意 FAILED -> PENDING 在结构上是合法的，但能否真的重排还要看
// 重试次数是否用完，这个判断由 CanRetry 单独负责。两件事分开，
// 是为了让"结构合法性"和"配额限制"能分别被测试。
#pragma once

#include "fleet_manager/task_types.hpp"

namespace fleet {

class TaskStateMachine
{
public:
  static bool IsLegal(TaskState from, TaskState to);
  static bool IsTerminal(TaskState s);

  // 重试配额是否还有余量
  static bool CanRetry(const Task& t);
};

}  // namespace fleet
