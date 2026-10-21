#include "fleet_manager/state_machine.hpp"

namespace fleet {

const char* ToString(TaskState s)
{
  switch (s) {
    case TaskState::kPending:    return "PENDING";
    case TaskState::kAssigned:   return "ASSIGNED";
    case TaskState::kNavigating: return "NAVIGATING";
    case TaskState::kDocking:    return "DOCKING";
    case TaskState::kDone:       return "DONE";
    case TaskState::kFailed:     return "FAILED";
    case TaskState::kCanceled:   return "CANCELED";
  }
  return "UNKNOWN";
}

const char* ToString(RobotState s)
{
  switch (s) {
    case RobotState::kIdle:    return "IDLE";
    case RobotState::kBusy:    return "BUSY";
    case RobotState::kError:   return "ERROR";
    case RobotState::kOffline: return "OFFLINE";
  }
  return "UNKNOWN";
}

const char* ToString(Color c)
{
  switch (c) {
    case Color::kRed:   return "RED";
    case Color::kGreen: return "GREEN";
    case Color::kBlue:  return "BLUE";
    case Color::kUnknown: break;
  }
  return "UNKNOWN";
}

namespace {

// 迁移表：legal[from][to]。用二维数组而不是 switch 里的长 case 列表，
// 是为了让规则在源码里就是一张能被逐行核对的表。
constexpr int kStateCount = 7;

constexpr bool kLegal[kStateCount][kStateCount] = {
  /* from PENDING    */ {false, true,  false, false, false, true,  true},
  /* from ASSIGNED   */ {true,  false, true,  false, false, true,  true},
  /* from NAVIGATING */ {true,  false, false, true,  false, true,  true},
  /* from DOCKING    */ {true,  false, false, false, true,  true,  true},
  /* from DONE       */ {false, false, false, false, false, false, false},
  /* from FAILED     */ {true,  false, false, false, false, false, true},
  /* from CANCELED   */ {false, false, false, false, false, false, false},
};

constexpr std::size_t Index(TaskState s) { return static_cast<std::size_t>(s); }

}  // namespace

bool TaskStateMachine::IsLegal(TaskState from, TaskState to)
{
  if (Index(from) >= kStateCount || Index(to) >= kStateCount) {
    return false;
  }
  // 自迁移一律拒绝：状态变更必须是真实推进，重复上报同一状态由
  // 调用方按幂等处理，不该走迁移接口。
  if (from == to) {
    return false;
  }
  return kLegal[Index(from)][Index(to)];
}

bool TaskStateMachine::IsTerminal(TaskState s)
{
  return s == TaskState::kDone || s == TaskState::kCanceled;
}

bool TaskStateMachine::CanRetry(const Task& t)
{
  return t.retry_count < t.max_retries;
}

}  // namespace fleet
