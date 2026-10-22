// cmd_vel 安全链路：把"发速度/停车"从 ROS 节点里拆出来，便于单测。
//
// 真机上"停车"是安全语义，不能依赖"碰巧发了两次零速"：
//   * 发布端用可靠 QoS + KeepLast(1)，停车一次连发 frames 帧覆盖单次丢包
//   * 非有限值（NaN/Inf）一律按停车处理，绝不把坏数据发给底盘
//   * 节点析构/取消/失败收尾都走同一个 Stop()
#pragma once

#include <cmath>
#include <functional>
#include <utility>

namespace fleet {

struct StopPolicy {
  int frames{3};   // 连续发送的零速帧数
};

class CmdVelLink
{
public:
  using PublishFn = std::function<void(double linear, double angular)>;

  explicit CmdVelLink(PublishFn publish, StopPolicy policy = StopPolicy{})
  : publish_(std::move(publish)), policy_(policy)
  {
    if (policy_.frames < 1) {
      policy_.frames = 1;
    }
  }

  // 返回 false 表示输入非法、已按停车处理
  bool Drive(double linear, double angular)
  {
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      Stop();
      return false;
    }
    publish_(linear, angular);
    return true;
  }

  void Stop()
  {
    for (int i = 0; i < policy_.frames; ++i) {
      publish_(0.0, 0.0);
    }
  }

  const StopPolicy& policy() const { return policy_; }

private:
  PublishFn publish_;
  StopPolicy policy_;
};

}  // namespace fleet