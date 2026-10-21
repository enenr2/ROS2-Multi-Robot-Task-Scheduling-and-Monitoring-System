// 接近点几何：把"目标位姿"换算成"导航目标位姿"。
//
// 为什么需要：调度给的是物块位置，直接让 Nav2 开过去会撞上物块，而且车头
// 朝向不受控 —— 到了还得原地绕圈找目标。这里提前把导航终点退到
// standoff + 余量 的位置，并让车头正对目标，视觉精对位才有稳定的起点。
//
// 纯几何、无 ROS 依赖：可直接对坐标算例做单测。
#pragma once

#include "fleet_manager/task_types.hpp"

namespace fleet {

struct ApproachResult {
  Pose2D pose;             // 导航终点（朝向已对准目标）
  double distance_to_target{0.0};
  bool already_close{false};  // 当前距离已小于接近距离，可跳过导航直接对位
};

// approach_distance：导航终点与目标的距离，建议 standoff + 0.6~0.8
// 机器人已经在近处（<= approach_distance）时不后退，原地对准目标即可
ApproachResult ComputeApproach(const Pose2D& robot, const Pose2D& target,
                               double approach_distance);

}  // namespace fleet
