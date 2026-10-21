#include "fleet_agent/agent_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace fleet {

ApproachResult ComputeApproach(const Pose2D& robot, const Pose2D& target,
                               double approach_distance)
{
  ApproachResult out;
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double dist = std::hypot(dx, dy);

  out.distance_to_target = dist;

  // 目标与机器人重合：没有方向可算，保持原地
  if (dist < 1e-6) {
    out.pose = robot;
    out.pose.yaw = robot.yaw;
    out.already_close = true;
    return out;
  }

  const double yaw_toward_target = std::atan2(dy, dx);
  const double ux = dx / dist;
  const double uy = dy / dist;

  if (dist <= approach_distance) {
    // 已经比预定终点更近：不往后退，原地转向目标
    out.pose = Pose2D{robot.x, robot.y, yaw_toward_target};
    out.already_close = true;
    return out;
  }

  out.pose.x = target.x - ux * approach_distance;
  out.pose.y = target.y - uy * approach_distance;
  out.pose.yaw = yaw_toward_target;
  return out;
}

}  // namespace fleet
