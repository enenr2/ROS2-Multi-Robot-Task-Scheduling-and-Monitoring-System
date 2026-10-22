#!/usr/bin/env bash
# 停止仿真与所有配套进程，并清理 Fast DDS 共享内存残留。
#
# 为什么需要专门的脚本：
#   1. Nav2 节点在生命周期过渡卡住时会忽略 SIGTERM，普通 kill 杀不掉，
#      残留进程与下一次启动的同名节点撞车 —— 表现为 lifecycle_manager
#      卡在 "Configuring controller_server" 或服务调用发不出去。
#   2. 反复启停会在 /dev/shm 留下 fastdds_* 段，有时会让某台车的
#      static TF 凭空消失（/tf_static 里看不到该车连杆）。
#
# 用法：./scripts/stop_sim.sh
set -uo pipefail

PATTERN='ros2 launch fleet_bringup|gz-sim-main|ros_gz_bridge|parameter_bridge|robot_state_publisher|static_transform_publisher|nav2_|robot_agent|perception_node|fleet_manager_node|rviz2|fleet_monitor'

collect_pids() {
  ps -eo pid,args | grep -E "$PATTERN" | grep -v grep | awk '{print $1}'
}

PIDS="$(collect_pids)"
if [[ -z "$PIDS" ]]; then
  echo "没有运行中的仿真进程"
else
  echo "停止进程: $(echo "$PIDS" | tr '\n' ' ')"
  # shellcheck disable=SC2086
  kill $PIDS 2>/dev/null
  for _ in $(seq 1 10); do
    sleep 0.5
    PIDS="$(collect_pids)"
    [[ -z "$PIDS" ]] && break
  done
  if [[ -n "$PIDS" ]]; then
    echo "以下进程未响应 SIGTERM，强制结束: $(echo "$PIDS" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill -9 $PIDS 2>/dev/null
    sleep 1
  fi
fi

LEFT="$(collect_pids)"
if [[ -n "$LEFT" ]]; then
  echo "[EE] 仍有进程存活: $(echo "$LEFT" | tr '\n' ' ')" >&2
  exit 1
fi

# Fast DDS 共享内存段：没有 ROS 进程时删除是安全的
rm -f /dev/shm/fastdds_* 2>/dev/null
echo "已清理 /dev/shm 下的 fastdds 段，可以重新启动"
