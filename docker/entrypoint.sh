#!/usr/bin/env bash
# 容器入口：sim / monitor / replay / shell / 任意命令
#
#   docker run --rm --network host fleet-sim sim
#   docker run --rm --network host fleet-sim monitor        # 需要 X11
#   docker run --rm --network host -v /path/bag:/bag fleet-sim replay /bag
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

MODE="${1:-sim}"
shift || true

case "$MODE" in
  sim)
    exec ros2 launch fleet_bringup fleet_sim.launch.py \
      gui:=false rviz:=false monitor:=false "$@"
    ;;
  monitor)
    exec ros2 run fleet_gui fleet_monitor "$@"
    ;;
  replay)
    exec ros2 run fleet_gui fleet_monitor --replay "$@"
    ;;
  record)
    exec /ws/scripts/record_demo_bag.sh "$@"
    ;;
  shell)
    exec bash
    ;;
  *)
    exec "$MODE" "$@"
    ;;
esac