#!/usr/bin/env bash
# 录一段标准演示包，供监控台回放（"回放固化"的采集侧）。
#
# 为什么只录这几个话题：回放要看的是任务全生命周期与车辆在线状态，
# 相机图像体积大且 bag 回放不渲染，所以不录。
#
# 用法:
#   ./scripts/record_demo_bag.sh                 # 90s，存到 ~/fleet_bags/demo_<时间>
#   ./scripts/record_demo_bag.sh /tmp/demo 120  # 指定目录与时长
set -uo pipefail

OUT="${1:-$HOME/fleet_bags/demo_$(date +%Y%m%d_%H%M%S)}"
DURATION="${2:-90}"

TOPICS=(
  /fleet/tasks
  /robot1/robot_status
  /robot2/robot_status
  /robot1/task_status
  /robot2/task_status
  /robot1/detections
  /robot2/detections
)

if [[ -e "$OUT" ]]; then
  echo "[EE] 目录已存在: $OUT" >&2
  exit 2
fi

mkdir -p "$(dirname "$OUT")"
echo "[..] 录制 ${DURATION}s 到 $OUT"
echo "     话题: ${TOPICS[*]}"

# timeout 到时返回 124，属于正常结束
timeout "$DURATION" ros2 bag record -o "$OUT" --topics "${TOPICS[@]}" || true

if [[ -f "$OUT/metadata.yaml" ]]; then
  echo "[OK] 录制完成"
  echo "     回放: ros2 run fleet_gui fleet_monitor --replay $OUT"
else
  echo "[EE] 没生成 metadata.yaml，录制可能失败" >&2
  exit 1
fi