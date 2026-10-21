#!/usr/bin/env bash
# 安装项目依赖：Nav2、视觉桥接、Docker
# 用法：sudo bash scripts/install_deps.sh
# 特性：幂等（已安装的会跳过），每步都有验证输出
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-lyrical}"
TARGET_USER="${SUDO_USER:-$(id -un)}"

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    [OK] %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m    [!!] %s\033[0m\n' "$*"; }

if [[ $EUID -ne 0 ]]; then
  echo "需要 root 权限，请用：sudo bash scripts/install_deps.sh" >&2
  exit 1
fi

# 避免安装过程中弹出交互式配置界面卡住
export DEBIAN_FRONTEND=noninteractive

log "步骤 0/4：刷新软件包索引"
apt-get update
ok "apt 索引已更新"

log "步骤 1/4：安装 Nav2（导航栈）"
# navigation2 是元包，会拉入 planner/controller/bt_navigator/behaviors 等全部组件
# nav2-bringup 提供官方启动脚本与参数模板
NAV2_PKGS=(
  "ros-${ROS_DISTRO_NAME}-navigation2"
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"
  "ros-${ROS_DISTRO_NAME}-nav2-msgs"
)
apt-get install -y "${NAV2_PKGS[@]}"
ok "Nav2 安装完成"

log "步骤 2/4：安装视觉依赖（cv_bridge / OpenCV）"
VIS_PKGS=(
  "ros-${ROS_DISTRO_NAME}-cv-bridge"
  "ros-${ROS_DISTRO_NAME}-vision-opencv"
  "ros-${ROS_DISTRO_NAME}-image-transport"
)
apt-get install -y "${VIS_PKGS[@]}"
ok "视觉依赖安装完成"

log "步骤 3/4：安装 Docker"
DOCKER_PKGS=("docker.io")
# compose v2 在较新 Ubuntu 上是独立包名，不存在就跳过，不影响主功能
if apt-cache show docker-compose-v2 >/dev/null 2>&1; then
  DOCKER_PKGS+=("docker-compose-v2")
else
  warn "docker-compose-v2 不在源内，跳过"
fi
apt-get install -y "${DOCKER_PKGS[@]}"

systemctl enable --now docker
ok "Docker 服务已启用并启动"

if id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx docker; then
  ok "用户 $TARGET_USER 已在 docker 组"
else
  usermod -aG docker "$TARGET_USER"
  warn "已将 $TARGET_USER 加入 docker 组 —— 需重新登录（或 newgrp docker）才生效"
fi

log "步骤 4/4：验证"
FAIL=0

echo "--- Nav2 包 ---"
for p in nav2_planner nav2_controller nav2_bt_navigator nav2_bringup nav2_map_server; do
  if [[ -d "/opt/ros/${ROS_DISTRO_NAME}/share/$p" ]]; then
    ok "$p"
  else
    warn "$p 未找到"
    FAIL=1
  fi
done

echo "--- 视觉包 ---"
for p in cv_bridge image_transport; do
  if [[ -d "/opt/ros/${ROS_DISTRO_NAME}/share/$p" ]]; then
    ok "$p"
  else
    warn "$p 未找到"
    FAIL=1
  fi
done

echo "--- Docker ---"
if docker --version; then
  ok "docker CLI 可用"
else
  warn "docker CLI 异常"
  FAIL=1
fi
if systemctl is-active --quiet docker; then
  ok "docker 服务运行中"
else
  warn "docker 服务未运行"
  FAIL=1
fi

echo "--- 资源余量 ---"
df -h / | tail -1
free -g | sed -n '2p'

if [[ $FAIL -eq 0 ]]; then
  printf '\n\033[1;32m全部依赖安装并验证通过。\033[0m\n'
  printf '请执行以下命令使 docker 组权限生效（或直接注销重登）：\n  newgrp docker\n'
else
  printf '\n\033[1;31m有项目未通过验证，请查看上方 [!!] 项。\033[0m\n'
  exit 1
fi
