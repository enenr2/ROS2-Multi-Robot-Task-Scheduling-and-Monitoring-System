#!/usr/bin/env bash
# 安装本项目的运行依赖：Nav2 导航栈 + Docker
#
# 用法:
#   sudo bash install_deps.sh              # 自动探测已安装的 ROS 发行版
#   sudo bash install_deps.sh jazzy        # 指定 ROS 发行版
#
# 特性：幂等，可重复执行；每步都有验证输出；非交互（不会卡在配置界面）
set -uo pipefail

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    [OK] %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m    [!!] %s\033[0m\n' "$*"; }
err()  { printf '\033[1;31m    [EE] %s\033[0m\n' "$*"; }

TARGET_USER="${SUDO_USER:-$(id -un)}"
FAIL=0

if [[ $EUID -ne 0 ]]; then
  err "需要 root 权限，请用: sudo bash $0"
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------- 探测 ROS

log "步骤 1/5：探测 ROS 发行版"

ROS_DISTRO_NAME="${1:-}"
if [[ -z "$ROS_DISTRO_NAME" ]]; then
  # 优先取环境变量，其次取 /opt/ros 下唯一的目录
  if [[ -n "${ROS_DISTRO:-}" && -d "/opt/ros/${ROS_DISTRO}" ]]; then
    ROS_DISTRO_NAME="$ROS_DISTRO"
  else
    mapfile -t found < <(ls -1 /opt/ros 2>/dev/null)
    if [[ ${#found[@]} -eq 1 ]]; then
      ROS_DISTRO_NAME="${found[0]}"
    elif [[ ${#found[@]} -eq 0 ]]; then
      err "未找到 /opt/ros 下的任何 ROS 安装"
      echo
      echo "请先安装 ROS2，再运行本脚本。参考（以 jazzy 为例）:"
      echo "  sudo apt install software-properties-common"
      echo "  sudo add-apt-repository universe"
      echo "  sudo apt install ros-jazzy-ros-base"
      echo "或指定发行版重跑: sudo bash $0 <distro>"
      exit 1
    else
      err "检测到多个 ROS 发行版: ${found[*]}"
      echo "请显式指定其一，例如: sudo bash $0 ${found[0]}"
      exit 1
    fi
  fi
fi

if [[ ! -d "/opt/ros/${ROS_DISTRO_NAME}" ]]; then
  err "/opt/ros/${ROS_DISTRO_NAME} 不存在"
  exit 1
fi
ok "ROS 发行版: ${ROS_DISTRO_NAME}"

# ---------------------------------------------------------------- apt 索引

log "步骤 2/5：刷新软件包索引"
if apt-get update -qq; then
  ok "apt 索引已更新"
else
  warn "apt-get update 有报错（镜像源问题？），继续尝试安装"
fi

# ---------------------------------------------------------------- Nav2

log "步骤 3/5：安装 Nav2 与视觉依赖"

NAV2_PKGS=(
  "ros-${ROS_DISTRO_NAME}-navigation2"     # 元包，拉入 planner/controller/bt_navigator 等
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"    # 官方启动脚本与参数模板
  "ros-${ROS_DISTRO_NAME}-nav2-msgs"
)
VIS_PKGS=(
  "ros-${ROS_DISTRO_NAME}-cv-bridge"       # OpenCV 与 ROS Image 互转
  "ros-${ROS_DISTRO_NAME}-vision-opencv"
  "ros-${ROS_DISTRO_NAME}-image-transport"
)
# M8 监控端：Qt6 控件 + rosbag2 回放
GUI_PKGS=(
  "qt6-base-dev"
  "ros-${ROS_DISTRO_NAME}-rosbag2"
  "ros-${ROS_DISTRO_NAME}-rosbag2-storage-mcap"
)

for pkg in "${NAV2_PKGS[@]}" "${VIS_PKGS[@]}" "${GUI_PKGS[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    ok "$pkg 已安装，跳过"
  elif apt-get install -y "$pkg" >/dev/null 2>&1; then
    ok "$pkg 安装完成"
  else
    err "$pkg 安装失败"
    FAIL=1
  fi
done

# ---------------------------------------------------------------- Docker

log "步骤 4/5：安装 Docker"

if command -v docker >/dev/null 2>&1; then
  ok "docker 已安装：$(docker --version)"
else
  DOCKER_PKGS=("docker.io")
  # compose v2 在较新 Ubuntu 上是独立包，不存在就跳过，不影响主功能
  apt-cache show docker-compose-v2 >/dev/null 2>&1 && DOCKER_PKGS+=("docker-compose-v2")
  if apt-get install -y "${DOCKER_PKGS[@]}" >/dev/null 2>&1; then
    ok "docker 安装完成"
  else
    err "docker 安装失败"
    FAIL=1
  fi
fi

if systemctl enable --now docker >/dev/null 2>&1; then
  ok "docker 服务已启用并启动"
else
  warn "docker 服务启用失败，请检查是否在容器/无 systemd 环境中运行"
  FAIL=1
fi

if id -nG "$TARGET_USER" 2>/dev/null | tr ' ' '\n' | grep -qx docker; then
  ok "用户 $TARGET_USER 已在 docker 组"
else
  if usermod -aG docker "$TARGET_USER" 2>/dev/null; then
    warn "已将 $TARGET_USER 加入 docker 组 —— 需重新登录或执行 newgrp docker 才生效"
  else
    err "加入 docker 组失败"
    FAIL=1
  fi
fi

# ---------------------------------------------------------------- 验证

log "步骤 5/5：验证"

echo "--- Nav2 关键组件 ---"
for p in nav2_planner nav2_controller nav2_bt_navigator nav2_behaviors \
         nav2_map_server nav2_lifecycle_manager nav2_bringup; do
  if [[ -d "/opt/ros/${ROS_DISTRO_NAME}/share/$p" ]]; then
    ok "$p"
  else
    err "$p 缺失"
    FAIL=1
  fi
done

echo "--- 视觉组件 ---"
for p in cv_bridge image_transport; do
  if [[ -d "/opt/ros/${ROS_DISTRO_NAME}/share/$p" ]]; then
    ok "$p"
  else
    err "$p 缺失"
    FAIL=1
  fi
done

echo "--- Docker ---"
if command -v docker >/dev/null 2>&1 && docker --version >/dev/null 2>&1; then
  ok "$(docker --version)"
else
  err "docker CLI 不可用"
  FAIL=1
fi
systemctl is-active --quiet docker && ok "docker 服务运行中" || { err "docker 服务未运行"; FAIL=1; }

echo "--- 环境余量 ---"
df -h / | tail -1
free -g | sed -n '2p'

echo
if [[ $FAIL -eq 0 ]]; then
  printf '\033[1;32m全部依赖就绪。\033[0m\n'
  echo "下一步：让 docker 组权限生效"
  echo "  newgrp docker        # 当前会话立即生效，或直接注销重登"
  echo "验证："
  echo "  docker ps"
  echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && ros2 pkg list | grep -c nav2_"
else
  printf '\033[1;31m有项目未通过验证，见上方 [EE]。\033[0m\n'
  exit 1
fi
