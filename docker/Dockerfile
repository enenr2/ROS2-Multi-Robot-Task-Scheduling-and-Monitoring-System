# 多机调度系统的运行镜像：ROS 2 + Gazebo + Nav2 + Qt 监控台
#
# 说明：
#   * 只装运行期依赖，源码在构建阶段编译进 /ws/install
#   * 入口脚本见 docker/entrypoint.sh，支持 sim / monitor / replay / shell
#   * 仿真需要软件渲染（LIBGL_ALWAYS_SOFTWARE=1），compose 文件已设置
FROM ros:lyrical-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-turtlebot3-gazebo \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-opencv \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rosbag2 \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-launch-testing \
      ros-${ROS_DISTRO}-launch-testing-ament-cmake \
      ros-${ROS_DISTRO}-launch-testing-ros \
      qt6-base-dev \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    (rosdep update --rosdistro "${ROS_DISTRO}" || true) && \
    rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
COPY scripts /ws/scripts
RUN chmod +x /entrypoint.sh /ws/scripts/*.sh

# 仿真数据走容器内的共享内存
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sim"]