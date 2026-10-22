cmake_minimum_required(VERSION 3.16)
project(fleet_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# 纯逻辑核心：任务快照去重、心跳判活、日志/告警生成。
# 不依赖 ROS 与 Qt，单测可以直接跑，不用起仿真或开窗口。
add_library(monitor_core src/monitor_core.cpp)
target_include_directories(monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(monitor_core PUBLIC cxx_std_17)

# 回放逻辑单独成库：GUI 和回放单测共用同一份实现
add_library(replay_core src/bag_replay.cpp)
target_include_directories(replay_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(replay_core PUBLIC
  monitor_core
  rclcpp::rclcpp
  ${fleet_msgs_TARGETS}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
)
target_compile_features(replay_core PUBLIC cxx_std_17)

add_executable(fleet_monitor
  src/main.cpp
  src/main_window.cpp
  src/map_view.cpp
  src/camera_view.cpp
  src/ros_bridge.cpp
  # 带 Q_OBJECT 的头文件必须进源文件列表，AUTOMOC 才会生成 moc
  include/fleet_gui/main_window.hpp
  include/fleet_gui/map_view.hpp
  include/fleet_gui/camera_view.hpp
  include/fleet_gui/ros_bridge.hpp
)
target_link_libraries(fleet_monitor
  monitor_core
  replay_core
  rclcpp::rclcpp
  ${fleet_msgs_TARGETS}
  nav_msgs::nav_msgs
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  Qt6::Widgets
)
target_compile_features(fleet_monitor PRIVATE cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_monitor_core test/test_monitor_core.cpp)
  target_link_libraries(test_monitor_core monitor_core)

  # 真写一个 sqlite3 bag 再读回来，覆盖"回放"这条链路
  ament_add_gtest(test_bag_replay test/test_bag_replay.cpp)
  target_link_libraries(test_bag_replay replay_core)
endif()

install(TARGETS monitor_core replay_core
  EXPORT monitor_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fleet_monitor
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(monitor_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp fleet_msgs nav_msgs sensor_msgs rosbag2_cpp rosbag2_storage)

ament_package()