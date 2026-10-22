cmake_minimum_required(VERSION 3.16)
project(fleet_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 纯逻辑核心库：刻意不依赖 rclcpp / fleet_msgs / geometry_msgs。
# 目的：gtest 不必起 ROS 上下文、不必起仿真，毫秒级跑完；
# 时间由调用方传入而不是内部取时钟，测试可以自由注入时间点。
# ROS 适配层（订阅、服务、action）在 M6 单独加，只做类型转换。
add_library(fleet_core
  src/state_machine.cpp
  src/task_queue.cpp
  src/assignment.cpp
  src/watchdog.cpp
)
target_include_directories(fleet_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(fleet_core PUBLIC cxx_std_17)

# ROS 适配层：只做话题/服务/action 接线，决策仍全部走 fleet_core。
# 它单独成一个 target，确保 fleet_core 本身不引入任何 ROS 依赖（见 test_no_ros_dependency）。
add_executable(fleet_manager_node src/fleet_manager_node.cpp)
target_link_libraries(fleet_manager_node
  fleet_core
  rclcpp::rclcpp
  ${rclcpp_action_TARGETS}
  ${fleet_msgs_TARGETS}
  geometry_msgs::geometry_msgs
  visualization_msgs::visualization_msgs
)
target_compile_features(fleet_manager_node PRIVATE cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine fleet_core)

  ament_add_gtest(test_task_queue test/test_task_queue.cpp)
  target_link_libraries(test_task_queue fleet_core)

  ament_add_gtest(test_assignment test/test_assignment.cpp)
  target_link_libraries(test_assignment fleet_core)

  ament_add_gtest(test_watchdog test/test_watchdog.cpp)
  target_link_libraries(test_watchdog fleet_core)

  # 业务逻辑库不该引入 ROS 依赖，用编译期断言把这个约束固化下来
  ament_add_gtest(test_no_ros_dependency test/test_no_ros_dependency.cpp)
  target_link_libraries(test_no_ros_dependency fleet_core)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS fleet_core
  EXPORT fleet_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fleet_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/verify_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(fleet_core)
ament_export_targets(fleet_core_targets)

ament_package()
