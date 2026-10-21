cmake_minimum_required(VERSION 3.16)
project(fleet_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

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

ament_export_include_directories(include)
ament_export_libraries(fleet_core)
ament_export_targets(fleet_core_targets)

ament_package()
