cmake_minimum_required(VERSION 3.16)
project(fleet_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(fleet_manager REQUIRED)

# 纯逻辑核心：接近点几何 + 视觉对位控制律。
# 不依赖 ROS —— 这两块最容易出现"差一点、转反了、超时没判"的边界错误，
# 用注入时间的单测能毫秒级覆盖，不用起仿真。
add_library(agent_core
  src/agent_geometry.cpp
  src/docking_controller.cpp
)
target_include_directories(agent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${fleet_manager_INCLUDE_DIRS}
)
target_compile_features(agent_core PUBLIC cxx_std_17)

# 只用到 fleet_core 头文件里的纯类型（Pose2D/TaskState）
target_link_libraries(agent_core PUBLIC ${fleet_manager_LIBRARIES})

add_executable(robot_agent src/robot_agent_node.cpp)
target_link_libraries(robot_agent
  agent_core
  rclcpp::rclcpp
  ${rclcpp_action_TARGETS}
  ${nav2_msgs_TARGETS}
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  ${fleet_msgs_TARGETS}
  ${fleet_manager_LIBRARIES}
)
target_compile_features(robot_agent PRIVATE cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_docking_controller test/test_docking_controller.cpp)
  target_link_libraries(test_docking_controller agent_core)
endif()

install(TARGETS agent_core
  EXPORT agent_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS robot_agent
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(PROGRAMS scripts/verify_agent.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(agent_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
