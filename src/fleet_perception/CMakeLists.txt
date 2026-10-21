cmake_minimum_required(VERSION 3.16)
project(fleet_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# 感知核心库：依赖 OpenCV（图像分割本身离不开），但**不依赖 ROS**。
# 这样测距/去抖/颜色分割都能在 gtest 里毫秒级验证，不需要起仿真也不需要 rosbag。
add_library(perception_core
  src/ranging.cpp
  src/debouncer.cpp
  src/color_detector.cpp
)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(perception_core PUBLIC ${OpenCV_LIBS})
target_compile_features(perception_core PUBLIC cxx_std_17)

# ROS 适配层：只做订阅、TF 查询、消息转换与发布
add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node
  perception_core
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${cv_bridge_TARGETS}
  ${fleet_msgs_TARGETS}
)
target_compile_features(perception_node PRIVATE cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ranging test/test_ranging.cpp)
  target_link_libraries(test_ranging perception_core)

  ament_add_gtest(test_debouncer test/test_debouncer.cpp)
  target_link_libraries(test_debouncer perception_core)

  ament_add_gtest(test_color_detector test/test_color_detector.cpp)
  target_link_libraries(test_color_detector perception_core)
endif()

install(TARGETS perception_core
  EXPORT perception_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(PROGRAMS scripts/verify_perception.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(perception_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
