cmake_minimum_required(VERSION 3.16)
project(fleet_integration)

find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_agent
)

if(BUILD_TESTING)
  # 真调度中心 + 假执行代理：不需要 Gazebo / Nav2，CI 可跑
  add_launch_test(test/test_manager_loop.py
    TIMEOUT 180
  )
endif()

ament_package()