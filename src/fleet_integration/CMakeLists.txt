cmake_minimum_required(VERSION 3.16)
project(fleet_integration)

find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_agent
)
install(PROGRAMS scripts/fake_nav.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_nav
)

if(BUILD_TESTING)
  # 真调度中心 + 假执行代理：不需要 Gazebo / Nav2，CI 可跑
  add_launch_test(test/test_manager_loop.py
    TIMEOUT 180
  )
  # 超时改派 + 迟到回调的时序测试（两个假 agent）
  add_launch_test(test/test_late_callbacks.py
    TIMEOUT 120
  )
  # 真 agent + 假 Nav2：导航/对位/搜索三阶段的取消语义
  add_launch_test(test/test_agent_cancel.py
    TIMEOUT 120
  )
endif()

ament_package()