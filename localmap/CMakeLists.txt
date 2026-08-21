cmake_minimum_required(VERSION 3.8)
project(airy_localmap)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(
  DIRECTORY localmap_core
  DESTINATION ${PYTHON_INSTALL_DIR}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS localmap_core/runtime_ros/fixture_plan_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fixture_plan_server
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_fixture_planning tests/test_fixture_planning.py
  )
  ament_add_pytest_test(
    test_fixture_ros_entrypoint tests/test_fixture_ros_entrypoint.py
  )
  ament_add_pytest_test(
    test_fixture_plan_action_integration
    tests/test_fixture_plan_action_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_replay_clock
    tests/test_replay_clock.py
    TIMEOUT 15
  )
  ament_add_pytest_test(
    test_mission_target_registry
    tests/test_mission_target_registry.py
  )
endif()

ament_package()
