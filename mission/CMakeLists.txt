cmake_minimum_required(VERSION 3.8)
project(airy_mission_runtime)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(
  FILES
    __init__.py
    contract.py
    dig_point_catalog.py
    demo.py
    demo_runner.py
    follow.py
    home.py
    markers.py
    replay.py
    shadow_observation.py
    state_machine.py
    trajectory_tracker.py
  DESTINATION ${PYTHON_INSTALL_DIR}/mission
)

install(
  DIRECTORY runtime_ros
  DESTINATION ${PYTHON_INSTALL_DIR}/mission
  PATTERN "__pycache__" EXCLUDE
)

# The live ExcavationCycle ROS executable is installed independently of the
# source tree, so install the PC-Orin RPC client modules that it imports.
install(
  FILES
    ../runtime_bridge/__init__.py
    ../runtime_bridge/orin_behavior_rpc.py
    ../runtime_bridge/orin_follow_client.py
    ../runtime_bridge/orin_cycle_client.py
  DESTINATION ${PYTHON_INSTALL_DIR}/runtime_bridge
)

install(
  FILES
    apps/__init__.py
    apps/publish_mission_markers.py
  DESTINATION ${PYTHON_INSTALL_DIR}/mission/apps
)

install(
  PROGRAMS runtime_ros/follow_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME machine_behavior_shadow_server
)

install(
  PROGRAMS runtime_ros/send_hold_follow_goal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME send_hold_follow_goal
)

install(
  PROGRAMS runtime_ros/send_plan_goal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME send_plan_goal
)

install(
  PROGRAMS runtime_ros/run_plan_follow_shadow.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_plan_follow_shadow
)

install(
  PROGRAMS runtime_ros/run_plan_follow_live.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_plan_follow_live
)

install(
  PROGRAMS runtime_ros/run_prepared_plan_follow_live.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_prepared_plan_follow_live
)

install(
  PROGRAMS runtime_ros/send_return_home_goal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME send_return_home_goal
)

install(
  PROGRAMS runtime_ros/excavation_cycle_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME excavation_cycle_server
)

install(
  PROGRAMS runtime_ros/run_excavation_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_excavation_demo
)

install(
  PROGRAMS apps/publish_mission_markers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_snapshot_publisher
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config schemas
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dig_point_catalog tests/test_dig_point_catalog.py)
  ament_add_pytest_test(test_markers tests/test_markers.py)
  ament_add_pytest_test(test_follow_session tests/test_follow_session.py)
  ament_add_pytest_test(
    test_follow_action_integration
    tests/test_follow_action_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_named_joint_poses tests/test_named_joint_poses.py)
  ament_add_pytest_test(
    test_mission_snapshot_publisher
    tests/test_mission_snapshot_publisher.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_plan_follow_action_integration
    tests/test_plan_follow_action_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_ros_entrypoints tests/test_ros_entrypoints.py)
  ament_add_pytest_test(
    test_run_plan_follow_live
    tests/test_run_plan_follow_live.py
  )
  ament_add_pytest_test(
    test_run_prepared_plan_follow_live
    tests/test_run_prepared_plan_follow_live.py
  )
  ament_add_pytest_test(
    test_return_home_action_integration
    tests/test_return_home_action_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_excavation_cycle_action_integration
    tests/test_excavation_cycle_action_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_demo_program tests/test_demo_program.py)
  ament_add_pytest_test(test_demo_runner tests/test_demo_runner.py)
  ament_add_pytest_test(
    test_excavation_demo_ros_goal
    tests/test_excavation_demo_ros_goal.py
  )
  ament_add_pytest_test(
    test_excavation_demo_action_integration
    tests/test_excavation_demo_action_integration.py
    TIMEOUT 30
  )
endif()

ament_package()
