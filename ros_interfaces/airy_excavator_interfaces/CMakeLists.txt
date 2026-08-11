cmake_minimum_required(VERSION 3.8)
project(airy_excavator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(
  FILES airy_excavator_interfaces/snapshot_digest.py
  DESTINATION ${PYTHON_INSTALL_DIR}/airy_excavator_interfaces
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HomePoseCatalog.msg"
  "msg/RuntimeStatus.msg"
  "msg/TargetSnapshot.msg"
  "msg/TrajectorySnapshot.msg"
  "action/Follow.action"
  "action/ExecuteDig.action"
  "action/ExecuteDump.action"
  "action/ExcavationCycle.action"
  "action/Plan.action"
  "action/ReturnHome.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
