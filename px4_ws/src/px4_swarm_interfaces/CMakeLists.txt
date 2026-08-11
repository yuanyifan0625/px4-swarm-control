cmake_minimum_required(VERSION 3.8)
project(px4_swarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ChangeFormation.action"
  "action/LandSwarm.action"
  "action/MoveLeader.action"
  "action/PauseSwarm.action"
  "action/TakeoffSwarm.action"
  "msg/FailsafeCommand.msg"
  "msg/FormationMode.msg"
  "msg/LeaderGoal.msg"
  "msg/MissionCommand.msg"
  "msg/VehicleSetpoint.msg"
  "msg/VehicleStatus.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
