cmake_minimum_required(VERSION 3.8)
project(sih_amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetHeader.msg"
  "msg/GridCell.msg"
  "msg/RobotState.msg"
  "msg/PeerTrack.msg"
  "msg/PeerTrackArray.msg"
  "msg/TrajectoryIntent.msg"
  "msg/RoutePlan.msg"
  "msg/Task.msg"
  "msg/TaskAnnouncement.msg"
  "msg/TaskConsensus.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskExecutionStatus.msg"
  "msg/CorridorProtocol.msg"
  "msg/BlockageObservation.msg"
  "msg/FleetHealth.msg"
  "msg/SafetyState.msg"
  "msg/FleetEvent.msg"
  "msg/DockProtocol.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
