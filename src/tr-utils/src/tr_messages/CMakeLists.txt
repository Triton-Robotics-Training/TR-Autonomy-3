cmake_minimum_required(VERSION 3.14)
project(tr_messages)

# Suppress CMP0148 warnings for ROS 2 Humble
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif() 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetWithImg.msg"
  "msg/FromSTM32.msg"
  "msg/ToSTM32.msg"
  "msg/Panel.msg"
  "msg/Panels.msg"
  "msg/WriteChassis.msg"
  "msg/WriteGimbal.msg"
  "msg/RefData.msg"
  "msg/FiringSolution.msg"
  "msg/BallisticsSolutionEstimations.msg"
  "msg/BallisticsSolutionPanels.msg"
  "msg/NavState.msg"
  "msg/EmbeddedUserInput.msg"
  "msg/EmbeddedRobotState.msg"
  "msg/RobotEstimations.msg"

  "msg/StateMachine.msg"
  
  "msg/Geometry.msg"

  "srv/ListenSerial.srv"
  "srv/WriteSerial.srv"
  "srv/WriteOdometry.srv"
  "srv/LidarOdometry.srv"
  "srv/QueryOdometry.srv"
  "srv/Estimations.srv"

  "msg/simulation/SimTeleopInput.msg"
  "msg/simulation/SimGroundTruth.msg"
  "msg/simulation/RobotGroundTruth.msg"
  DEPENDENCIES vision_msgs sensor_msgs std_msgs builtin_interfaces geometry_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
