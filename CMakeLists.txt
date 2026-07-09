cmake_minimum_required(VERSION 3.10)
project(arm_api2)

# Silence CMP0167 dev warnings from MoveIt config extras that still call
# find_package(Boost): use Boost's own CMake config instead of the removed
# FindBoost module.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(joy REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arm_api2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# include headers
include_directories(include)

# Build moveit2_iface
add_executable(
  moveit2_iface
  src/moveit2_iface.cpp
  src/moveit2_iface_node.cpp
  src/piper_joint_gripper.cpp
  src/robotiq_gripper.cpp
  src/utils.cpp)
target_include_directories(moveit2_iface
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  include/)

# Build moveit2_iface
target_compile_features(moveit2_iface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit2_iface
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  arm_api2_msgs
  moveit_servo
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

# Install moveit2_iface
install(TARGETS
  moveit2_iface
  DESTINATION lib/${PROJECT_NAME})

# Install launch directory
install(DIRECTORY
  launch
  config
  utils
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Tutorial assets (YAML + README); pick_place_sequence.py is installed as PROGRAMS below.
install(
  DIRECTORY tutorials/
  DESTINATION share/${PROJECT_NAME}/tutorials
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.md"
  PATTERN "*.gif"
)

# Install C++ files
add_executable(joy_ctl src/arm_joy_node.cpp src/arm_joy.cpp)

target_include_directories(joy_ctl
  PRIVATE
  include/
)

# python dependencies
ament_target_dependencies(joy_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs control_msgs)

install(TARGETS
joy_ctl
DESTINATION lib/${PROJECT_NAME})

add_executable(keyboard_ctl src/servo_keyboard_input.cpp)

ament_target_dependencies(keyboard_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs control_msgs rclcpp_action)

install(TARGETS
  keyboard_ctl
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tutorials/pick_place_sequence.py
  examples/scripts/joint_sender_action_client_async.py
  examples/scripts/joint_sender_action_client.py
  examples/scripts/pose_sender_action_client_async.py
  examples/scripts/pose_sender_action_client.py
  examples/scripts/trajectory_sender_action_client_async.py
  examples/scripts/trajectory_sender_action_client.py
  examples/scripts/servo_twist_sender.py
  examples/scripts/servo_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck()
  ament_copyright()
  ament_cpplint(
    FILTERS
      -build/header_guard
      -build/include_order
      -build/include_what_you_use
      -build/namespaces
      -readability/braces
      -readability/casting
      -readability/todo
      -runtime/explicit
      -runtime/int
      -runtime/string
      -whitespace/blank_line
      -whitespace/comments
      -whitespace/line_length
  )
  ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)
  ament_lint_cmake()
  ament_pep257(
    --add-ignore
      D200
      D202
      D205
      D209
      D213
      D400
      D401
      D406
      D407
      D413
      D415
  )
  ament_uncrustify()
  ament_xmllint()
endif()

ament_package()
