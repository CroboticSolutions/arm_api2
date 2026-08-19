# Changes added by KIT (Edgar Welte)

## Sep 2024

- add frame_id to published current pose
- add service to set max_vel and max_acc scaling factor
- add action servers instead of subscribers to receive commands (pose, joint_state, or pose path)
- add example script to send pose to action server
- add LIN, EST and PRM Planner for pose action

## Oct 2024

- add keyboard control node to control the robot via keyboard in servo mode

## Nov 2024
- integrate xbox joy ctl


## Jan 2025

- refactor code for planning
- add gripper functionality incl. action client for robotiq_2f
- add service for setting end effector link in moveit2 interface
- add servo watchdog node to monitor twist/jog commands and publish zero velocity if inactive
- add cumotion as planner

## Feb 2025
- set proper planner priority: 1. cumotion, if failed then PRM, EST, LIN (each 3 times)
- add planonly mode

## Mar - Aug 2025
- create `moveit2_simple_iface`, a topic/service-only interface (later merged back into `moveit2_iface`)
- add init config for ABB and Piper (sim)
- add service to add collision objects to the planning scene
- add service to change motion planner (`arm/set_planner`)
- standardize yaml configuration files across all supported robots

## Sep - Dec 2025
- port arm_api2 to ROS 2 Jazzy
- add SO-ARM100 support
- implement MoveIt Servo control for the Jazzy API
- add multi-robot launch support (spawn multiple UR robots via namespaces)
- add joystick-driven fingertip trajectory following in servo mode
- fix IK solver / kinematics-loading bug for UR robot

## 2026
- add Piper joint gripper support; extend Piper gripper control
- add Robotiq gripper support (parallel gripper action + legacy backend)
- add Cartesian path validation and reachability-check services (`arm/check_cartesian_path`, `arm/check_reachability`)
- add CRX-10iA sim, real and servo-real config profiles
- refactor: consolidate `moveit2_iface` and `moveit2_simple_iface`, removing the duplicated legacy interface and launch files
- fix CMP0167 Boost `find_package` warnings in CMake
- update `package.xml` dependencies (add `joy`, `moveit_msgs`; align `moveit_ros` -> `moveit_ros_planning`)

# Future potential features

- realtime pose follower (teleoperation: servo via absolute position)
- constraints for planners (e.g. orientation constraint for cup transport)
- dynamic scene object update
