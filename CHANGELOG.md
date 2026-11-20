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

## Nov 2026
- execution through move_group's `ExecuteTrajectory` action (client on the internal node), not `MoveGroupInterface::execute()`; one execution at a time, with a watchdog for a missing goal response / result
- `arm/stop` (`std_srvs/Trigger`, YAML `srv.stop.name`): cancels arm_api2's execution and sends MoveIt stop (also stops trajectories other clients sent to move_group); answers during planning and blocking executions
- action cancel cancels the running execution (goal ends CANCELED); queued goals are canceled before planning; new goals are rejected while a trajectory executes
- `change_state` while executing: `robot.on_state_change_while_moving: stop` (default) cancels the execution, `reject` refuses the change
- `robot.async_actions` (default `false`): action goals execute without blocking `run()`
- action feedback: `progress`, `elapsed_sec`, `remaining_sec`, `planned_duration_sec` (time based) and status `planning | executing | stopping`
- `plan_status` also for the action paths (reason `<action>: ...`), same codes as the topic paths; final status after execution with the measured final joint error and max path deviation
- execution checks on the robot's own `joint_states`: `robot.goal_joint_tolerance_rad` (+ `goal_settle_sec`, default 0.5 s) -> `GOAL_NOT_REACHED`; `robot.path_deviation_max_rad` (distance to the planned joint path) stops the motion -> `PATH_DEVIATION`; both 0 = off, values are always measured and reported
- config validated at start: all missing / mistyped / out-of-range keys reported at once, node exits with code 1 instead of aborting; unknown `robot.*` keys warned (typos); launch: clear error for an unknown robot/profile, explicit warning on the `_sim` fallback, new `config_path:=` and `gripper:=` args
- `gripper_action_type: none` / launch `gripper:=none`: no gripper backend, open/close fail at once with a message, `gripper_state` publishes `none` (GUI hides Open/Close); PiPER welding gun stacks pass `gripper:=none`
- YAML `planning:` section: `eef_step`, `pose_planners`, `topic_pose_planners` (`pipeline`, `planner`, `time`, `attempts`); defaults are the previous hard-coded LIN->EST->PRM x3 and LIN 0.25 s -> EST 0.75 s. MoveIt's `jump_threshold` is deprecated in Jazzy; joint jumps are rejected by `robot.max_joint_step_rad`
- namespaced robots: monitored planning scene `<move_group_ns>/monitored_planning_scene`, services `<ns>/arm_api2/get_planning_scene`, `<ns>/moveit_servo/publish_planning_scene` (root namespace unchanged); `srv.add_collision_object.name` configurable
- removed unused config keys: `planning_scene`, `with_planner`, `num_cart_pts`, `cmd_tool_orientation`, `cmd_delta_pose`, `display_trajectory`, `disable_collision`, `add_collision`

- `/diagnostics` (diagnostic_updater): move_group, planning scene monitor, joint_states age, execution (current / last result, counters), gripper, controllers (`list_controllers`), arm state and run loop busy time
- permanent motion log: one JSON line per motion command (planner, planning time, fraction, planned vs measured duration, final joint error, max path deviation, start/goal joints, result) in `motion_log_dir` (launch arg / param; default `~/.ros/arm_api2/motion_log`, `none` disables), daily files, never pruned
- `moveit2_iface.cpp` split by concern (core/state machine, config, motion, execution, services, servo, observability); pure logic in library `arm_api2_logic` (`motion_logic`, `motion_log`) with gtest unit tests (`test_motion_logic`, `test_motion_log`, `test_utils`); all package linters pass
- `utils::normalizeOrientation` keeps the header (`plan_status` of topic pose → joint commands had an empty frame)
- `joy_ctl` / `keyboard_ctl` publish to arm_api2's servo input `<ns>/moveit2_iface/servo_twist_cmd` (they sent to `/moveit2_iface_node/delta_twist_cmds`, which nothing read); topics, frames, axes and deadman are parameters; `joy_ctl` no longer crashes on short Joy messages and is also launched for a single robot with `launch_joy:=true`; keyboard gripper goes to `arm/gripper_control`; keyboard joint jog removed (no JointJog input in arm_api2)
- removed `servo_watchdog.py` and the `launch_servo_watchdog` launch argument: every twist is one servo step and zero twists are ignored, so the arm already stops when commands stop
- startup: a constructor failure after the heartbeat/executor threads started no longer ends in `std::terminate`

- `robot.async_actions` now defaults to `true` (action goals no longer block `run()`; cancel stops the robot, `ctl_state`/`current_pose` keep publishing); set `false` for the old blocking behaviour

## Nov 2026 (2) — tasks without MTC

- tasks: `arm/plan` (plan only, `plan_id` + segments for preview), `arm/execute_plan`, `arm/execute_task`; the whole task is planned in a copy of move_group's scene (each step from the end of the previous, attach/detach in the copy, carried object collision checked), the robot moves only when every step planned, failures name the step; per-segment execution through the existing execution path (stop, cancel, goal checks, motion log); start-state check before every segment (`plans.start_tolerance_rad`)
- `arm/move_relative` (TCP or base frame, linear or planned), `arm/move_to_named` + `arm/save_named_pose` / `list_named_poses` / `delete_named_pose` (saved file > YAML `named_poses:` > SRDF)
- `arm/attach_object` / `arm/detach_object` with touch links; `arm/set_path_constraints` (joint, orientation, position) / `arm/clear_path_constraints`, Pilz skipped while constraints are set
- scene under `arm/`: `add_collision_object` (now synchronous, also on the old root name), `add_collision_mesh`, `remove_collision_object`, `list_collision_objects`, `clear_collision_objects`
- Python client `arm_api2_py` (blocking/async, task builder); unit tests `test_task_logic`, `tests/test_arm_api2_py.py`

# Future potential features

- realtime pose follower (teleoperation: servo via absolute position)
- dynamic scene object update
