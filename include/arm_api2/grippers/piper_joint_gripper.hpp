/*******************************************************************************
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Gripper shim for AgileX Piper ROS2: either publishes full JointState to
 * `joint_ctrl_single` (piper_ctrl_single_node), or sends FollowJointTrajectory
 * goals like MoveIt/`piper_moveit_bridge` (joint7 meters).
 ******************************************************************************/

#ifndef ARM_API2_PIPER_JOINT_GRIPPER_HPP_
#define ARM_API2_PIPER_JOINT_GRIPPER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

enum class PiperGripperCommandMode
{
  /// `piper_ctrl_single_node` with `joint_ctrl_single` remapped to `cmd_topic` (JointState).
  JointState,
  /// Same as MoveIt/RViz: `FollowJointTrajectory` on e.g. `gripper_controller` (hardware: `piper_moveit_bridge`).
  FollowJointTrajectory,
};

struct PiperJointGripperConfig
{
  /// Topic exposing arm + gripper feedback (e.g. `/joint_states` from `piper_moveit_bridge`, or `joint_states_single`).
  std::string state_topic{"/joint_states_single"};
  /// Topic Piper subscribes to as `joint_ctrl_single` when remapped to `/joint_states` (JointState mode only).
  std::string cmd_topic{"/joint_states"};
  double open_stroke_m{0.0};
  double close_stroke_m{0.035};

  PiperGripperCommandMode command_mode{PiperGripperCommandMode::FollowJointTrajectory};
  /// Action name (absolute or relative to node namespace), e.g. `/gripper_controller/follow_joint_trajectory`.
  std::string trajectory_action{"/gripper_controller/follow_joint_trajectory"};
  /// Must match MoveIt `gripper_controller` joints (meters stroke for Piper bridge).
  std::string trajectory_joint_name{"joint7"};
  /**
   * Optional mirror finger joint (e.g. Piper sim `joint8`). When the
   * `gripper_controller` owns two prismatic finger joints and rejects
   * partial-joint goals (`allow_partial_joints_goal:=false`, the ros2_control
   * default), a joint7-only goal is refused and the gripper never moves. Set
   * this to the second joint so both fingers are commanded together; its
   * position is `trajectory_mirror_sign * stroke` (Piper: joint8 = -joint7).
   * Empty => single-joint goal (unchanged behaviour).
   */
  std::string trajectory_mirror_joint_name{};
  double trajectory_mirror_sign{-1.0};
  /**
   * Optional second FollowJointTrajectory action for the mirror finger when sim
   * splits joint7/joint8 across gripper_controller + gripper8_controller.
   * When set, arm_api2 sends joint7 and joint8 goals together on each open/close.
   */
  std::string mirror_trajectory_action{};
  std::string mirror_trajectory_joint_name{"joint8"};
  double trajectory_time_from_start_sec{0.2};
  /**
   * When true, reflect Robotiq-normalized commands (0≈open, 0.8≈closed) before mapping to joint stroke,
   * and reflect stroke→normalized on feedback. Use when hardware/driver stroke grows in the opposite
   * direction so GUI Open/Close match physical motion.
   */
  bool invert_robotiq_normalized{false};
};

/**
 * Mirrors RobotiqGripper helpers used by arm_api2: normalized position uses
 * Robotiq semantics (0 ≈ fully open, 0.8 ≈ fully closed) and is mapped linearly to
 * [open_stroke_m, close_stroke_m] for Piper's `gripper` JointState axis (meters).
 */
class PiperJointGripper
{
public:
  explicit PiperJointGripper(std::shared_ptr<rclcpp::Node> node);

  void configure(const PiperJointGripperConfig & config);

  bool send_gripper_command(double normalized_position_robotiq, double max_effort = 140.0);

  float get_position() const;

  float get_effort() const;

  bool is_stalled() const;

  bool reached_goal() const;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFj = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool try_wait_for_feedback(std::chrono::milliseconds total_wait) const;

  [[nodiscard]] std::string resolve_action_name(const std::string & action) const;

  bool send_gripper_command_joint_state(
    double normalized_position_robotiq, double max_effort);

  bool send_gripper_command_trajectory(double stroke_m, double max_effort);

  /// Stroke in meters from feedback (prefer `joint7` in trajectory setups, else `gripper` / legacy index 6).
  [[nodiscard]] size_t stroke_feedback_index(const std::vector<std::string> & names) const;

  std::shared_ptr<rclcpp::Node> node_;
  PiperJointGripperConfig cfg_{};
  bool configured_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr fjt_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr mirror_fjt_client_;

  mutable std::mutex state_mtx_;
  sensor_msgs::msg::JointState last_state_;
  bool have_state_{false};

  std::atomic<float> last_cmd_normalized_{0.0F};
  std::atomic<float> last_effort_{0.0F};
  std::atomic<bool> traj_done_{false};
  std::atomic<bool> traj_success_{false};
};

#endif  // ARM_API2_PIPER_JOINT_GRIPPER_HPP_
