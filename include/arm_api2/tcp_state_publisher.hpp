// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2024-2026 Crobotic Solutions d.o.o.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ARM_API2__TCP_STATE_PUBLISHER_HPP_
#define ARM_API2__TCP_STATE_PUBLISHER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace arm_api2
{

enum class TcpVelocitySource
{
  Auto,              // J(q)·q̇ while joint velocities agree with pose differences, else differences
  JointVelocity,     // J(q)·q̇ only (driver must publish real joint velocities)
  FiniteDifference,  // differentiate consecutive TCP poses
};

struct TcpStateConfig
{
  bool enabled{true};
  std::string joint_states_topic{"joint_states"};
  std::string pose_topic{"arm/state/tcp_pose"};
  std::string twist_topic{"arm/state/tcp_twist"};
  std::string group;
  std::string tip_link;
  TcpVelocitySource velocity_source{TcpVelocitySource::Auto};
  /* low-pass time constant for finite-difference twist [s]; 0 disables */
  double filter_tau_s{0.02};
  /* gaps longer than this restart differentiation instead of producing a bogus spike [s] */
  double max_dt_s{0.5};
};

/* 6-vector twist [vx vy vz wx wy wz] of a frame moving from prev to curr in dt. */
Eigen::Matrix<double, 6, 1> finiteDifferenceTwist(
  const Eigen::Isometry3d & prev, const Eigen::Isometry3d & curr, double dt);

/**
 * Publishes the TCP pose and twist (both in the robot model frame, twist referenced at the TCP
 * origin) on every joint_states message. The subscription lives on the internal MoveIt node,
 * whose MultiThreadedExecutor keeps running while m2Iface blocks on synchronous actions.
 */
class TcpStatePublisher
{
public:
  TcpStatePublisher(
    const rclcpp::Node::SharedPtr & sub_node, rclcpp::Node & pub_node,
    const moveit::core::RobotModelConstPtr & model, const TcpStateConfig & cfg);

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  std::optional<Eigen::Matrix<double, 6, 1>> jacobianTwist(
    const sensor_msgs::msg::JointState & msg);

  TcpStateConfig cfg_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  moveit::core::RobotModelConstPtr model_;
  moveit::core::RobotState state_;
  const moveit::core::JointModelGroup * group_{nullptr};
  const moveit::core::LinkModel * tip_link_model_{nullptr};
  std::string frame_id_;

  bool have_prev_{false};
  Eigen::Isometry3d prev_pose_{Eigen::Isometry3d::Identity()};
  rclcpp::Time prev_stamp_;
  Eigen::Matrix<double, 6, 1> fd_filtered_{Eigen::Matrix<double, 6, 1>::Zero()};
  int joint_velocity_mismatch_count_{0};
  bool joint_velocity_trusted_{true};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};

}  // namespace arm_api2

#endif  // ARM_API2__TCP_STATE_PUBLISHER_HPP_
