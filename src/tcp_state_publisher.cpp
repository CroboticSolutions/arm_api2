#include "arm_api2/tcp_state_publisher.hpp"

#include <cmath>
#include <functional>
#include <unordered_map>

namespace arm_api2
{

namespace
{
/* Auto mode drops joint velocities after this many consecutive messages that disagree with the
 * pose differences (e.g. mock hardware publishing zero velocity while the arm moves). */
constexpr int kMismatchMessages = 50;

bool twistsDisagree(const Eigen::Matrix<double, 6, 1> & jac, const Eigen::Matrix<double, 6, 1> & fd)
{
  const double lin_fd = fd.head<3>().norm();
  const double ang_fd = fd.tail<3>().norm();
  return (jac.head<3>() - fd.head<3>()).norm() > 0.002 + 0.3 * lin_fd ||
         (jac.tail<3>() - fd.tail<3>()).norm() > 0.02 + 0.3 * ang_fd;
}

geometry_msgs::msg::TwistStamped toTwistMsg(
  const Eigen::Matrix<double, 6, 1> & v, const std::string & frame_id, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.twist.linear.x = v(0);
  msg.twist.linear.y = v(1);
  msg.twist.linear.z = v(2);
  msg.twist.angular.x = v(3);
  msg.twist.angular.y = v(4);
  msg.twist.angular.z = v(5);
  return msg;
}
}  // namespace

Eigen::Matrix<double, 6, 1> finiteDifferenceTwist(
  const Eigen::Isometry3d & prev, const Eigen::Isometry3d & curr, double dt)
{
  Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
  if (dt <= 0.0) {
    return v;
  }
  v.head<3>() = (curr.translation() - prev.translation()) / dt;
  /* rotation increment expressed in the base frame: R_curr = dR * R_prev */
  const Eigen::AngleAxisd d_rot(curr.linear() * prev.linear().transpose());
  v.tail<3>() = d_rot.axis() * d_rot.angle() / dt;
  return v;
}

TcpStatePublisher::TcpStatePublisher(
  const rclcpp::Node::SharedPtr & sub_node, rclcpp::Node & pub_node,
  const moveit::core::RobotModelConstPtr & model, const TcpStateConfig & cfg)
: cfg_(cfg), logger_(pub_node.get_logger().get_child("tcp_state")),
  clock_(pub_node.get_clock()), model_(model), state_(model), prev_stamp_(0, 0,
    clock_->get_clock_type())
{
  state_.setToDefaultValues();
  frame_id_ = model_->getModelFrame();
  group_ = model_->getJointModelGroup(cfg_.group);
  if (model_->hasLinkModel(cfg_.tip_link)) {
    tip_link_model_ = model_->getLinkModel(cfg_.tip_link);
  }
  if (!state_.knowsFrameTransform(cfg_.tip_link)) {
    RCLCPP_ERROR(logger_, "TCP frame '%s' is not in the robot model; TCP state disabled",
      cfg_.tip_link.c_str());
    return;
  }
  if (cfg_.velocity_source == TcpVelocitySource::JointVelocity && (!group_ || !tip_link_model_)) {
    RCLCPP_WARN(logger_,
      "velocity_source joint_velocity needs group '%s' and link '%s'; using finite difference",
      cfg_.group.c_str(), cfg_.tip_link.c_str());
    cfg_.velocity_source = TcpVelocitySource::FiniteDifference;
  }

  pose_pub_ = pub_node.create_publisher<geometry_msgs::msg::PoseStamped>(cfg_.pose_topic, 10);
  twist_pub_ = pub_node.create_publisher<geometry_msgs::msg::TwistStamped>(cfg_.twist_topic, 10);

  cb_group_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = cb_group_;
  sub_ = sub_node->create_subscription<sensor_msgs::msg::JointState>(
    cfg_.joint_states_topic, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&TcpStatePublisher::jointStateCb, this, std::placeholders::_1), opts);

  RCLCPP_INFO(logger_, "Publishing %s and %s for '%s' in '%s' from %s",
    pose_pub_->get_topic_name(), twist_pub_->get_topic_name(), cfg_.tip_link.c_str(),
    frame_id_.c_str(), sub_->get_topic_name());
}

std::optional<Eigen::Matrix<double, 6, 1>> TcpStatePublisher::jacobianTwist(
  const sensor_msgs::msg::JointState & msg)
{
  if (!group_ || !tip_link_model_ || msg.velocity.size() != msg.name.size()) {
    return std::nullopt;
  }
  std::unordered_map<std::string, double> vel_by_name;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    vel_by_name[msg.name[i]] = msg.velocity[i];
  }
  const auto & joints = group_->getActiveJointModels();
  Eigen::VectorXd qdot(group_->getVariableCount());
  Eigen::Index k = 0;
  for (const auto * jm : joints) {
    if (jm->getVariableCount() != 1) {
      return std::nullopt;
    }
    const auto it = vel_by_name.find(jm->getName());
    if (it == vel_by_name.end() || !std::isfinite(it->second)) {
      return std::nullopt;
    }
    qdot(k++) = it->second;
  }
  if (k != qdot.size()) {
    return std::nullopt;
  }

  Eigen::MatrixXd jac;
  if (!state_.getJacobian(group_, tip_link_model_, Eigen::Vector3d::Zero(), jac) ||
    jac.cols() != qdot.size())
  {
    return std::nullopt;
  }
  /* getJacobian is expressed in the group's root link frame; rotate into the model frame */
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const auto * root_link = group_->getJointModels().front()->getParentLinkModel();
  if (root_link) {
    rot = state_.getGlobalLinkTransform(root_link).linear();
  }
  const Eigen::Matrix<double, 6, 1> v_root = jac * qdot;
  Eigen::Matrix<double, 6, 1> v;
  v.head<3>() = rot * v_root.head<3>();
  v.tail<3>() = rot * v_root.tail<3>();
  return v;
}

void TcpStatePublisher::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  bool any = false;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    const auto * jm = model_->getJointModel(msg->name[i]);
    if (!jm || jm->getVariableCount() != 1 || !std::isfinite(msg->position[i])) {
      continue;
    }
    state_.setVariablePosition(jm->getFirstVariableIndex(), msg->position[i]);
    any = true;
  }
  if (!any) {
    return;
  }
  state_.update();

  rclcpp::Time stamp(msg->header.stamp, clock_->get_clock_type());
  if (stamp.nanoseconds() == 0) {
    stamp = clock_->now();
  }
  const Eigen::Isometry3d pose = state_.getFrameTransform(cfg_.tip_link);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id_;
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position.x = pose.translation().x();
  pose_msg.pose.position.y = pose.translation().y();
  pose_msg.pose.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  pose_pub_->publish(pose_msg);

  /* finite difference: always tracked so Auto can cross-check joint velocities */
  std::optional<Eigen::Matrix<double, 6, 1>> fd;
  if (have_prev_) {
    const double dt = (stamp - prev_stamp_).seconds();
    if (dt <= 0.0) {
      return;  // duplicate / reordered message: keep previous sample
    }
    if (dt <= cfg_.max_dt_s) {
      const auto raw = finiteDifferenceTwist(prev_pose_, pose, dt);
      const double alpha = cfg_.filter_tau_s > 0.0 ? dt / (cfg_.filter_tau_s + dt) : 1.0;
      fd_filtered_ += alpha * (raw - fd_filtered_);
      fd = fd_filtered_;
    } else {
      fd_filtered_.setZero();
    }
  }
  have_prev_ = true;
  prev_pose_ = pose;
  prev_stamp_ = stamp;

  std::optional<Eigen::Matrix<double, 6, 1>> twist;
  switch (cfg_.velocity_source) {
    case TcpVelocitySource::FiniteDifference:
      twist = fd;
      break;
    case TcpVelocitySource::JointVelocity:
      twist = jacobianTwist(*msg);
      break;
    case TcpVelocitySource::Auto: {
        const auto jac = joint_velocity_trusted_ ? jacobianTwist(*msg) : std::nullopt;
        if (jac && fd) {
          joint_velocity_mismatch_count_ =
            twistsDisagree(*jac, *fd) ? joint_velocity_mismatch_count_ + 1 : 0;
          if (joint_velocity_mismatch_count_ >= kMismatchMessages) {
            joint_velocity_trusted_ = false;
            RCLCPP_WARN(logger_,
              "joint_states velocities disagree with TCP motion for %d messages; "
              "switching TCP twist to finite difference", kMismatchMessages);
          }
        }
        twist = (jac && joint_velocity_trusted_) ? jac : fd;
        break;
      }
  }
  if (twist) {
    twist_pub_->publish(toTwistMsg(*twist, frame_id_, stamp));
  }
}

}  // namespace arm_api2
