#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define GROUP "panda_arm"
#define PLANNING_FRAME "base_link"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_api2",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("arm_api2");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS 
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node); 
  auto spinner = std::thread([&executor] () {executor.spin(); }); 

  // Initi move_group interface
  using moveit::planning_interface::MoveGroupInterface; 
  auto move_group_interface = MoveGroupInterface(node, GROUP); 

  // Construct and initialize MoveItVisualTools 
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, PLANNING_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC, 
    move_group_interface.getRobotModel()};
     
  moveit_visual_tools.deleteAllMarkers(); 
  moveit_visual_tools.loadRemoteControl(); 

  // Useful to show state of the program at high level at 1m height
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity(); 
      msg.translation().z() = 1.0; 
      return msg; 
    }(); 
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, 
                                    rviz_visual_tools::XLARGE); 
  }; 

  // This function blocks program until the USER press next button in rviz
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text); 
  }; 

  // Draw trajectory path
  auto const draw_trajectory_tool_path = 
    [&moveit_visual_tools, 
    jmg = move_group_interface.getRobotModel()->getJointModelGroup(
      GROUP)](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); 
      };

  auto const target_pose = []{
    geometry_msgs::msg::Pose msg; 
    msg.orientation.w = 1.0; 
    msg.position.x = 0.28; 
    msg.position.y = -0.2; 
    msg.position.z = 0.5; 
    return msg; 
  }(); 

  move_group_interface.setPoseTarget(target_pose); 

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg; 
    auto const ok = static_cast<bool>(move_group_interface.plan(msg)); 
    return std::make_pair(ok, msg);
  }(); 

  // Execute the plan 
  if (success) {
    move_group_interface.execute(plan);
    moveit_visual_tools.trigger(); 
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute"); 
    draw_title("Executing"); 
    moveit_visual_tools.trigger(); 
    move_group_interface.execute(plan); 
  } else {
    draw_title("Planning failed!"); 
    moveit_visual_tools.trigger(); 
    RCLCPP_ERROR(logger, "Planning failed!"); 
  }

  // Shutdown ROS
  rclcpp::shutdown();
  // Ending initialized ROS thread
  spinner.join(); 
  return 0;
}
