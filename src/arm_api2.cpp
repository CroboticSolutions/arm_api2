#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

#define GROUP "arm"
#define PLANNING_FRAME "base_link"
#define EE_LINK_NAME "panda_link8"

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
    msg.orientation.y = 0.8; 
    msg.orientation.w = 0.6; 
    msg.position.x = 0.1; 
    msg.position.y = 0.4; 
    msg.position.z = 0.4; 
    return msg; 
  }(); 

  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] 
  {
    moveit_msgs::msg::CollisionObject collision_object; 
    collision_object.header.frame_id = frame_id; 
    collision_object.id = "box1"; 
    shape_msgs::msg::SolidPrimitive primitive; 

    // Define the size of the box in meters
    primitive.type = primitive.BOX; 
    primitive.dimensions.resize(3); 
    primitive.dimensions[primitive.BOX_X] = 0.5; 
    primitive.dimensions[primitive.BOX_Y] = 0.1; 
    primitive.dimensions[primitive.BOX_Z] = 0.5; 

    // Define the pose of the box (relative to the frame)
    geometry_msgs::msg::Pose box_pose; 
    box_pose.orientation.w = 1.0; 
    box_pose.position.x = 0.2; 
    box_pose.position.y = 0.2; 
    box_pose.position.z = 0.25; 

    collision_object.primitives.push_back(primitive); 
    collision_object.primitive_poses.push_back(box_pose); 
    collision_object.operation = collision_object.ADD; 

    return collision_object; 
  }(); 

  // Add the collision object to the scene 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
  planning_scene_interface.applyCollisionObject(collision_object); 
                                    

  // setPoseTarget
  move_group_interface.setPoseTarget(target_pose); 
  // print current pose
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

  double time = node->now().seconds();

  RCLCPP_INFO(node->get_logger(), "Current time is: %f", time); 

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg; 
    auto const ok = static_cast<bool>(move_group_interface.plan(msg)); 
    return std::make_pair(ok, msg);
  }(); 

  // Execute the plan 
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
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
