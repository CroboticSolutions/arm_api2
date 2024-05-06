#include "arm_api2/moveit2_iface.hpp"

m2Iface::m2Iface(): Node("moveit2_iface")
{   
    // devel --> use demo from moveit2_tutorials

    // necessary constants
    PLANNING_GROUP = std::string("panda_arm"); 
    auto node_ = std::make_shared<rclcpp::Node>(this->get_name(), 
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    //robotDescLoaded = loadRobotDesc
    moveGroupInit = setMoveGroup(node_, PLANNING_GROUP); 

    // robot Model Loader --> fix this part
    /* robot_model_loader::RobotModelLoader robot_model_loader(node_);    
    kinematic_model = new robot_model_loader.getModel();
    robot_state = new moveit::core::RobotState(kinematic_model);
    RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str()); */

    ns_ = this->get_namespace(); 	
    // TODO: Load yaml path from param
    config = init_config("/root/ws_moveit2/src/arm_api2/config/franka.yaml"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");
    init_publishers(); 
    init_subscribers(); 

    // TODO: Add as reconfigurable param
    std::chrono::duration<double> SYSTEM_DT(0.05);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    nodeInit = true; 
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{   
    return YAML::LoadFile(yaml_path);
}

void m2Iface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2Iface::pose_cmd_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped pose_cmd; 
    pose_cmd.pose = msg->pose; 
    std::cout << "Reciv msg" << std::endl; 
}

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName)
{
    // set mGroupIface 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(nodePtr, groupName);
    
    return true; 
}

void m2Iface::run()
{
    if(nodeInit)
    {
        if(moveGroupInit)
        {   
            // enable this part
            /*const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
            const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

            std::vector<double> joint_values;
            robot_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            robot_state->setJointGroupPositions(joint_model_group, joint_values);
            robot_state->enforceBounds();

            robot_state->setToRandomPositions(joint_model_group);
            const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("panda_link8");*/



            /* Print end-effector pose. Remember that this is in the model frame */
            /* RCLCPP_INFO_STREAM(this->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
            RCLCPP_INFO_STREAM(this->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n"); */
            RCLCPP_INFO_STREAM(this->get_logger(), "Running..."); 

        }
    }
}
