#include "arm_api2/moveit2_iface.hpp"

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 
    // Set node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.use_intra_process_comms(false); 

    // Create node 
    auto move_group_node = std::make_shared<m2Iface>(node_options); 
    rclcpp::spin(move_group_node);

    // Test like this :) [Without executors]
    //rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0; 

}