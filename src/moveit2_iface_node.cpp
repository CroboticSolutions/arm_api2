#include "arm_api2/moveit2_iface.hpp"

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 

    // Create node 
    rclcpp::Node::SharedPtr node = std::make_shared<m2Iface>(); 
    
    // Add multithreaded executor --> Check MoveIt states for multithreaded executor and relation with time 
    //rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node); 
    executor.spin(); 

    // Test like this :) [Without executors]
    //rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0; 

}