#include "arm_api2/moveit2_iface.hpp"

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 

    // Create node 
    rclcpp::Node::SharedPtr node = std::make_shared<m2Iface>(); 
    
    // Add multithreaded executor --> Check MoveIt states for multithreaded executor and relation with time 
    //rclcpp::executors::MultiThreadedExecutor executor;
    //rclcpp::executors::SingleThreadedExecutor executor; 
    //executor.add_node(node); 
    //executor.spin(); 

    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::Rate loop_rate(1000);
    while (rclcpp::ok())
    {
        executor->spin_node_once(node);
        loop_rate.sleep();
    }


    // Test like this :) [Without executors]
    //rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0; 

}