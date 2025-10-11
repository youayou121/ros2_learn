#include"rclcpp/rclcpp.hpp"
#include<iostream>
int main(int argc, char * argv[])
{   
    std::cout<<"argnum: " << argc << std::endl;
    for(int i = 0; i < argc; i++)
    {
        std::cout<<"arg" << i << ": " << argv[i] << std::endl;
    }
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(), "Hello ROS2 from C++");
    rclcpp::spin(node);
    return 0;
}