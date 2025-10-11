// #include<rclcpp/rclcpp.hpp>
#include<iostream>
int main(int argc, char * argv[])
{   
    std::cout<<"argnum: " << argc << std::endl;
    for(int i = 0; i < argc; i++)
    {
        std::cout<<"arg" << i << ": " << argv[i] << std::endl;
    }
    return 0;
}