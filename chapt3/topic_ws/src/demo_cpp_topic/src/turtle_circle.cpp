#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<chrono>
using namespace std::chrono_literals;

class TurtleCircleNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    explicit TurtleCircleNode(const std::string &node_name) : Node(node_name)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircleNode::publish_cmd, this));
    }

    void publish_cmd()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 1.0;
        cmd.angular.z = 1.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TurtleCircleNode> turtle_cicle_node = std::make_shared<TurtleCircleNode>("turtle_circle");
    rclcpp::spin(turtle_cicle_node);
    rclcpp::shutdown();
    return 0;
}