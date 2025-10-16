#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<turtlesim/msg/pose.hpp>
#include<chrono>
using namespace std::chrono_literals;

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    float target_x_ = 1.0;
    float target_y_ = 1.0;
    float target_theta_ = 1.0;
    float p_k_ = 0.5;
    float max_speed_ = 3.0;
public:
    explicit TurtleControlNode(const std::string &node_name) : Node(node_name)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleControlNode::pose_callback, this, std::placeholders::_1));
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose_msg)
    {
        float current_x = pose_msg->x;
        float current_y = pose_msg->y;
        float current_theta = pose_msg->theta;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = target_x_ - current_x;
        cmd.linear.y = target_y_ - current_y;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TurtleControlNode> turtle_control_node = std::make_shared<TurtleControlNode>("turtle_control");
    rclcpp::spin(turtle_control_node);
    rclcpp::shutdown();
    return 0;
}