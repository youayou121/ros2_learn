#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<turtlesim/msg/pose.hpp>
#include<chrono>
#include<chapt4_interfaces/srv/patrol.hpp>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Service<Patrol>::SharedPtr patrol_srv_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    float target_x_ = 1.0;
    float target_y_ = 1.0;
    float target_theta_ = 1.0;
    float p_k_ = 0.5;
    float max_speed_ = 3.0;
    bool get_first_target_ = false;
public:
    explicit TurtleControlNode(const std::string &node_name) : Node(node_name)
    {
        patrol_srv_ = this->create_service<Patrol>("/turtle1/patrol", std::bind(&TurtleControlNode::patrol_request_callback, this, std::placeholders::_1, std::placeholders::_2));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleControlNode::pose_callback, this, std::placeholders::_1));
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TurtleControlNode::param_callback, this, std::placeholders::_1));
        this->declare_parameter("p_k", 1.0);
        this->get_parameter("p_k", p_k_);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose_msg)
    {
        float current_x = pose_msg->x;
        float current_y = pose_msg->y;
        float current_theta = pose_msg->theta;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = (target_x_ - current_x) * p_k_;
        cmd.linear.y = (target_y_ - current_y) * p_k_;
        cmd.angular.z = 0.0;
        if(get_first_target_)
            cmd_pub_->publish(cmd);
    }

    void patrol_request_callback(const Patrol::Request::SharedPtr request, const Patrol::Response::SharedPtr response)
    {
        if(request->target_x < 0.0 || request->target_x > 11.0 || request->target_y < 0.0 || request->target_y > 11.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Patrol request out of bounds: (%.2f, %.2f)", request->target_x, request->target_y);
            response->result = Patrol::Response::FAIL;
        }
        else
        {
            this->target_x_ = request->target_x;
            this->target_y_ = request->target_y;
            response->result = Patrol::Response::SUCCESS;
            get_first_target_ = true;
        }
        return;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for(const rclcpp::Parameter &param : parameters)
        {
            if(param.get_name() == "p_k")
            {
                p_k_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter p_k updated to: %.2f", p_k_);
            }
        }
        return result;
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