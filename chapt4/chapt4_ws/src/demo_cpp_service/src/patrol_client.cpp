#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<turtlesim/msg/pose.hpp>
#include<chrono>
#include<chapt4_interfaces/srv/patrol.hpp>
#include<ctime>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
#include<rcl_interfaces/msg/parameter.hpp>
#include<rcl_interfaces/msg/parameter_type.hpp>
#include<rcl_interfaces/msg/parameter_value.hpp>
#include<rcl_interfaces/srv/set_parameters.hpp>
using Patrol = chapt4_interfaces::srv::Patrol;
using SetParameters = rcl_interfaces::srv::SetParameters;
using namespace std::chrono_literals;
class TurtlePatrolClient : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::Client<SetParameters>::SharedPtr param_client_;
public:
    explicit TurtlePatrolClient(const std::string &node_name) : Node(node_name)
    {
        patrol_client_ = this->create_client<Patrol>("/turtle1/patrol");
        param_client_ = this->create_client<SetParameters>("/turtle_control/set_parameters");
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtlePatrolClient::send_patrol, this));
        srand(time(NULL));
    }
    
    void send_patrol()
    {
        while(this->patrol_client_->wait_for_service(1s) == false)
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "rclcpp is shutdown, exiting");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "patrol service not available, waiting again...");

        }
        Patrol::Request::SharedPtr request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 12;
        request->target_y = rand() % 12;
        RCLCPP_INFO(this->get_logger(), "Sending patrol request: (%.2f, %.2f)", request->target_x, request->target_y);
        patrol_client_->async_send_request(request, std::bind(&TurtlePatrolClient::send_target_callback, this, std::placeholders::_1));
    }

    void send_target_callback(rclcpp::Client<Patrol>::SharedFuture future)
    {
        Patrol::Response::SharedPtr response = future.get();
        if(response->result == Patrol::Response::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Request for target success");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Request for target failed");
        }
    }
    void set_double_param(std::string param_name, double param_value)
    {
        rcl_interfaces::msg::Parameter parameter;
        parameter.name = param_name;
        parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        parameter.value.double_value = param_value;
        call_set_parameters(parameter);
    }
    void call_set_parameters(const rcl_interfaces::msg::Parameter &parameter)
    {
        while(this->param_client_->wait_for_service(1s) == false)
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "rclcpp is shutdown, exiting");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "param service not available, waiting again...");
        }
        SetParameters::Request::SharedPtr request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(parameter);
        std::function<void (rclcpp::Client<SetParameters>::SharedFuture)> callback = [&](rclcpp::Client<SetParameters>::SharedFuture future)->void{
            SetParameters::Response::SharedPtr response = future.get();
        };
        this->param_client_->async_send_request(request, callback);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TurtlePatrolClient> turtle_patrol_client = std::make_shared<TurtlePatrolClient>("turtle_patrol_client");
    turtle_patrol_client->set_double_param("p_k", 0.3);
    rclcpp::spin(turtle_patrol_client);
    rclcpp::shutdown();
    return 0;
}